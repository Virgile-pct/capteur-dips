"""Génère les données du tableau de bord (docs/donnees.js) depuis les captures réelles.

Rejoue chaque séance enregistrée dans la chaîne d'analyse, ajoute le profil
charge-vitesse, la calibration du capteur et les jalons du projet, et écrit le
tout en un seul fichier JS chargé par docs/index.html (pas de fetch : la page
marche aussi bien en local qu'hébergée).

Usage :  python genere_dashboard.py     (depuis analyse/, après chaque séance)
"""

import contextlib
import io
import json
import os
from datetime import date

import numpy as np
import analyse_reps as ar

ICI = os.path.dirname(os.path.abspath(__file__))
SORTIE = os.path.join(ICI, "..", "docs", "donnees.js")

# Séances à publier : (fichier, date, titre, type, lest kg, note)
SEANCES = [
    ("seance_dips_001.csv", "2026-08-05", "Dips contrôlés", "controle", 0,
     "Première séance instrumentée — tempo appliqué"),
    ("seance_dips_002_explosif.csv", "2026-08-05", "Dips explosifs", "explosif", 0,
     "Intention maximale à chaque rep"),
    ("profil_p10kg.csv", "2026-08-05", "Palier +10 kg", "palier", 10,
     "Profil charge-vitesse — point 2"),
    ("profil_p25kg.csv", "2026-08-05", "Palier +25 kg", "palier", 25,
     "Profil charge-vitesse — point 3"),
    ("profil_p40kg.csv", "2026-08-05", "Palier +40 kg", "palier", 40,
     "Profil charge-vitesse — point 4"),
]


def analyser(fichier):
    with contextlib.redirect_stdout(io.StringIO()):
        return ar.analyser_fichier(os.path.join(ICI, fichier))


def zone_perte(p):
    if p < 10: return "bonne"
    if p < 20: return "attention"
    if p < 30: return "serieuse"
    return "critique"


def main():
    donnees = {"genere_le": date.today().isoformat(), "seances": []}

    for fichier, d, titre, type_, lest, note in SEANCES:
        chemin = os.path.join(ICI, fichier)
        if not os.path.exists(chemin):
            print(f"  (absent, ignoré : {fichier})")
            continue
        reps = analyser(fichier)
        best = max(r["v_moy"] for r in reps)
        liste = []
        for r in reps:
            perte = (1 - r["v_moy"] / best) * 100
            liste.append({
                "v_moy": round(r["v_moy"], 3), "v_pic": round(r["v_pic"], 3),
                "rom_cm": round(r["rom"] * 100, 1), "duree_s": round(r["duree"], 2),
                "perte_pct": round(perte, 1), "zone": zone_perte(perte),
            })
        donnees["seances"].append({
            "fichier": fichier, "date": d, "titre": titre, "type": type_,
            "lest_kg": lest, "note": note, "n_reps": len(liste),
            "v_best": round(best, 3),
            "perte_finale_pct": liste[-1]["perte_pct"], "reps": liste,
        })
        print(f"  {titre} : {len(liste)} reps, meilleure {best:.3f} m/s")

    # Profil charge-vitesse : points + régression recalculée
    chemin_profil = os.path.join(ICI, "profil_2026-08-05.json")
    if os.path.exists(chemin_profil):
        with open(chemin_profil, encoding="utf-8") as f:
            profil = json.load(f)
        x = np.array([p["lest_kg"] for p in profil["points"]], float)
        y = np.array([p["v_moy_best"] for p in profil["points"]], float)
        pente, intercept = np.polyfit(x, y, 1)
        y_pred = pente * x + intercept
        r2 = 1 - float(np.sum((y - y_pred) ** 2)) / float(np.sum((y - y.mean()) ** 2))
        donnees["profil"] = {
            "date": profil["date"], "note": profil.get("note", ""),
            "points": [{"lest": float(a), "v": float(b)} for a, b in zip(x, y)],
            "pente": round(float(pente), 5), "v0": round(float(intercept), 3),
            "r2": round(r2, 4),
            "estimations_mvt": {str(m): round((intercept - m) / abs(pente))
                                for m in (0.20, 0.15, 0.10)},
            "vrai_1rm_frais": 70,
        }

    # Calibration du capteur (l'exemplaire réel)
    chemin_cal = os.path.join(ICI, "capteur_calibration.json")
    if os.path.exists(chemin_cal):
        with open(chemin_cal, encoding="utf-8") as f:
            donnees["calibration"] = json.load(f)

    # Contexte statique : références, validation, jalons, feuille de route
    donnees["references"] = {
        "v_controle": 0.65, "v_explosif_best": 0.858, "rom_typique_cm": 36,
        "vrai_1rm_lest": 70, "poids_corps": 62,
        "seuil_force_pct": [15, 20], "seuil_hypertrophie_pct": [25, 30],
    }
    donnees["validation"] = {
        "simulation": {"v_moy_pct": 2.1, "v_pic_pct": 0.8, "rom_pct": 1.0},
        "materiel": "ESP32 + MPU6050 ±8 g, 100 Hz, calibration 6 faces",
    }
    donnees["jalons"] = [
        {"date": "2026-07-19", "t": "Algorithme validé sur simulation (erreur 2,3 %)"},
        {"date": "2026-07-24", "t": "Dépôt open source publié + firmware compilé à blanc"},
        {"date": "2026-08-05", "t": "Premier contact réel : calibration 6 faces, "
                                    "10 dips mesurés, 3 leçons du réel codées"},
        {"date": "2026-08-05", "t": "Premier profil charge-vitesse (R² = 0,995)"},
        {"date": "2026-08-07", "t": "Firmware V2 BLE écrit et compilé (85 % flash)"},
    ]
    donnees["roadmap"] = [
        {"etat": "fait", "t": "V1 filaire : mesure vitesse/ROM/perte validée sur vraies séries"},
        {"etat": "fait", "t": "Calibration 6 faces par exemplaire"},
        {"etat": "fait", "t": "Profil charge-vitesse (outil + première droite)"},
        {"etat": "encours", "t": "V2 sans fil : firmware BLE prêt — attente batterie "
                                 "(LiPo + TP4056 + MT3608 commandés)"},
        {"etat": "avenir", "t": "Palet XIAO ESP32-S3 (20 g, chargeur intégré)"},
        {"etat": "avenir", "t": "V3 : algo embarqué + vibreur au seuil de perte"},
        {"etat": "avenir", "t": "Profil officiel un jour frais + validation vidéo 240 fps"},
        {"etat": "avenir", "t": "Force : barres instrumentées (cellules 50 kg + HX711)"},
    ]

    os.makedirs(os.path.dirname(SORTIE), exist_ok=True)
    with open(SORTIE, "w", encoding="utf-8") as f:
        f.write("window.DONNEES = ")
        json.dump(donnees, f, ensure_ascii=False, indent=1)
        f.write(";\n")
    print(f"OK -> {os.path.abspath(SORTIE)}")


if __name__ == "__main__":
    main()
