"""Calibration 6 faces de l'accéléromètre — à faire une fois par capteur.

Protocole : enregistrer une capture où le capteur est posé immobile quelques
secondes sur chacune de ses 6 faces (ordre libre, inclinaisons approximatives
tolérées). Le script repère les plateaux immobiles, identifie pour chaque axe
les faces où il voit la gravité en positif et en négatif, et en déduit le
décalage (offset) et le gain de chaque axe :

    gain_i = (lecture_i+ - lecture_i-) / (2 g)      offset_i = (lecture_i+ + lecture_i-) / 2

Résultat écrit dans capteur_calibration.json, appliqué automatiquement par
analyse_reps.py à toutes les analyses suivantes.

Usage :
    python calibre_6faces.py [capture.csv]
"""

import json
import os
import sys
import numpy as np

G = 9.80665
SEUIL_STD_ACCEL = 0.15
SEUIL_STD_GYRO = 5.0
DUREE_PLATEAU_S = 1.5


def moyenne_glissante(x, n):
    noyau = np.ones(n) / n
    xp = np.pad(x, (n // 2, n - 1 - n // 2), mode="edge")
    return np.convolve(xp, noyau, mode="valid")


def std_glissant(x, n):
    m = moyenne_glissante(x, n)
    m2 = moyenne_glissante(x ** 2, n)
    return np.sqrt(np.maximum(m2 - m ** 2, 0.0))


def plages_vraies(masque):
    d = np.diff(masque.astype(np.int8), prepend=0, append=0)
    return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1)))


def main():
    chemin = sys.argv[1] if len(sys.argv) > 1 else "essai_main.csv"
    d = np.genfromtxt(chemin, delimiter=",", names=True, comments="#")
    t = d["t_ms"] / 1000.0
    fs = 1.0 / np.median(np.diff(t))
    accel = np.column_stack([d["ax"], d["ay"], d["az"]])
    gyro = np.column_stack([d["gx"], d["gy"], d["gz"]])

    n_fen = int(0.25 * fs)
    immobile = (std_glissant(np.linalg.norm(accel, axis=1), n_fen) < SEUIL_STD_ACCEL) & \
               (std_glissant(np.linalg.norm(gyro, axis=1), n_fen) < SEUIL_STD_GYRO)

    plateaux = []
    for (i0, i1) in plages_vraies(immobile):
        if (i1 - i0) / fs < DUREE_PLATEAU_S:
            continue
        m = accel[i0:i1].mean(axis=0)
        plateaux.append({"t": (t[i0], t[min(i1, len(t) - 1)]), "m": m,
                         "norme": float(np.linalg.norm(m))})

    if len(plateaux) < 4:
        raise SystemExit(f"Seulement {len(plateaux)} plateaux immobiles ≥ "
                         f"{DUREE_PLATEAU_S} s trouvés — il en faut au moins 4 "
                         "faces différentes. Refaire la capture en posant plus longtemps.")

    print(f"{len(plateaux)} plateaux immobiles trouvés :")
    for p in plateaux:
        print(f"  t={p['t'][0]:7.1f}-{p['t'][1]:7.1f}s  accel=({p['m'][0]:+7.3f}, "
              f"{p['m'][1]:+7.3f}, {p['m'][2]:+7.3f})  norme={p['norme']:.3f}")

    gains, offsets, details = [1.0, 1.0, 1.0], [0.0, 0.0, 0.0], []
    for i, nom in enumerate("XYZ"):
        # faces où l'axe i domine (>85 % de la norme), côté + et côté -
        dom = [p for p in plateaux if abs(p["m"][i]) / p["norme"] > 0.85]
        plus = [p for p in dom if p["m"][i] > 0]
        moins = [p for p in dom if p["m"][i] < 0]
        if plus and moins:
            r_plus = max(p["m"][i] for p in plus)
            r_moins = min(p["m"][i] for p in moins)
            gains[i] = (r_plus - r_moins) / (2 * G)
            offsets[i] = (r_plus + r_moins) / 2
            details.append(f"  {nom} : +g vu à {r_plus:+.3f}, -g vu à {r_moins:+.3f} "
                           f"-> gain {gains[i]:.4f}, offset {offsets[i]:+.4f} m/s²")
        elif plus or moins:
            p = (plus or moins)[0]
            offsets[i] = p["m"][i] - np.sign(p["m"][i]) * G
            details.append(f"  {nom} : une seule face vue -> gain supposé 1, "
                           f"offset {offsets[i]:+.4f} m/s² (approx.)")
        else:
            details.append(f"  {nom} : AUCUNE face dominante — axe non calibré !")

    print("\nCalibration par axe :")
    for l in details:
        print(l)

    # contrôle : normes corrigées de tous les plateaux
    residus = []
    for p in plateaux:
        c = (p["m"] - np.array(offsets)) / np.array(gains)
        residus.append(abs(np.linalg.norm(c) - G))
    print(f"\nContrôle : écart des normes corrigées à g = "
          f"{np.mean(residus):.3f} m/s² en moyenne, {np.max(residus):.3f} max "
          f"(avant calibration : jusqu'à ~1.5)")

    sortie = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "capteur_calibration.json")
    with open(sortie, "w", encoding="utf-8") as f:
        json.dump({"gain": gains, "offset_m_s2": offsets,
                   "source": os.path.basename(chemin)}, f, indent=2)
    print(f"\nÉcrit : {sortie} — appliqué automatiquement par analyse_reps.py")


if __name__ == "__main__":
    main()
