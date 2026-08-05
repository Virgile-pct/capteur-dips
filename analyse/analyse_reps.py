"""Analyse VBT d'un enregistrement IMU (réel ou simulé) — le cœur du capteur de dips.

Chaîne de traitement (celle qui sera portée en C++ sur l'ESP32 une fois figée) :
  1. détection des périodes immobiles (variance accéléro + gyro faibles)
  2. calibration sur la 1re période immobile : biais gyro + vecteur gravité
  3. suivi de l'orientation : propagation gyro + recalage accéléro (filtre
     complémentaire sur le vecteur gravité)
  4. accélération linéaire verticale = composante le long de la gravité - g
  5. intégration -> vitesse, avec ZUPT : vitesse forcée à zéro sur chaque
     période immobile, dérive résiduelle retirée linéairement entre deux ancres
  6. découpage en reps (phases concentriques/excentriques) + métriques :
     vitesse moyenne, vitesse pic, ROM, perte de vitesse dans la série

Usage :
    python analyse_reps.py [fichier.csv] [--verite verite_terrain.json]
                           [--plot analyse_reps.png]
"""

import argparse
import json
import os
import numpy as np

G_DEFAUT = 9.80665

# Seuils de la chaîne (à re-régler sur données réelles si besoin)
SEUIL_STD_ACCEL = 0.15      # m/s²  — variabilité accel max d'une fenêtre immobile
SEUIL_STD_GYRO = 5.0        # °/s   — variabilité gyro max ; réglé sur données réelles :
                            #         une « pause » tenue par un humain tremble toujours
                            #         (~2-4 °/s), un mouvement dépasse largement (>10)
# Ancres à deux niveaux (réglées empiriquement sur simulation + 2 séances réelles).
# Une pause LONGUE vaut ancre sans condition. Une pause COURTE n'est une ancre que
# si elle est ENTOURÉE de mouvement franc : un verrouillage bref n'existe qu'entre
# deux reps vigoureuses, alors que les phases lentes d'une rep fatiguée (même
# variance) sont entourées de mouvement mou — c'est le contexte qui les sépare.
DUREE_ANCRE_LONGUE_S = 0.80     # s
DUREE_ANCRE_COURTE_S = 0.18     # s    — les verrouillages explosifs réels durent ~0,2 s
SEUIL_CONTEXTE_ANCRE = 1.0      # m/s² — variance accel à dépasser au voisinage
RAYON_CONTEXTE_S = 0.6          # s    — rayon du voisinage examiné
# (triplet validé par recherche sur grille contre les 3 jeux : simulation 6/6 à
#  4,3 % d'erreur max, série contrôlée 11 reps, série explosive 10 reps)
FENETRE_IMMOBILE_S = 0.25   # s     — fenêtre glissante des critères
SEUIL_V_REP = 0.05          # m/s   — seuil de détection d'une phase de rep
DUREE_MINI_PHASE_S = 0.25   # s
ROM_MINI_M = 0.15           # m     — écarte les micro-mouvements
ROM_MAXI_M = 1.0            # m     — écarte les artefacts d'intégration (aucun
                            #         mouvement de muscu ne déplace le bassin d'1 m)
GAIN_RECALAGE = 0.02        # gain du filtre complémentaire (par échantillon quasi-statique)


def moyenne_glissante(x, n):
    """Moyenne glissante centrée, même taille que x (bords gérés par répétition)."""
    n = max(1, int(n))
    noyau = np.ones(n) / n
    xp = np.pad(x, (n // 2, n - 1 - n // 2), mode="edge")
    return np.convolve(xp, noyau, mode="valid")


def plages_vraies(masque):
    """Liste des (debut, fin exclus) des plages contiguës à True."""
    d = np.diff(masque.astype(np.int8), prepend=0, append=0)
    debuts = np.flatnonzero(d == 1)
    fins = np.flatnonzero(d == -1)
    return list(zip(debuts, fins))


def cumtrapz(y, dt):
    """Intégrale cumulée par trapèzes, en partant de zéro."""
    out = np.zeros_like(y)
    out[1:] = np.cumsum((y[1:] + y[:-1]) * 0.5 * dt)
    return out


def std_norme_glissante(vecteurs, fs):
    """Écart-type glissant de la norme d'un signal 3 axes."""
    x = np.linalg.norm(vecteurs, axis=1)
    n_fen = int(FENETRE_IMMOBILE_S * fs)
    m = moyenne_glissante(x, n_fen)
    m2 = moyenne_glissante(x ** 2, n_fen)
    return np.sqrt(np.maximum(m2 - m ** 2, 0.0))


def detecter_immobilite(accel, gyro_dps, fs):
    """Masque booléen des instants immobiles.

    Critères de VARIABILITÉ (écart-type glissant) sur accel et gyro, jamais de
    niveau absolu : la variance est immunisée contre le biais du gyroscope, qui
    dérive avec la température et ruinerait un seuil absolu.
    """
    return (std_norme_glissante(accel, fs) < SEUIL_STD_ACCEL) & \
           (std_norme_glissante(gyro_dps, fs) < SEUIL_STD_GYRO)


def suivre_gravite(accel, omega_rad, immobile, g_init, g_mes, dt):
    """Vecteur gravité unitaire dans le repère capteur, échantillon par échantillon.

    Propagation : un vecteur fixe du monde vu depuis un repère qui tourne à
    omega évolue selon dg/dt = -omega x g. Recalage : quand la norme de
    l'accéléro est proche de g et que ça tourne peu, l'accéléro pointe la
    gravité -> on tire doucement l'estimation vers lui (filtre complémentaire).
    """
    n = accel.shape[0]
    g_hat = np.empty((n, 3))
    g = g_init / np.linalg.norm(g_init)
    norme_a = np.linalg.norm(accel, axis=1)
    quasi_statique = (np.abs(norme_a - g_mes) < 0.05 * g_mes) & \
                     (np.linalg.norm(omega_rad, axis=1) < np.deg2rad(6.0))
    for k in range(n):
        g = g - dt * np.cross(omega_rad[k], g)
        if quasi_statique[k] or immobile[k]:
            g = (1.0 - GAIN_RECALAGE) * g + GAIN_RECALAGE * accel[k] / norme_a[k]
        g /= np.linalg.norm(g)
        g_hat[k] = g
    return g_hat


def integrer_avec_zupt(a_lin, points_zero, spans_zero, dt):
    """Vitesse intégrée entre des ancres PONCTUELLES où v = 0.

    Épingler le zéro en un point (le cœur de la pause) plutôt que sur toute la
    plage immobile évite de mordre le début des phases : le lissage de la
    détection d'immobilité déborde toujours un peu sur le mouvement voisin.
    Entre deux points, le reliquat de vitesse à l'arrivée mesure la dérive
    (biais accéléro + fuite de gravité), retirée en rampe linéaire — exact pour
    un biais constant. Les longues plages immobiles sont ensuite forcées à zéro.
    """
    v = np.zeros(a_lin.size)
    for i0, i1 in zip(points_zero[:-1], points_zero[1:]):
        if i1 <= i0:
            continue
        seg = cumtrapz(a_lin[i0:i1 + 1], dt)
        derive = seg[-1] / (len(seg) - 1)
        v[i0:i1 + 1] = seg - derive * np.arange(len(seg))
    # Tête et queue du fichier : intégration depuis le point le plus proche,
    # sans correction de dérive (pas de second point pour la contraindre) —
    # nécessaire quand l'enregistrement démarre ou finit en plein mouvement.
    p0, pN = points_zero[0], points_zero[-1]
    if p0 > 0:
        tete = cumtrapz(a_lin[:p0 + 1], dt)
        v[:p0 + 1] = tete - tete[-1]
    if pN < a_lin.size - 1:
        v[pN:] = cumtrapz(a_lin[pN:], dt)
    for (a, b) in spans_zero:
        v[a:b] = 0.0
    return v


def decouper_phases(v, fs, sens):
    """Plages des phases concentriques (sens=+1) ou excentriques (-1).

    Détection par seuil, puis extension des bornes jusqu'à ce que la vitesse
    retombe sous un petit epsilon (2,5 % du pic de la phase) — pas jusqu'au
    zéro strict, sinon on avale les plateaux de vitesse quasi nulle (pause
    basse, tremblements) et la vitesse moyenne est diluée.
    """
    phases, deja = [], set()
    for d, f in plages_vraies(sens * v > SEUIL_V_REP):
        eps = max(0.02, 0.025 * float(np.max(sens * v[d:f])))
        while d > 0 and sens * v[d - 1] > eps:
            d -= 1
        while f < v.size and sens * v[f] > eps:
            f += 1
        if d in deja:
            continue
        deja.add(d)
        duree = (f - d) / fs
        rom = abs(np.sum(v[d:f]) / fs)
        if duree >= DUREE_MINI_PHASE_S and ROM_MINI_M <= rom <= ROM_MAXI_M:
            phases.append((d, f))
    return phases


def analyser_fichier(chemin, verite=None, plot=None):
    """Chaîne d'analyse complète d'un CSV ; renvoie la liste des reps mesurées."""
    # Première ligne : soit l'en-tête CSV, soit le marqueur « # source: simulation »
    # (piège numpy : names=True lirait les noms depuis la ligne commentée — on la saute)
    with open(chemin, encoding="utf-8") as f:
        premiere = f.readline()
    est_simulation = "simulation" in premiere
    d = np.genfromtxt(chemin, delimiter=",", names=True, comments="#",
                      skip_header=1 if premiere.startswith("#") else 0)
    t = d["t_ms"] / 1000.0
    fs = 1.0 / np.median(np.diff(t))
    dt = 1.0 / fs
    accel = np.column_stack([d["ax"], d["ay"], d["az"]])
    gyro_dps = np.column_stack([d["gx"], d["gy"], d["gz"]])

    # Calibration accéléro par capteur (offsets/gains mesurés par calibre_6faces.py).
    # Indispensable dès que le boîtier tourne pendant le geste : un offset fixe dans
    # le repère capteur se projette variablement sur la verticale et fabrique de la
    # fausse vitesse. Sans fichier de calibration : données brutes.
    chemin_cal = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "capteur_calibration.json")
    if est_simulation:
        print("(données simulées : calibration capteur non appliquée)")
    elif os.path.exists(chemin_cal):
        with open(chemin_cal, encoding="utf-8") as f:
            cal = json.load(f)
        accel = (accel - np.array(cal["offset_m_s2"])) / np.array(cal["gain"])
        print(f"(calibration capteur appliquée : {chemin_cal})")

    # 1-2. immobilité + calibration sur la première vraie période immobile
    immobile = detecter_immobilite(accel, gyro_dps, fs)
    plages_imm = [p for p in plages_vraies(immobile)
                  if (p[1] - p[0]) / fs >= DUREE_ANCRE_LONGUE_S]
    if not plages_imm:
        raise SystemExit("Aucune période immobile d'au moins 0,8 s dans tout "
                         "l'enregistrement : impossible de calibrer. Marquer de "
                         "vraies pauses (départ posé, verrouillages tenus).")
    # Calibration sur la plage la plus calme du fichier, pas forcément la
    # première : tolère un départ d'enregistrement raté ou agité.
    ng_tout = np.linalg.norm(gyro_dps, axis=1)
    c0, c1 = min(plages_imm, key=lambda p: ng_tout[p[0]:p[1]].std())
    biais_gyro = gyro_dps[c0:c1].mean(axis=0)
    g_vec0 = accel[c0:c1].mean(axis=0)
    g_mes = float(np.linalg.norm(g_vec0))
    omega_rad = np.deg2rad(gyro_dps - biais_gyro)

    # 3-4. gravité suivie puis retirée -> accélération linéaire verticale (+ = haut)
    g_hat = suivre_gravite(accel, omega_rad, immobile, g_vec0, g_mes, dt)
    a_lin = np.einsum("ij,ij->i", accel, g_hat) - g_mes

    # 5. intégration ZUPT à ancres ponctuelles, deux niveaux : une pause longue
    # donne deux points d'ancrage (près de ses bords, en retrait du lissage) et
    # son intérieur est forcé à zéro ; une pause courte donne un point en son
    # cœur, mais seulement si son voisinage contient du mouvement franc — un
    # vrai verrouillage bref n'existe qu'entre deux reps vigoureuses, alors que
    # les phases lentes d'une rep fatiguée (même variance) baignent dans du mou.
    std_a_gliss = std_norme_glissante(accel, fs)
    rayon = int(RAYON_CONTEXTE_S * fs)
    retrait = max(1, int(0.5 * FENETRE_IMMOBILE_S * fs))
    points_zero, spans_zero, ancres, courtes = [], [], [], []
    for (a, b) in plages_vraies(immobile):
        duree = (b - a) / fs
        if duree >= DUREE_ANCRE_LONGUE_S:
            points_zero += [a + retrait, b - retrait]
            spans_zero.append((a + retrait, b - retrait))
            ancres.append((a, b))
        elif duree >= DUREE_ANCRE_COURTE_S:
            courtes.append((a, b))
            voisinage = std_a_gliss[max(0, a - rayon):min(t.size, b + rayon)]
            if voisinage.max() >= SEUIL_CONTEXTE_ANCRE:
                points_zero.append((a + b) // 2)
                ancres.append((a, b))
    # Privilège de bord : la première/dernière pause courte du fichier vaut
    # ancre même sans contexte — sans point de départ, tout ce qui précède la
    # première ancre resterait à vitesse nulle (reps effacées).
    for (a, b) in courtes:
        c = (a + b) // 2
        if not points_zero or c < min(points_zero):
            points_zero.append(c)
            ancres.append((a, b))
    for (a, b) in reversed(courtes):
        c = (a + b) // 2
        if c > max(points_zero):
            points_zero.append(c)
            ancres.append((a, b))
    points_zero = sorted(set(points_zero))
    if len(points_zero) < 2:
        raise SystemExit("Moins de deux ancres d'immobilité détectées : pas de ZUPT "
                         "possible. Marquer une vraie pause en haut entre les reps.")
    v = integrer_avec_zupt(a_lin, points_zero, spans_zero, dt)
    z = cumtrapz(v, dt)

    # 6. découpage en reps + métriques
    concentriques = decouper_phases(v, fs, +1)
    excentriques = decouper_phases(v, fs, -1)
    reps = []
    for (d0, f0) in concentriques:
        prec = [e for e in excentriques if e[1] <= d0 and (d0 - e[1]) / fs < 3.0]
        reps.append({
            "debut": d0, "fin": f0,
            "v_moy": float(v[d0:f0].mean()),
            "v_pic": float(v[d0:f0].max()),
            "rom": float(np.sum(v[d0:f0]) * dt),
            "duree": (f0 - d0) / fs,
            "duree_exc": (prec[-1][1] - prec[-1][0]) / fs if prec else None,
        })

    if not reps:
        raise SystemExit("Aucune rep détectée.")

    meilleure = max(r["v_moy"] for r in reps)
    print(f"\nFichier : {chemin}  ({t[-1] - t[0]:.1f} s à {fs:.0f} Hz, "
          f"g mesuré {g_mes:.3f} m/s², {len(ancres)} ancres ZUPT)")
    print(f"\n{len(reps)} reps détectées :")
    print("  rep   v_moy    v_pic     ROM   t_conc   perte")
    for i, r in enumerate(reps, 1):
        perte = (1 - r["v_moy"] / meilleure) * 100
        print(f"   {i:2d}   {r['v_moy']:.3f}    {r['v_pic']:.3f}   "
              f"{r['rom']*100:5.1f}cm   {r['duree']:.2f}s   {perte:5.1f}%")

    # Aide à la décision façon doc projet : seuil de perte de vitesse
    derniere_perte = (1 - reps[-1]["v_moy"] / meilleure) * 100
    print(f"\nPerte de vitesse en fin de série : {derniere_perte:.0f}% "
          f"(coupure force ~20%, hypertrophie ~25-30%)")

    # Validation contre la vérité terrain si fournie
    if verite and os.path.exists(verite):
        with open(verite, encoding="utf-8") as f:
            verite = json.load(f)
        if len(verite) == len(reps):
            print("\nValidation contre la vérité terrain :")
            err_moy, err_pic, err_rom = [], [], []
            for r, vt in zip(reps, verite):
                e_m = 100 * (r["v_moy"] - vt["v_moy_m_s"]) / vt["v_moy_m_s"]
                e_p = 100 * (r["v_pic"] - vt["v_pic_m_s"]) / vt["v_pic_m_s"]
                e_r = 100 * (r["rom"] - vt["rom_m"]) / vt["rom_m"]
                err_moy.append(e_m); err_pic.append(e_p); err_rom.append(e_r)
                print(f"   rep {vt['rep']} : v_moy {vt['v_moy_m_s']:.3f} -> "
                      f"{r['v_moy']:.3f} ({e_m:+.1f}%) | v_pic {e_p:+.1f}% | ROM {e_r:+.1f}%")
            for nom, e in [("v_moy", err_moy), ("v_pic", err_pic), ("ROM", err_rom)]:
                print(f"   erreur {nom} : moyenne {np.mean(np.abs(e)):.1f}% | "
                      f"max {np.max(np.abs(e)):.1f}%")
        else:
            print(f"\n(vérité terrain ignorée : {len(verite)} reps attendues, "
                  f"{len(reps)} détectées)")

    # Graphique de contrôle
    if plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(3, 1, sharex=True, figsize=(11, 7))
        axes[0].plot(t, a_lin, lw=0.7)
        axes[0].set_ylabel("a lin. vert. (m/s²)")
        axes[1].plot(t, v, lw=0.9)
        for (d0, f0) in concentriques:
            axes[1].axvspan(t[d0], t[min(f0, len(t) - 1)], alpha=0.25, color="tab:green")
        for (d0, f0) in ancres:
            axes[1].axvspan(t[d0], t[min(f0, len(t) - 1)], alpha=0.15, color="tab:gray")
        axes[1].set_ylabel("vitesse (m/s)")
        axes[2].plot(t, z, lw=0.9)
        axes[2].set_ylabel("position (m)")
        axes[2].set_xlabel("temps (s)")
        axes[0].set_title("Capteur de dips — analyse VBT (vert = concentrique, gris = ancres ZUPT)")
        fig.tight_layout()
        fig.savefig(plot, dpi=130)
        print(f"\nGraphique : {os.path.abspath(plot)}")

    return reps


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("csv", nargs="?", default="reps_simulees.csv")
    ap.add_argument("--verite", default=None, help="JSON de vérité terrain (validation)")
    ap.add_argument("--plot", default="analyse_reps.png")
    args = ap.parse_args()
    analyser_fichier(args.csv, args.verite, args.plot)


if __name__ == "__main__":
    main()
