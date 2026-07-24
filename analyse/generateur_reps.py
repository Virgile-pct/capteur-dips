"""Générateur de données synthétiques — série de dips vue par un MPU6050 virtuel.

Simule une série de 6 dips lestés avec fatigue progressive (la vitesse
concentrique chute rep après rep), puis produit ce que verrait le capteur sur
la ceinture : accélération spécifique dans le repère du boîtier (gravité
comprise, boîtier incliné et qui oscille) + gyroscope, avec bruit et biais
réalistes. Le format de sortie est identique à celui du firmware.

Usage :
    python generateur_reps.py [--sortie reps_simulees.csv]

Produit :
    reps_simulees.csv    mesures brutes (t_ms,ax,ay,az,gx,gy,gz — m/s², °/s)
    verite_terrain.json  vitesses/ROM réels de chaque rep, pour valider l'algo
"""

import argparse
import json
import numpy as np

FS = 100.0            # Hz, comme le firmware
G = 9.80665           # m/s²


def profil_phase(duree_s, rom_m, sens, fs):
    """Vitesse verticale d'une phase de rep : demi-sinusoïde d'aire = ROM.

    v(t) = v_pic * sin(pi*t/T)  ->  v_moy = ROM/T, v_pic = (pi/2)*v_moy.
    sens = +1 pour la montée (concentrique), -1 pour la descente.
    """
    n = max(2, int(round(duree_s * fs)))
    t = np.arange(n) / fs
    v_pic = rom_m * np.pi / (2.0 * duree_s)
    return sens * v_pic * np.sin(np.pi * t / duree_s)


def construire_seance(rng):
    """Concatène les phases d'une série et renvoie v(t) monde + la vérité terrain."""
    rom_nominal = 0.40                                  # m, amplitude d'un dip
    durees_conc = [0.85, 0.92, 1.00, 1.10, 1.22, 1.38]  # s, la fatigue ralentit la montée
    morceaux, verite = [], []

    morceaux.append(np.zeros(int(3.0 * FS)))            # 3 s immobile (calibration)
    for i, t_conc in enumerate(durees_conc):
        rom = rom_nominal * (1 + rng.uniform(-0.02, 0.02))
        t_exc = 1.25 + rng.uniform(-0.1, 0.1)           # descente contrôlée
        morceaux.append(profil_phase(t_exc, rom, -1, FS))
        morceaux.append(np.zeros(int(0.35 * FS)))       # pause basse courte
        morceaux.append(profil_phase(t_conc, rom, +1, FS))
        morceaux.append(np.zeros(int(1.25 * FS)))       # verrouillage haut (ancre ZUPT)
        verite.append({
            "rep": i + 1,
            "v_moy_m_s": round(rom / t_conc, 4),
            "v_pic_m_s": round(rom * np.pi / (2 * t_conc), 4),
            "rom_m": round(rom, 4),
            "duree_conc_s": round(t_conc, 3),
        })
    morceaux.append(np.zeros(int(3.0 * FS)))            # 3 s immobile en fin
    return np.concatenate(morceaux), verite


def matrices_rotation(roll, pitch):
    """R corps->monde (convention ZYX avec lacet nul), vectorisé sur le temps."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    n = roll.size
    R = np.empty((n, 3, 3))
    # R = Ry(pitch) @ Rx(roll)
    R[:, 0, 0], R[:, 0, 1], R[:, 0, 2] = cp, sp * sr, sp * cr
    R[:, 1, 0], R[:, 1, 1], R[:, 1, 2] = 0.0, cr, -sr
    R[:, 2, 0], R[:, 2, 1], R[:, 2, 2] = -sp, cp * sr, cp * cr
    return R


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--sortie", default="reps_simulees.csv")
    ap.add_argument("--graine", type=int, default=42, help="graine aléatoire (reproductible)")
    args = ap.parse_args()
    rng = np.random.default_rng(args.graine)

    v_monde, verite = construire_seance(rng)
    n = v_monde.size
    t = np.arange(n) / FS
    dt = 1.0 / FS

    # Cinématique verticale vraie (monde, z vers le haut)
    a_monde_z = np.gradient(v_monde, dt)

    # Orientation du boîtier : inclinaison fixe + oscillation pendant le mouvement
    enveloppe = np.convolve(np.abs(v_monde) > 0.02, np.ones(60) / 60, mode="same")
    pitch = np.deg2rad(12.0 + 2.5 * enveloppe * np.sin(2 * np.pi * 0.9 * t + 0.7)
                       + 0.3 * np.sin(2 * np.pi * 0.25 * t))
    roll = np.deg2rad(6.0 + 1.8 * enveloppe * np.sin(2 * np.pi * 1.3 * t + 2.1))

    # Gyroscope cohérent avec ces angles (taux corps, lacet nul) :
    # omega = [roll', pitch'*cos(roll), -pitch'*sin(roll)]
    droll, dpitch = np.gradient(roll, dt), np.gradient(pitch, dt)
    omega = np.stack([droll,
                      dpitch * np.cos(roll),
                      -dpitch * np.sin(roll)], axis=1)          # rad/s

    # Accélération spécifique dans le repère du boîtier : f_b = R^T (a_monde - g_monde)
    a_monde = np.zeros((n, 3))
    a_monde[:, 2] = a_monde_z
    a_monde[:, 0] = 0.12 * enveloppe * np.sin(2 * np.pi * 1.1 * t + 0.3)   # léger balancement
    a_monde[:, 1] = 0.10 * enveloppe * np.sin(2 * np.pi * 0.8 * t + 1.9)
    f_monde = a_monde + np.array([0.0, 0.0, G])                 # - g_monde avec g = (0,0,-G)
    R = matrices_rotation(roll, pitch)
    f_corps = np.einsum("nij,nj->ni", R.transpose(0, 2, 1), f_monde)

    # Défauts capteur : biais résiduels + bruit blanc (ordres de grandeur MPU6050)
    accel = (f_corps
             + np.array([0.04, -0.03, 0.05])                    # biais accel, m/s²
             + rng.normal(0, 0.06, (n, 3)))                     # bruit accel
    gyro_dps = (np.rad2deg(omega)
                + np.array([0.40, -0.30, 0.25])                 # biais gyro, °/s
                + rng.normal(0, 0.12, (n, 3)))                  # bruit gyro

    en_tete = "t_ms,ax,ay,az,gx,gy,gz"
    donnees = np.column_stack([np.round(t * 1000).astype(int),
                               np.round(accel, 4), np.round(gyro_dps, 3)])
    np.savetxt(args.sortie, donnees, delimiter=",", header=en_tete, comments="",
               fmt=["%d"] + ["%.4f"] * 3 + ["%.3f"] * 3)

    with open("verite_terrain.json", "w", encoding="utf-8") as f:
        json.dump(verite, f, indent=2, ensure_ascii=False)

    print(f"OK : {n} échantillons ({t[-1]:.1f} s à {FS:.0f} Hz) -> {args.sortie}")
    print("Vérité terrain -> verite_terrain.json")
    for v in verite:
        print(f"  rep {v['rep']} : v_moy {v['v_moy_m_s']:.3f} m/s | "
              f"v_pic {v['v_pic_m_s']:.3f} m/s | ROM {v['rom_m']*100:.1f} cm")


if __name__ == "__main__":
    main()
