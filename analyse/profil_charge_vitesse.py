"""Profil charge-vitesse : régression sur les meilleures reps par palier de lest.

Entrée : un JSON de points {"date": ..., "points": [{"lest_kg": x, "v_moy_best": y}]}
Sortie : pente, R², estimation du lest max pour une gamme de vitesses minimales
(MVT), et un graphique de la droite.

Usage :
    python profil_charge_vitesse.py profil_2026-08-05.json [--vrai-1rm 70]
"""

import argparse
import json
import numpy as np


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("json_points")
    ap.add_argument("--vrai-1rm", type=float, default=None,
                    help="1RM connu (kg de lest), affiché pour comparaison")
    ap.add_argument("--plot", default=None, help="PNG de sortie (défaut : <json>.png)")
    args = ap.parse_args()

    with open(args.json_points, encoding="utf-8") as f:
        data = json.load(f)
    pts = sorted(data["points"], key=lambda p: p["lest_kg"])
    x = np.array([p["lest_kg"] for p in pts], dtype=float)
    y = np.array([p["v_moy_best"] for p in pts], dtype=float)
    if len(x) < 3:
        raise SystemExit("Il faut au moins 3 paliers pour une droite honnête.")

    pente, intercept = np.polyfit(x, y, 1)
    y_pred = pente * x + intercept
    ss_res = float(np.sum((y - y_pred) ** 2))
    ss_tot = float(np.sum((y - y.mean()) ** 2))
    r2 = 1 - ss_res / ss_tot

    print(f"Profil charge-vitesse — {data.get('date', '?')} "
          f"({data.get('note', '')})".rstrip())
    for p in pts:
        print(f"  +{p['lest_kg']:>4.0f} kg -> {p['v_moy_best']:.3f} m/s")
    print(f"\nDroite : v = {intercept:.3f} {pente:+.5f} x lest   (R² = {r2:.4f})")
    print(f"Pente : {abs(pente)*10:.3f} m/s perdu par 10 kg")
    print(f"v0 théorique (lest nul) : {intercept:.3f} m/s")

    print("\nLest maximal estimé selon le seuil de vitesse minimale (MVT) :")
    for mvt in (0.20, 0.15, 0.10):
        print(f"  MVT {mvt:.2f} m/s -> +{(intercept - mvt) / abs(pente):.0f} kg")
    if args.vrai_1rm is not None:
        v_au_max = intercept + pente * args.vrai_1rm
        print(f"\n1RM réel fourni : +{args.vrai_1rm:.0f} kg -> vitesse prédite "
              f"{v_au_max:.3f} m/s (négatif ou < 0,10 = profil pris fatigué "
              f"ou relation non linéaire près du max)")

    sortie = args.plot or args.json_points.replace(".json", ".png")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(8, 5.5))
    xs = np.linspace(0, max(x.max() * 1.5, 60), 100)
    ax.plot(xs, intercept + pente * xs, "--", color="tab:blue", lw=1.2,
            label=f"v = {intercept:.3f} − {abs(pente):.4f}·lest  (R²={r2:.3f})")
    ax.plot(x, y, "o", color="tab:red", ms=8, zorder=3, label="meilleure rep par palier")
    ax.axhspan(0.10, 0.20, color="tab:orange", alpha=0.15,
               label="zone MVT (vitesse d'un vrai max)")
    if args.vrai_1rm is not None:
        ax.axvline(args.vrai_1rm, color="tab:gray", ls=":",
                   label=f"1RM réel frais (+{args.vrai_1rm:.0f} kg)")
    ax.set_xlabel("lest (kg)")
    ax.set_ylabel("vitesse moyenne concentrique (m/s)")
    ax.set_title(f"Dips lestés — profil charge-vitesse ({data.get('date', '')})")
    ax.set_xlim(0, xs.max())
    ax.set_ylim(0, max(1.0, y.max() * 1.15))
    ax.grid(True, lw=0.3)
    ax.legend(loc="upper right", fontsize=9)
    fig.tight_layout()
    fig.savefig(sortie, dpi=130)
    print(f"\nGraphique : {sortie}")


if __name__ == "__main__":
    main()
