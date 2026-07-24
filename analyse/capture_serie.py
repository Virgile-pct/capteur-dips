"""Capture le flux série de l'ESP32 vers un fichier CSV, prêt pour analyse_reps.py.

Usage :
    python capture_serie.py --sortie seance.csv          (port auto-détecté)
    python capture_serie.py --port COM5 --sortie seance.csv

Arrêt : Ctrl+C (le fichier est refermé proprement).
Nécessite pyserial :  pip install pyserial
"""

import argparse
import sys

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("pyserial manquant : lancer  pip install pyserial")


def detecter_port():
    """Cherche un port USB-série typique d'un ESP32 (CP210x, CH340...)."""
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = (p.description or "").upper()
        if any(m in desc for m in ("CP210", "CH340", "CH910", "USB", "UART")):
            return p.device
    if ports:
        return ports[0].device
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default=None)
    ap.add_argument("--bauds", type=int, default=115200)
    ap.add_argument("--sortie", default="seance.csv")
    args = ap.parse_args()

    port = args.port or detecter_port()
    if not port:
        sys.exit("Aucun port série trouvé. Brancher l'ESP32 puis réessayer, "
                 "ou préciser --port COMx")

    print(f"Capture sur {port} à {args.bauds} bauds -> {args.sortie}")
    print("Ctrl+C pour arrêter.\n")
    n = 0
    with serial.Serial(port, args.bauds, timeout=2) as lien, \
            open(args.sortie, "w", encoding="utf-8", newline="") as f:
        try:
            while True:
                ligne = lien.readline().decode("utf-8", errors="replace").strip()
                if not ligne:
                    continue
                f.write(ligne + "\n")
                n += 1
                if ligne.startswith("#"):
                    print(ligne)          # messages du firmware (calibration...)
                elif n % 500 == 0:
                    print(f"  {n} lignes...")
        except KeyboardInterrupt:
            pass
    print(f"\nTerminé : {n} lignes dans {args.sortie}")
    print(f"Analyse :  python analyse_reps.py {args.sortie}")


if __name__ == "__main__":
    main()
