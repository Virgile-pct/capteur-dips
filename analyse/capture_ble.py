"""Capture BLE du capteur (firmware V2) — reconstitue le même CSV que le filaire.

Scanne le Bluetooth, se connecte au capteur « CapteurDips », s'abonne au flux
de mesures et écrit un CSV identique à celui de la liaison série : la chaîne
d'analyse (analyse_reps.py) fonctionne sans aucune modification.

Usage :
    python capture_ble.py [--duree 150] [--sortie seance.csv]

Nécessite :  pip install bleak   (et un PC avec Bluetooth — sinon dongle à 5 €)
"""

import argparse
import asyncio
import struct
import sys
import time

try:
    from bleak import BleakClient, BleakScanner
except ImportError:
    sys.exit("bleak manquant : lancer  pip install bleak")

NOM_CAPTEUR = "CapteurDips"
UUID_CARAC_DATA = "b5f0a1c0-9d6b-4b7a-8e1f-3c2a7d9e0002"
TAILLE_ECH = 16          # octets par échantillon (cf. firmware V2)


async def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--duree", type=float, default=150.0, help="secondes de capture")
    ap.add_argument("--sortie", default="seance_ble.csv")
    args = ap.parse_args()

    print(f"Recherche de « {NOM_CAPTEUR} » en BLE...")
    appareil = await BleakScanner.find_device_by_name(NOM_CAPTEUR, timeout=15.0)
    if appareil is None:
        sys.exit("Capteur introuvable. Vérifier : capteur allumé (LED qui bat "
                 "lentement), Bluetooth du PC actif, distance < 10 m.")

    n = [0]
    f = open(args.sortie, "w", encoding="utf-8", newline="")
    f.write("t_ms,ax,ay,az,gx,gy,gz\n")

    def sur_notification(_, donnees: bytearray):
        for i in range(0, len(donnees) - TAILLE_ECH + 1, TAILLE_ECH):
            t_ms, ax, ay, az, gx, gy, gz = struct.unpack_from("<Ihhhhhh", donnees, i)
            f.write(f"{t_ms},{ax/500:.4f},{ay/500:.4f},{az/500:.4f},"
                    f"{gx/10:.3f},{gy/10:.3f},{gz/10:.3f}\n")
            n[0] += 1
            if n[0] % 500 == 0:
                print(f"  {n[0]} échantillons...")

    async with BleakClient(appareil) as client:
        print(f"Connecté. Capture {args.duree:.0f} s -> {args.sortie}")
        await client.start_notify(UUID_CARAC_DATA, sur_notification)
        t0 = time.time()
        try:
            while time.time() - t0 < args.duree:
                await asyncio.sleep(0.5)
        except KeyboardInterrupt:
            pass
        await client.stop_notify(UUID_CARAC_DATA)

    f.close()
    duree_utile = n[0] / 100.0
    print(f"Terminé : {n[0]} échantillons ({duree_utile:.1f} s utiles) -> {args.sortie}")
    if n[0]:
        print(f"Analyse :  python analyse_reps.py {args.sortie}")


if __name__ == "__main__":
    asyncio.run(main())
