"""Générateur du boîtier V1 imprimable en 3D — capteur de dips.

Boîtier de capture pour la ceinture lestée : carte ESP32 DevKit + module GY-521
montés rigides dans un même corps, deux ailes à fente pour passer la sangle
(la sangle plaque le boîtier contre la ceinture, comme un bracelet de montre),
échancrure pour le câble USB, couvercle à jupe friction imprimé à côté.

Toutes les cotes sont des constantes ci-dessous, en millimètres. À la livraison
du matériel : mesurer la carte au pied à coulisse, ajuster L_CARTE / l_CARTE,
relancer le script, réimprimer si besoin.

Usage :
    python generer_boitier.py          -> boitier_capteur_v1.stl + apercu.png

Géométrie volontairement 100 % additive (uniquement des pavés, aucun perçage
booléen) : chaque ouverture est un vide laissé entre des pavés. Le STL est donc
trivial à générer sans bibliothèque de CAO, et chaque pavé est un solide propre
que le slicer fusionne.

Impression : PLA ou PETG, 0,2 mm, 3 périmètres, 20 % de remplissage, SANS
support (toutes les faces en surplomb sont ouvertes vers le haut). Le rigide
est préférable au TPU : un boîtier souple filtrerait le mouvement à mesurer.
"""

import struct
import numpy as np

# ---------------------------------------------------------------- cotes (mm)
EP_MUR = 2.0        # épaisseur des murs
EP_FOND = 2.5       # épaisseur du fond
JEU = 0.3           # jeu d'ajustement couvercle/murs

# Carte ESP32 DevKit (à MESURER à la livraison, valeurs enveloppe par défaut)
L_CARTE = 52.0      # longueur de la baie carte (carte ~48-49 + jeu)
l_CARTE = 29.5      # largeur de la baie (couvre les variantes 25,4 à 28,5)
H_APPUI = 7.0       # hauteur des rails d'appui : les broches soudées sous la
                    # carte (~6 mm) pendent dans le vide central
LARG_APPUI = 2.5    # largeur de chaque rail d'appui sous les bords de la carte
H_SOUS_COUVERCLE = 14.0   # hauteur intérieure totale (appui + carte + USB)

# Module GY-521 (21,2 x 16,4 réels ; souder la barrette vers le HAUT)
L_GY = 23.0         # longueur de la loge
l_GY = 16.8         # largeur utile entre les cales (grip léger sur la carte)
H_CALE_GY = 4.0     # hauteur des cales qui encadrent le module

# Passage de sangle (sangle de ceinture lestée ~38-40 mm de large)
L_FENTE = 42.0      # longueur de la fente
l_FENTE = 4.0       # largeur de la fente (épaisseur sangle + aisance)
LARG_AILE = 13.0    # largeur de chaque aile
L_AILE = 50.0       # longueur des ailes
H_AILE = 6.0        # épaisseur des ailes

# Échancrure USB (connecteur USB-C + surmoulage du câble)
LARG_USB = 13.0

# ------------------------------------------------------- dérivées, ne pas toucher
X_INT0, Y_INT0 = EP_MUR, EP_MUR                    # coin intérieur de la cavité
L_INT = L_GY + EP_MUR + L_CARTE                    # loge GY | séparateur | baie carte
l_INT = l_CARTE
X_EXT, Y_EXT = L_INT + 2 * EP_MUR, l_INT + 2 * EP_MUR
H_CORPS = EP_FOND + H_SOUS_COUVERCLE

pave_liste = []


def pave(x0, y0, z0, x1, y1, z1):
    """Ajoute un pavé par ses deux coins opposés."""
    pave_liste.append((min(x0, x1), min(y0, y1), min(z0, z1),
                       max(x0, x1), max(y0, y1), max(z0, z1)))


def construire_corps():
    x1_gy = X_INT0 + L_GY                    # fin de la loge GY
    x0_carte = x1_gy + EP_MUR                # début de la baie carte
    x1_int = X_INT0 + L_INT                  # fin de la cavité
    yc = Y_EXT / 2.0

    # Fond plein sous toute la cavité
    pave(0, 0, 0, X_EXT, Y_EXT, EP_FOND)

    # Murs : côté GY plein, deux longs côtés pleins
    pave(0, 0, 0, EP_MUR, Y_EXT, H_CORPS)
    pave(0, 0, 0, X_EXT, EP_MUR, H_CORPS)
    pave(0, Y_EXT - EP_MUR, 0, X_EXT, Y_EXT, H_CORPS)

    # Mur côté USB : deux montants pleins + un seuil bas -> échancrure ouverte
    # vers le haut (imprimable sans support), du niveau de la carte au sommet
    y0_usb, y1_usb = yc - LARG_USB / 2, yc + LARG_USB / 2
    pave(x1_int, 0, 0, X_EXT, y0_usb, H_CORPS)
    pave(x1_int, y1_usb, 0, X_EXT, Y_EXT, H_CORPS)
    pave(x1_int, y0_usb, 0, X_EXT, y1_usb, EP_FOND + H_APPUI)

    # Séparateur GY / carte, avec un passage central de 12 mm pour les 4 fils
    y0_pass, y1_pass = yc - 6, yc + 6
    pave(x1_gy, Y_INT0, EP_FOND, x0_carte, y0_pass, EP_FOND + H_APPUI)
    pave(x1_gy, y1_pass, EP_FOND, x0_carte, Y_INT0 + l_INT, EP_FOND + H_APPUI)

    # Rails d'appui de la carte ESP32 (les broches pendent entre les deux)
    pave(x0_carte, Y_INT0, EP_FOND, x1_int, Y_INT0 + LARG_APPUI, EP_FOND + H_APPUI)
    pave(x0_carte, Y_INT0 + l_INT - LARG_APPUI, EP_FOND, x1_int, Y_INT0 + l_INT,
         EP_FOND + H_APPUI)

    # Cales latérales de la loge GY : le module se glisse entre elles, à plat
    # sur le fond (couplage rigide capteur/boîtier = mesure propre)
    larg_cale = (l_INT - l_GY) / 2.0
    pave(X_INT0, Y_INT0, EP_FOND, x1_gy, Y_INT0 + larg_cale, EP_FOND + H_CALE_GY)
    pave(X_INT0, Y_INT0 + l_INT - larg_cale, EP_FOND, x1_gy, Y_INT0 + l_INT,
         EP_FOND + H_CALE_GY)


def construire_ailes():
    """Deux ailes latérales, chacune percée d'une fente par assemblage de
    4 pavés (cadre autour du vide) : la sangle monte dans une fente, passe
    sous le dos du boîtier, redescend dans l'autre — tension = plaquage."""
    x0, x1 = (X_EXT - L_AILE) / 2, (X_EXT + L_AILE) / 2
    xf0, xf1 = (X_EXT - L_FENTE) / 2, (X_EXT + L_FENTE) / 2
    for y_base, sens in ((0.0, -1), (Y_EXT, +1)):
        y_ext = y_base + sens * LARG_AILE
        y_f0 = y_base + sens * ((LARG_AILE - l_FENTE) / 2)
        y_f1 = y_f0 + sens * l_FENTE
        pave(x0, y_f1, 0, x1, y_ext, H_AILE)          # bande extérieure
        pave(x0, y_base, 0, x1, y_f0, H_AILE)         # bande intérieure
        pave(x0, y_f0, 0, xf0, y_f1, H_AILE)          # bouchon gauche
        pave(xf1, y_f0, 0, x1, y_f1, H_AILE)          # bouchon droit


def construire_couvercle(y_decal):
    """Couvercle imprimé à plat à côté du corps : plaque + jupe friction.
    Jupe sur les deux longs côtés + le côté GY seulement (le côté USB reste
    libre pour le connecteur). Une fois retourné, le côté sans jupe va sur
    l'échancrure USB."""
    pave(0, y_decal, 0, X_EXT, y_decal + Y_EXT, 2.0)
    j0 = EP_MUR + JEU                 # face extérieure de la jupe
    ep_j, h_j = 1.6, 6.0
    pave(j0, y_decal + j0, 2.0, X_EXT - j0, y_decal + j0 + ep_j, h_j)
    pave(j0, y_decal + Y_EXT - j0 - ep_j, 2.0, X_EXT - j0, y_decal + Y_EXT - j0, h_j)
    pave(j0, y_decal + j0, 2.0, j0 + ep_j, y_decal + Y_EXT - j0, h_j)


def ecrire_stl(chemin):
    """STL binaire : 12 triangles par pavé, normales sortantes."""
    tris = []
    for (x0, y0, z0, x1, y1, z1) in pave_liste:
        p = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
             (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
        faces = [  # (normale, 4 sommets en ordre anti-horaire vu de dehors)
            ((0, 0, -1), [0, 3, 2, 1]), ((0, 0, 1), [4, 5, 6, 7]),
            ((0, -1, 0), [0, 1, 5, 4]), ((0, 1, 0), [2, 3, 7, 6]),
            ((-1, 0, 0), [0, 4, 7, 3]), ((1, 0, 0), [1, 2, 6, 5]),
        ]
        for n, (a, b, c, d) in faces:
            tris.append((n, p[a], p[b], p[c]))
            tris.append((n, p[a], p[c], p[d]))
    with open(chemin, "wb") as f:
        f.write(b"Boitier capteur de dips V1 - genere par generer_boitier.py".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        for n, a, b, c in tris:
            f.write(struct.pack("<12fH", *n, *a, *b, *c, 0))
    return len(tris)


def dessiner_apercu(chemin):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    fig, (haut, cote) = plt.subplots(1, 2, figsize=(13, 6),
                                     gridspec_kw={"width_ratios": [1.1, 1]})
    for (x0, y0, z0, x1, y1, z1) in pave_liste:
        haut.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, linewidth=0.6,
                                 edgecolor="tab:blue", facecolor="tab:blue", alpha=0.25))
        cote.add_patch(Rectangle((x0, z0), x1 - x0, z1 - z0, linewidth=0.6,
                                 edgecolor="tab:orange", facecolor="tab:orange", alpha=0.25))
    haut.set_title("Vue de dessus (XY) — corps + ailes, couvercle décalé")
    cote.set_title("Vue de côté (XZ)")
    for ax in (haut, cote):
        ax.set_aspect("equal")
        ax.autoscale()
        ax.grid(True, lw=0.3)
        ax.set_xlabel("mm")
    fig.tight_layout()
    fig.savefig(chemin, dpi=120)


if __name__ == "__main__":
    construire_corps()
    construire_ailes()
    construire_couvercle(y_decal=Y_EXT + LARG_AILE + 12)

    n_tris = ecrire_stl("boitier_capteur_v1.stl")
    dessiner_apercu("apercu_boitier.png")

    print("Boîtier V1 généré : boitier_capteur_v1.stl "
          f"({len(pave_liste)} pavés, {n_tris} triangles)")
    print(f"  Corps : {X_EXT:.0f} x {Y_EXT:.0f} x {H_CORPS:.0f} mm "
          f"(+ ailes : encombrement {X_EXT:.0f} x {Y_EXT + 2*LARG_AILE:.0f} mm)")
    print(f"  Baie carte : {L_CARTE:.1f} x {l_CARTE:.1f} mm | loge GY-521 : "
          f"{L_GY:.1f} x {l_GY:.1f} mm | fentes sangle : {L_FENTE:.0f} x {l_FENTE:.0f} mm")
    print("  Aperçu : apercu_boitier.png — vérifier avant impression")
