# Capteur de dips — VBT sans contrainte

Un capteur de musculation qui mesure la vitesse d'exécution des répétitions
(*velocity-based training*) et se fait oublier. Conçu, mesuré et validé à partir
de zéro : électronique, traitement du signal, firmware embarqué, boîtier.

**→ [Tableau de bord des mesures](https://virgile-pct.github.io/capteur-dips/)**

## L'idée

Le VBT est un outil d'élite : la vitesse d'une répétition dit la charge réelle,
la fatigue du jour et le moment où il faut s'arrêter — sans jamais faire de
maximum. Les capteurs du commerce coûtent de 300 à 500 €, et la plupart imposent
une façon de s'entraîner : marquer une pause entre chaque rep, déclarer son
exercice avant chaque série, ne rien avoir d'autre sur soi.

Un pratiquant expérimenté n'accepte pas ces contraintes. Elles finissent
toujours par ne plus être respectées, et le capteur finit dans un tiroir.

Le parti pris de ce projet : **le capteur ne doit rien imposer.** On l'enfile,
on s'entraîne normalement, et on consulte ses chiffres après la séance.

## Ce qui marche aujourd'hui

| | |
|---|---|
| Reps enchaînées sans pause | aucune contrainte de rythme |
| Exercice reconnu seul | 11 séries de référence sur 11 |
| Erreur sur la vitesse moyenne | 1,8 % contre vérité terrain |
| 1RM estimé sans maximum | +41 kg estimé contre +40 kg réel |
| Mouvements parasites | écartés, sans jamais perdre une rep de maximum |
| Traitement | embarqué sur la carte, plus besoin d'un PC à l'écoute |

Le 1RM estimé à un kilo près est le résultat dont je suis le plus content :
la chaîne complète — capteur maison, traitement, régression charge-vitesse —
est tombée juste sur une valeur qu'elle ne connaissait pas.

## Le matériel

ESP32 + accéléromètre-gyroscope MPU6050 sur la ceinture de lest, alimentation
sur batterie, liaison Bluetooth. Environ 15 € de composants. Une étude
d'implantation d'un circuit imprimé sur mesure est
[visible ici](https://virgile-pct.github.io/capteur-dips/pcb-v3.html).

## Quelques problèmes rencontrés, et résolus

- Un convertisseur de tension rayonnait assez pour corrompre le bus de données
  de l'accéléromètre. Diagnostiqué en mesurant le taux de lectures perdues en
  fonction de la distance : 100 % au contact, 0,07 % à quelques centimètres.
- Le capteur avait un défaut d'usine de +1,23 m/s² sur un axe, qui fabriquait
  des mètres d'amplitude fantôme. Corrigé par une calibration six faces propre à
  chaque exemplaire.
- Un défaut électrique d'une milliseconde pouvait figer la liaison et coûter une
  séance entière. Le firmware sait maintenant se rétablir seul.
- Toute la chaîne reposait sur deux secondes de calibration au démarrage, sans
  qu'aucun contrôle ne vérifie qu'elles étaient bien immobiles. Une fenêtre
  agitée faisait passer l'erreur de 1-4 % à 15-47 %. Le firmware refuse
  désormais de démarrer sur une calibration douteuse.

## Le code

Le code source — algorithme, firmware, chaîne d'analyse — **n'est pas public**.
Ce dépôt ne contient que le tableau de bord des mesures et l'étude du circuit
imprimé.

La méthode est le résultat de plusieurs semaines de mesures sur données réelles,
et le projet a une suite commerciale envisagée. Pour toute question, une
démonstration ou une collaboration : ouvrez une *issue*.

## Le projet en cours

Le capteur sert d'abord à préparer un championnat de France amateur de dips
lestés. La suite : essais sur cinq athlètes de morphologies différentes,
retour immédiat par vibration au seuil de fatigue, et une piste plus ambitieuse
— aider au jugement de la profondeur en compétition, aujourd'hui laissé à
l'appréciation humaine sur un geste qui dure une demi-seconde.

---

© 2026 Virgile Pourchet — tous droits réservés.
