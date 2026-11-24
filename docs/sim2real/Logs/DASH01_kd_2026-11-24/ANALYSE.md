# DASH-01 — Test Kd du 24/11 : analyse

Plan : `../../DASH01_plan_test_Kd_2026-11-24.md` (version 2, trame « home » = action 0 de
`standing04_xshape_best`). Chiffres complets : `summary.md` / `summary.json`
(`tools/kd_validation/analyze_runs.py`). Logs : `raw/`.

## Runs

Tous debout, harnais tendu, « calme » (sans poussées), 10 s, 93–96 Hz.

| run | kp jambes | kd jambes | sifflement (oreille) |
|---|---|---|---|
| REF SET_POS | (boucle position) | — | très léger |
| E | 200 | 0,5 | atroce |
| B | 200 | 5 | énorme |
| D | 200 | 1 | énorme |
| G | 100 | 1 | très fort |

Le premier run A (pas figé, version 1) a coupé l'alimentation du robot à 0,87 s (session
`c08e49d1`, pas d'arrêt propre dans la boîte noire). Le robot était peut-être en charge : à vérifier.
Le run E suivant (1,3 s) a été arrêté par la garde de rythme (76 Hz). Le D suspendu n'a pas de log.

## Résultats

Ondulation du courant (A, écart-type autour de la moyenne glissante sur 90 ms) :

| run | came G | cuisse G | came D | cuisse D |
|---|---|---|---|---|
| E (kp 200, kd 0,5) | 6,1 | 0,15 | 6,5 | 0,39 |
| D (kp 200, kd 1) | 6,3 | 0,16 | 6,7 | 0,85 |
| B (kp 200, kd 5) | 3,0 | 0,32 | 6,3 | 1,65 |
| G (kp 100, kd 1) | 6,0 | 0,31 | 5,1 | 0,29 |
| *SET_POS immobile (run 4, 17/11)* | *0,07* | *0,02* | *0,01* | *0,02* |

1. **Le kd n'est pas la cause (H1 réfutée).** De kd 0,5 à kd 5, les cames ondulent pareil (5 à
   7 A). Seule la came G à kd 5 ondule moins (3 A). Ça confirme l'oreille.
2. **Le kp non plus, ou très peu.** Diviser kp par deux (G) laisse 5 à 6 A sur les cames.
3. **Ce n'est pas la dynamique de la politique.** Une trame fixe donne le même bruit que la
   politique (4 à 6 A aux runs 3 à 9). Les sauts de kp d'un tick à l'autre ne sont pas nécessaires.
4. **Ce sont les cames.** Les cuisses ondulent 4 à 40 fois moins, et les abductions (kp 120, kd 4)
   restent calmes (0,1 A). La vitesse des cames change de signe sur 17 à 38 % des ticks : le
   rotor vibre.
5. **La loi commandée n'explique rien du courant des cames** (`law_R2` ≈ 0). La vibration se
   passe au-dessus de 50 Hz, dans le driver. Les logs à 100 Hz n'en voient que le repli (pics à
   15–50 Hz).
6. **Même charge, même pose : SET_POS est silencieux, MIT siffle.** Il reste deux causes : le
   mode MIT du driver sous charge, quels que soient les gains, ou la chaîne fermée par le pied
   (H2). La « lutte » mesurée (puissances de signe opposé) vaut 49 à 63 % des ticks sur la jambe
   droite et 6 à 30 % sur la gauche. Mais avec une vitesse qui change de signe un tick sur trois,
   ce chiffre est gonflé par la vibration elle-même.

## Autres constats

- **Raideur statique ≈ 1,8 × la commande**, sur les quatre articulations de jambe et aux deux
  niveaux de kp (`kp_static` 340 à 380 pour kp 200, 160 à 190 pour kp 100). Le rapport est le même
  à deux kp différents : c'est donc une échelle, pas un décalage de zéro. Soit le couple loggé
  (courant × kt) est surestimé d'environ 1,8, soit le driver applique un kp plus grand que celui
  demandé. Ça touche directement le sim-to-real. À vérifier par un test statique avec une charge
  connue.
- Les gains de boucle de courant du driver diffèrent entre les cames. La came D a cur kp 0,207 /
  ki 2545 et la came G cur kp 0,126 / ki 1705 (postmortem, config enregistrée). La came D est la
  plus bruyante (dI 9 à 12 A, contre 4 à 8 A). C'est une piste pour le côté firmware.
- Tangage moyen −8 à −11° : sans équilibre, le robot s'assoit en arrière dans le harnais, comme
  prévu. Le harnais porte une partie du poids.
- **Températures 55 à 59 °C** en fin de série (cuisse D 59 °C au run D) : la limite de 60 °C est
  presque atteinte.

## Après les runs : tests au panneau (MIT hold)

Le détail est dans `../../DASH01_journal_2026-11-24.md` §4.3 et §5.3. En bref :
- debout, le MIT hold siffle comme la politique, et le SET_POS est presque silencieux ;
- suspendu, ça siffle aussi, mais moins fort ;
- ça siffle toujours à kp 50 / kd 0,3, et même à kp = kd = 0,1 ;
- la note est à **2,5 kHz**, alors que le léger bruit en SET_POS est plus aigu ;
- le bruit grandit avec la force appliquée à la main. Au-delà d'un seuil, **une oscillation de toute
  la jambe s'entretient** et seul LIMP l'arrête.

→ La cause est le **mode force control des drives AKE90-8**. Les gains et la politique n'y sont pour
rien. Les « Suite » ci-dessous sont remplacées par le §7 du journal.

## Suite (avant les tests au panneau)

1. **Suspendu, pieds libres** (test 1 de `DASH01_plan_tests_bruit_moteurs.md`, ou §6.2 du plan) :
   même trame MIT sans le pied au sol. Si le bruit disparaît, c'est la chaîne fermée ou la charge.
   S'il reste, c'est le mode MIT seul.
2. **Une came seule en MIT, à la main** (robot suspendu, panneau MIT hold) : kp 50 / 100 / 200,
   avec et sans charge à la main. Ça sépare la came de la lutte avec la cuisse.
3. **Firmware** : fréquence de la boucle MIT, filtre de vitesse, gains de la boucle de courant
   (identiques G/D ?).
4. **Échelle du couple** (le facteur 1,8) : poids connu au bout d'une jambe, en SET_POS et en MIT.
