# DASH-01 — bruit aigu des moteurs pendant la politique (runs 3 et 4, 2026-11-17)

Données : dumps B MANUAL→POLICY des runs 3 et 4 (100 Hz, colonnes pos/cmd/spd/cur), événements de session.
Limite : la black box échantillonne à 100 Hz, le bruit audible est à plusieurs centaines de Hz ou plus.
On ne voit donc que sa trace repliée (aliasing) ; on ne peut pas mesurer sa fréquence avec ces logs.

## 1. Limite de courant : non

| | valeur |
|---|---|
| \|I\| max came pendant la politique | 22–27 A (un pic à 52 A, came G run 4, t = 1.2 s) |
| \|I\| max cuisse / abduction | 15 A / 9 A |
| courants déjà sortis par les mêmes drivers dans la même session | ~100 A (cuisse D, came G) |
| pic constructeur AKE90-8 / AK60-39 | 72 A / 17 A |
| codes d'erreur drivers | 0 partout |
| governor, saturation couple | 0 tick |
| température max | 54 °C |

Aucun plateau d'écrêtage dans les courants. Le moteur n'est pas en limite de courant.

## 2. Ondulation de courant rapide, surtout sur les cames, et seulement en mode MIT

Ondulation = courant moins sa moyenne glissante sur 90 ms (écart-type, A) :

| phase | came D | came G | cuisse D | cuisse G |
|---|---|---|---|---|
| SET_POS avant (immobile) run 4 | 0.01 | 0.07 | 0.02 | 0.02 |
| **POLICY run 3** | **4.0** | **3.7** | 1.4 | 0.7 |
| **POLICY run 4** | **5.3** | **6.4** | 1.6 | 1.1 |
| SET_POS après (robot qui se balance) run 4 | 1.6 | 1.5 | 0.3 | 0.7 |

À vitesse articulaire égale, l'ondulation de la came est 4 à 50 fois plus forte en POLICY (trame MIT,
kp 200 / kd 5) qu'en SET_POS (boucle de position du driver) :

| vitesse came | SET_POS | POLICY |
|---|---|---|
| < 0.5 °/s | 0.03 A | 1.6 A |
| 0.5–2 °/s | 0.10 A | 3.3 A |
| 5–10 °/s | 1.0 A | 4.9 A |

L'ondulation ne vient donc pas seulement du mouvement : elle vient de la façon dont le moteur est commandé.

Autres signes :
- la vitesse renvoyée par la came change de signe d'un échantillon à l'autre (±500 à ±1300 ERPM, soit
  ±20 à 50 °/s) alors que la position avance régulièrement de ~0.1° par tick : le rotor vibre ;
- ce bruit de vitesse vaut 11–38 °/s rms sur les cames et 3–14 °/s sur les cuisses ;
- les à-coups de la consigne de la politique (0.45° rms par tick, 4° au premier tick) n'expliquent que
  ~1 A sur les 4–6 A ; les ticks en retard n'aggravent presque rien (7.4 contre 6.6 A).
- la loi tau = kp·e − kd·v, calée sur les échantillons, explique à peine 13–21 % de la variance du courant
  des cames : l'essentiel se passe au-dessus de 50 Hz, là où les logs ne voient rien.

Hypothèse la plus probable : la boucle d'impédance MIT du driver avec **kd = 5 N·m·s/rad, le maximum de la
plage du driver**, et kp 200, amplifie le bruit de son estimation de vitesse (encodeur dérivé) et le jeu du
réducteur ou de la transmission de la came. Le résultat est une vibration à haute fréquence, c'est-à-dire
le sifflement. En simulation, kd = 5 est un amortisseur idéal ; sur le driver, c'est un gain sur une dérivée bruitée.

## 3. Couplage came / cuisse pied au sol : oui en basse fréquence, non pour le bruit

Basse fréquence (moyenne glissante 90 ms), pendant la politique :

| | run 3 D | run 3 G | run 4 D | run 4 G |
|---|---|---|---|---|
| corr(position came, position cuisse) | 0.74 | 0.81 | 0.95 | 0.98 |
| corr(courant came, courant cuisse) | 0.97 | 0.81 | 0.45 | 0.10 |

La chaîne fermée pied-sol est bien visible : les deux articulations bougent ensemble. La politique leur
demande en plus des choses opposées :

| | consigne − position, moyenne (min/max) |
|---|---|
| came D / G run 4 | +2.9° / +3.7° (jusqu'à +6.7°) : la came pousse |
| cuisse D / G run 4 | −1.7° / −2.8° (jusqu'à −4.7°) : la cuisse retient |

Exemple : cuisse G run 4, consigne 12.6 → 12.3° pendant que la position passe de 12.5 à 16.4°. La
cuisse est entraînée par la came à travers le pied, contre sa propre consigne. Le courant de la cuisse
bouge peu malgré l'erreur de 4.7°, ce qui laisse penser que la politique baisse le kp de la cuisse. On ne
peut pas le vérifier : le kp/kd par tick n'est pas dans les logs.

Haute fréquence : corr(ΔI came, ΔI cuisse) ≈ 0 (−0.16 à +0.0) pendant la politique. Jusqu'à 50 Hz,
l'ondulation de la came et celle de la cuisse ne varient pas ensemble. Au-dessus, les logs ne voient
rien.

### Lutte came/cuisse : les puissances (ajout du 2026-11-20)

Puissance mécanique de chaque articulation, P = couple × vitesse (moyennes glissantes 90 ms). Quand la
came fournit de la puissance pendant que la cuisse en absorbe, ou l'inverse, l'une pousse et l'autre
freine à travers le pied.

| | Puissances de signe opposé | Dont came motrice | corr(lutte, bruit de la came) |
|---|---|---|---|
| POLICY run 3 (D / G) | 52 % / 52 % | 31 % / 32 % | 0.61 / 0.47 |
| POLICY run 4 (D / G) | 58 % / **80 %** | 43 % / **66 %** | 0.26 / 0.21 |
| SET_POS après (runs 3 et 4) | 0 à 5 % | 0 à 5 % | — |

La lutte existe : elle vient des consignes de la politique (came en avance, cuisse en retard) et
n'existe pratiquement pas en SET_POS. Le bruit de la came augmente avec elle. Limites : lutte, bruit
et gains élevés n'apparaissent qu'ensemble, en POLICY, et augmentent tous avec la vitesse, donc la
corrélation ne prouve pas que la lutte cause le bruit. Une puissance de signe opposé peut aussi être
un partage légitime de l'effort sous le poids du robot.

**Conclusion revue** : deux causes possibles, sans doute combinées. Les gains (kd 5) rendent chaque
came prête à vibrer (H1), et la lutte came/cuisse à travers le pied lui fournit l'énergie (H2).
Les tests sont décrits dans `SimToReal/DASH01_plan_tests_bruit_moteurs.md`.

## 3 bis. Run 5 (2026-11-20, 83 Hz après le plan C)

- Même ondulation des cames : ΔI par tick 5.4 A (D) et 4.6 A (G), contre 6–10 A aux runs 3 et 4. Le
  rythme de la boucle n'en est pas la cause.
- Le fichier du run (`DASH01_run5_2026-11-20/policyrun_20260822_081716.csv`) donne les gains demandés
  à chaque tick :

  | | kd = 5 (le max) | Δkp par tick, rms (max) | marche de couple Δkp × erreur, rms (max) |
  |---|---|---|---|
  | came G | 76 % des ticks | 55 (138) N·m/rad | 2.9 (8.0) N·m |
  | came D | 99 % des ticks | 43 (114) N·m/rad | 1.8 (5.5) N·m |
  | cuisse G | 74 % | 37 (123) | 1.0 (3.6) |
  | cuisse D | 54 % | 48 (124) | 1.3 (4.9) |

- La politique **colle le kd des cames au maximum** et fait **sauter leur kp** d'un tick à l'autre.
  Les sauts de kp sont une deuxième source d'à-coups, plus petite (~1.5 A rms). Elle baisse aussi le
  kp de la cuisse gauche (médiane 86 N·m/rad).

## 4. Tests pour trancher (première version, remplacée par `SimToReal/DASH01_plan_tests_bruit_moteurs.md`)

1. Robot suspendu, pieds libres, trame MIT à **consigne fixe** kp 200 / kd 5 : s'il siffle, le couplage
   et la politique ne sont pas en cause, c'est le gain du driver.
2. Même chose avec kd 5 → 2 → 1 (kp fixe), puis kp 200 → 100 (kd fixe) : on trouve quel gain fait siffler.
3. Même consigne fixe, robot debout dans le harnais : si le bruit apparaît seulement debout, la chaîne
   fermée (raideur pied + jeu) y participe.
4. Sauvegarder le `log` du résultat de la politique (colonnes kp6/kd6 par tick) avec les dumps : il
   manque pour relier le bruit aux gains demandés par la politique.

Pistes si kd est en cause : plafonner kd en déploiement (et l'entraîner avec un kd réaliste), filtrer
la vitesse côté driver si le firmware le permet, interpoler la consigne et passer à 100 Hz tenus (plan B).
