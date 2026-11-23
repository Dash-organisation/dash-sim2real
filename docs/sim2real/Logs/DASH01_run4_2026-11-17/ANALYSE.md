# DASH-01 — run 4 de standing04_xshape (2026-11-17), premier run avec A (UI au ralenti)

Fichiers : `raw/` = black box brute (segment A continu, dumps B 200 Hz : LIMP→MANUAL et
MANUAL→POLICY, événements de la session) ; `.csv` = exports.

## Résultat

| | run 3 (sans A) | run 4 (avec A) |
|---|---|---|
| rythme réel de la politique | 60 Hz | **76 Hz** |
| ticks en retard | 88 / 100 | 72 / 100 |
| durée avant arrêt (governor : rythme < 90 %) | 1.69 s | 1.32 s |
| pitch au départ (basculé sur les talons) | −7.0° | −8.7° |
| pitch à l'arrêt | +4.8° (dépassement) | −3.0° (correction en cours, sans dépassement) |
| fin | SET_POS → chute en avant dans le harnais | SET_POS → **reste debout**, balancement −8° … −1° (0.75 Hz) |
| marge workspace min (jambe / consigne) | 5.2° / +0.5° | 5.2° / +0.9° |

Couples au handover : signes de la simulation sur les 6 articulations (cam ±19.4 / 20.5 N·m,
cuisses 10.7 / 22.5 N·m — la cuisse droite porte 4× la simulation).

## La boucle du daemon (colonne `dt`, 100 Hz nominal = 10 ms)

| mode | dt p50 | dt p90 | max |
|---|---|---|---|
| LIMP | 10.0 ms | 10.8 ms | 40 ms |
| MANUAL (SET_POS) | 10.0 ms | 11.2 ms | 66 ms |
| POLICY | 10.9 ms | **22.5 ms** | **248 ms** |

La boucle tient 100 Hz tant qu'il n'y a pas de politique. Avec la politique, ~1 tick sur 10 dure
plus de 20 ms : la loi de commande (3.5 ms seule), le governor (~1.9 ms), CAN, log, et le GIL
partagé avec le fil IMU et les requêtes ne tiennent pas dans 10 ms. A a gagné 60 → 76 Hz ; il en
faut ≥ 90 pour que le governor laisse tourner, et 100 pour que la politique tourne à sa vitesse.

## Ce que fait la politique

Partie de −8.7° (sur les talons), elle ramène le torse à −3° en ~1.3 s en avançant les cames
(53 → 57°) et les cuisses (12.5 → 16°), sans dépasser, avec des à-coups ponctuels (gyro jusqu'à
±55 °/s vers 1.0 s). C'est cohérent avec une politique qui rattrape une inclinaison qu'elle n'a pas
vue à l'entraînement, à 0.76× de sa vitesse.

## Conclusion

- A aide (+16 Hz) mais ne suffit pas : **B (inférence dans un processus séparé) est nécessaire.**
- Le balancement en SET_POS (semelles TPU + boucle de position des drivers) réapparaît dès la fin
  du run ; le robot démarre basculé de −7 à −9° sur les talons.
