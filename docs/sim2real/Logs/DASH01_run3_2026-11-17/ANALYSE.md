# DASH-01 — run 3 de standing04_xshape (2026-11-17, Pi : 04:35 heure Pi)

Fichiers : `raw/` = black box brute (segment A continu ~13 Hz, dumps B 200 Hz autour des
changements de mode, événements de la session) ; les `.csv` sont les exports
(`blackbox_read.py --dump … --csv`).

## Ce qui s'est passé

| t (s, depuis le handover) | |
|---|---|
| avant | Pose politique en SET_POS, robot basculé sur les talons : pitch **−7.0°** |
| 0.00 | handover : couples aux bons signes (cam ±18.6 / 18.0 N·m), marge workspace 7.3° |
| 0.0 → 1.0 | **la politique travaille** : elle redresse le torse de −7.0° à 0°, en bougeant cam, cuisse et abduction de 1 à 4° |
| 1.0 → 1.69 | elle dépasse : pitch 0 → **+4.8°** (vers les orteils) |
| 1.69 | arrêt **doux** par le governor : *boucle à 60 Hz au lieu de 100* (88 ticks en retard sur 100) → maintien SET_POS à la pose mesurée |
| 1.7 → 3.4 | articulations figées, le robot continue de basculer en avant d'un bloc : pitch 5° → 43° |
| 3.4 → | il roule sur le côté (roll −37°) et pend dans le harnais, articulations toujours tenues |

Marge workspace minimale pendant le run : 5.2° (jambe), consigne toujours dedans (+0.5°).
Aucun clamp du governor, aucun tick bloqué par le workspace.

## Diagnostic

1. **La politique a fonctionné** : premier run avec des entrées cohérentes (IMU retournée corrigée,
   gyro en rad/s, couple au bon signe). Elle a corrigé une inclinaison de 7° qu'elle n'avait jamais
   vue à l'entraînement.
2. **Elle tournait à 0.6× sa vitesse** : la loi de commande suppose 10 ms par tick, elle en a eu
   ~16.7. Ses actions arrivent en retard et ses vitesses observées sont gonflées de 1.7× — le
   dépassement à +4.8° est typique d'un correcteur ralenti.
3. **Pourquoi 60 Hz** : la loi de commande seule prend **3.5 ms** sur le Pi (p50, cœurs libres,
   pas de throttling, 1.2 GHz, 65 °C) — dans le budget de 7 ms. Dans le daemon elle en prend
   **9.2** : sur les 2 cœurs du service, la boucle CAN 200 Hz, le fil IMU 200 Hz, l'enregistreur
   et les requêtes du navigateur se partagent le GIL Python.
4. **La chute** vient de l'arrêt : un maintien SET_POS rigide ne rattrape pas un robot qui bascule
   déjà à 12–20 °/s. L'arrêt doux est sûr pour un robot quasi immobile, pas en plein mouvement.

## Pistes

- Immédiat, sans code : **un seul onglet** de l'interface ouvert, jumeau 3D et graphes masqués
  pendant le run (moins de requêtes HTTP = moins de contention sur le GIL).
- Code : faire tourner l'inférence de la politique dans **un processus séparé** sur les cœurs 2–3
  (hors du GIL du daemon), le daemon gardant la boucle CAN et le governor.
- Code : à la fin d'un run debout **en mouvement** (vitesse angulaire élevée), préférer laisser la
  politique ralentir le robot ou passer en maintien amorti plutôt qu'en SET_POS rigide.
- Le basculement sur les talons avant l'armement (−7°) et le balancement en SET_POS (semelles TPU
  + boucle de position des drivers) restent à traiter : semelles rigides, armement seulement robot
  immobile.
