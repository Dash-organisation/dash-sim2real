# DASH-01 — Journal de travail du 17 novembre 2026

Contexte pour le rapport de fin de semestre : premiers essais sur le robot réel de la politique
d'équilibre debout `standing04_xshape` (entraînée en simulation MuJoCo sur le modèle v8 « xshape »).

---

## 1. Point de départ

- La veille (16/11), le plan « calibration xshape & transition SET_POS → politique » avait été codé
  (commit `830017f`) : offsets du `deploy_map` dérivés de la pose debout du MJCF, bouton
  « Pose politique », garde-fous d'armement, handover SET_POS → MIT aux gains d'entraînement
  (`drive_kp`), fin de run en SET_POS plutôt qu'en LIMP.
- Ce code n'avait jamais tourné sur le robot. Le Pi (Raspberry Pi 3B, sans internet) faisait encore
  tourner une version antérieure.

## 2. Directions prises et pourquoi

| Décision | Alternatives écartées | Raison |
|---|---|---|
| Déploiement scripté et vérifié (`deploy_to_pi.sh`) | copie manuelle avec rsync | le Pi n'a pas internet et range le code dans `robot/` au lieu de `controller/` ; une copie à la main avait failli ne rien déployer. Le script vérifie, sauvegarde, copie, contrôle et sait revenir en arrière |
| **Zéro des moteurs pris debout** (option A : on capture la pose debout comme `STAND_POSE_DEG`) | option B : redéfinir tout le repère pour que 0° = debout partout | même effet pour la politique (debout = 0 rad dans le modèle), sans migrer workspace, gaits, fk_lut… ; l'opérateur dispose d'un gabarit de calibration debout reproductible |
| **Garder la butée physique du genou**, la déplacer, refaire le workspace | supprimer la butée | la butée est la dernière protection si le logiciel se trompe ; la pose de départ de la politique était à ~2.7° de la butée |
| **Garde du workspace sur la pose mesurée** pendant un run debout (la consigne peut dépasser le bord de 8°) | vérifier la consigne PD comme une pose | debout, la consigne PD est une demande de couple qui dépasse volontairement la pose réelle (le tassement) ; la vérifier comme une pose refusait tout armement |
| **Analyser chaque essai depuis la black box** (données 200 Hz) avant le suivant | relancer à l'aveugle | chaque chute avait une cause précise et mesurable ; c'est ce qui a permis de trouver trois bugs d'entrée |
| **Sortir l'inférence de la politique du daemon** (plan B, processus séparé) | changer de Pi (3B+ : même processeur), réentraîner à 50 Hz | le problème mesuré est la contention du verrou Python (GIL), pas la puissance du processeur |

## 3. Ce qui a été accompli

### 3.1 Outillage et déploiement
- Script `controller/deploy_to_pi.sh` : déploie les changements d'un commit (pas l'arbre de travail),
  vérifie que le Pi est bien sur la version attendue (fins de ligne ignorées), sauvegarde exactement
  les fichiers remplacés, contrôle md5 / `py_compile` / JSON, redémarre et lit le journal, rollback.
  Testé dans un bac à sable sur le Pi avant usage réel. Documenté dans `controller/README.md`.
- 7 déploiements sur le robot dans la journée, tous vérifiés, chacun avec une sauvegarde.

### 3.2 Calibration
- Zéro des moteurs capturé **debout** (la pose de départ de la politique) au lieu d'à genoux ; les
  anciennes calibrations restent lisibles ; le jumeau numérique affiche la pose debout par défaut.
- IMU : l'ancienne calibration de montage était **retournée de 180°** (datant d'avant le déplacement
  du Sense HAT sur le toit). Recalibrée par l'opérateur ; validée par le jumeau numérique, qui suit
  maintenant le robot. L'horizon artificiel du panneau était dessiné à l'envers : corrigé.
- Butée du genou déplacée et workspace refait (opérateur) : marge de la pose de départ passée de
  **1.7° à 7.3°**, la consigne centrale de la politique est maintenant dans le workspace.

### 3.3 Bugs trouvés et corrigés (tous avec tests)

| Bug | Effet | Comment il a été trouvé |
|---|---|---|
| Répétition (mock) : l'IMU simulée utilisait la calibration du vrai robot | toutes les répétitions « tombaient » au 1er tick ; risque d'écraser la vraie calibration | message `gravity_z +1.00, tilt 176°` |
| Gyroscope passé en **°/s** à la politique et au governor, qui attendent des **rad/s** | vitesses angulaires ×57 : arrêt d'urgence sur une fausse alerte, chute (run 1) | black box : 16 °/s mesurés = « 16.1 rad/s » dans le message d'arrêt |
| Couple observé sans le signe de calibration sur 3 moteurs | la politique voyait un couple inversé sur hip_roll_L, cam_L, thigh_R | comparaison des couples au handover avec ceux de la simulation : les 3 moteurs inversés étaient exactement ceux de signe −1 |
| Calibration IMU retournée de 180° (voir 3.2) | la politique recevait la gravité inversée | tests de basculement, cohérence avec le jumeau |

### 3.4 Sécurité
- Nouveau module `policy_ws.py` : distance signée au bord du workspace (grille du genou en champ de
  distance, abduction en intervalle). Pendant un run debout : arrêt si la jambe arrive à < 1° du
  bord ; la consigne peut mener le bord de 8° au plus ; marges enregistrées dans la black box et
  affichées dans le panneau.

### 3.5 Performance
- Ralentissement de l'interface pendant un run (« option A ») : le flux vers l'interface passe de 20
  à 2 trames/s et le navigateur arrête télémétrie et capteur. Gain : **60 → 76 Hz**.
- Diagnostic complet de la lenteur, et plan B rédigé pour un agent autonome
  (`DASH01_plan_B_policy_process.md`).

### 3.6 Commits de la journée (branche `homing-v7`)

| Commit | Contenu |
|---|---|
| `830017f` | (16/11, déployé le 17) calibration xshape, handover SET_POS → politique |
| `e731bc6` | script de déploiement + documentation |
| `4fe5849` | zéro des moteurs debout |
| `ac78b50` | répétition : calibration IMU propre au simulateur, en mémoire |
| `40129b2` | garde du workspace sur la pose mesurée (`policy_ws.py`) |
| `338d1ca` | gyroscope en rad/s pour la politique et le governor |
| `53b8116` | horizon du panneau IMU dans la bonne convention |
| `fdaaffe` | couple observé avec le signe de calibration |
| `1d8acb7` | interface au ralenti pendant un run |

## 4. Les essais sur le robot

| Run | Durée de la politique | Fin | Ce qu'on a appris |
|---|---|---|---|
| 1 | 0.17 s (2 ticks) | arrêt d'urgence « body rate 16.1 rad/s » → chute | bug d'unités du gyroscope ; IMU retournée ; couple de signe faux (3 bugs d'entrée) |
| 2 | 0.06 s (4 ticks) | garde workspace (jambe à 0.9° du bord) → maintien SET_POS | pose de départ à ~2.7° de la butée du genou ; le robot se balance sur ses talons |
| 3 | 1.69 s | governor : boucle à 60 Hz au lieu de 100 → SET_POS → chute en avant | **la politique fonctionne** : redresse le torse de −7° à 0° (puis dépasse à +5°) |
| 4 | 1.32 s | governor : 76 Hz → SET_POS, robot reste debout | redresse de −8.7° à −3° sans dépasser ; l'option A aide mais ne suffit pas |

Observations physiques notables :
- en maintien SET_POS, le robot **bascule sur ses talons** (−5 à −9°) et **se balance à 0.75 Hz**,
  d'amplitude croissante (4.9 → 8.2°) : semelles en TPU (ressort) + boucle de position des drivers
  qui réinjecte de l'énergie (courants en retard de 20–60 ms sur le pitch) ;
- couples réels vs simulation au départ : cames ≈ simulation (±19 vs ±14 N·m), cuisses 2 à 4× la
  simulation (cuisse droite 22.5 vs 5.5 N·m) → répartition du poids / centre de masse différents.

## 5. Mesures clés de performance (Pi 3B)

| Mesure | Valeur |
|---|---|
| loi de commande seule, cœurs libres | 3.5 ms p50 |
| loi de commande dans le daemon | 9.2 ms |
| boucle du daemon sans politique | 100 Hz tenus (dt p90 ~11 ms) |
| boucle avec politique (run 4) | dt p90 22.5 ms, max 248 ms → 76 Hz |
| processeur | 1.2 GHz, pas de throttling, 65 °C, OpenBLAS déjà en place |

Cause : un seul interpréteur Python (GIL) pour la boucle de contrôle, le fil IMU à 200 Hz,
l'enregistreur et l'interface.

## 6. Ce qui reste à faire

1. **Plan B** (en cours, agent autonome) : inférence dans un processus séparé ; validation sur le Pi
   (critère : ≥ 99 Hz tenus), puis nouveaux runs debout.
2. **Vérifier les signes du `deploy_map`** par un test de direction articulation par articulation
   (indice fort qu'ils sont bons : les couples au handover ont maintenant le signe de la simulation,
   mais ce n'est pas une preuve complète).
3. **Fin de run en mouvement** : le maintien SET_POS rigide fait tomber un robot qui bascule déjà.
4. **Écart simulation / réel au départ** : basculement sur les talons, balancement en SET_POS,
   semelles TPU. Pistes : semelles rigides pour les essais, armer seulement robot immobile,
   mesurer la raideur de la semelle depuis la black box et la modéliser en simulation.
5. **Robustesse de la politique** : elle a été entraînée sans randomisation (décalage du centre de
   masse 0 mm, poussées 0.02 m/s, masses exactes). Réentraîner avec randomisation du centre de
   masse, de l'inclinaison de départ, de la souplesse des pieds, et la vraie limite du genou.
6. Optionnel : passer à un Pi 4/5 (×3–8 par cœur) pour de la marge (réseaux plus gros, marche).

## 7. Enseignements méthodologiques (utiles pour le rapport)

- **Les tests avec des capteurs « parfaits » cachent les bugs d'interface** : les trois bugs d'entrée
  (unités, signe, repère) passaient les tests parce que le faux capteur publiait un gyroscope nul et
  que le bus simulé avait ses propres signes. Chaque correctif est maintenant accompagné d'un test
  qui l'aurait détecté.
- **La black box à 200 Hz a été décisive** : chaque échec a été expliqué par les données (pas par
  intuition), y compris en invalidant une première hypothèse (IMU « inversée » vs convention d'axes).
- **Comparer le réel à la simulation sur des grandeurs simples** (signe et ordre de grandeur des
  couples au départ, inclinaison au repos) a révélé des problèmes que la vue d'ensemble masquait.
- **Garder les protections physiques** et ajouter des gardes logicielles mesurées plutôt que
  d'en retirer.

## 8. Fichiers de référence

- Analyses : `~/Desktop/DASH01_run3_2026-11-17/ANALYSE.md`, `~/Desktop/DASH01_run4_2026-11-17/ANALYSE.md`
- Plan B : `~/Desktop/DASH01_plan_B_policy_process.md`
- Documentation déploiement : `controller/README.md` (section « Deploying to the robot's Pi »)
