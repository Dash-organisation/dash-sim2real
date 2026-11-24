# DASH-01 — Plan de test : Kd est-il le problème du sim-to-real ? (24/11)

Version 2, 24/11. Branche `hw/Kd_validation`.

> **⚠ Résultat du 24/11 (`DASH01_journal_2026-11-24.md`) : le sifflement vient du mode force
> control (MIT) des drives, pas du kd.** En MIT, au-delà d'un certain couple, la jambe entre dans
> une oscillation à 2,5 kHz qui s'entretient : **seul LIMP l'arrête**. Pas de runs MIT sous charge
> tant que ce n'est pas corrigé. (Version 1 : 23/11 au soir, pas figé de la politique.)

> **Changement du 24/11 : la trame n'est plus un pas de la politique.** Au premier run (A, pas
> figé), le robot s'est éteint d'un coup après 0,87 s (perte du Pi et du SSH : coupure
> d'alimentation probable, à diagnostiquer avec `blackbox_read.py --postmortem`). Après la
> recalibration, on garde le plus simple : chaque bundle envoie **l'action 0** de
> `standing04_xshape_best`, c'est-à-dire la trame de passage SET_POS → politique que le daemon
> envoie déjà : cible `nominal_ctrl` (la stance de départ `default_motor_pos` plus la déflexion
> de gravité qui porte le robot), kp 200 (jambes) / 120 (abductions), kd 5 / 4. Seuls le kd et le
> kp des jambes changent (§3). Les sections 1 et 2 décrivent la version 1 et restent valables
> pour la question posée. `make_frozen_bundles.py --frame step` refait les anciens bundles.
Suite de `DASH01_plan_tests_bruit_moteurs.md` et de `Logs/ANALYSE_bruit_moteurs.md`.

Outils : `tools/kd_validation/` (README en anglais). Bundles : `controller/deploy/bundles/kdval_*.npz`.

---

## 1. La question et l'idée du test

Pendant la politique, les cames sifflent, leur courant ondule de 4 à 6 A rms, et la vitesse renvoyée
change de signe d'un tick à l'autre. Les logs montrent que la politique **colle le kd des cames au
maximum (5 N·m·s/rad)**. Les autres hypothèses sont la lutte came/cuisse à travers le pied et les
sauts de kp d'un tick à l'autre (40 à 55 N·m/rad en rms au run 5). En POLICY, tout varie en même
temps, donc les logs ne permettent pas de trancher.

**Le test** consiste à prendre **un pas** de la politique `standing04_xshape_best` dans une
simulation où elle tient debout, et à le **figer** : la même trame MIT (cible, kp et kd par
articulation) est envoyée à chaque tick. On fait ensuite varier **seulement le kd** des quatre
articulations de jambe (cames et cuisses). On mesure, à chaque fois :
- le bruit : ondulation du courant, bruit de vitesse, sifflement (audio) ;
- le kp et le kd obtenus réellement, avec des poussées douces pour exciter les articulations.

**Comment.** Chaque variante est un bundle v3 normal. Sa dernière couche est mise à zéro
(`act_w = 0`) et son biais vaut l'action du pas choisi (`act_b = a*`). Le réseau sort donc a* quoi
qu'il observe. Le daemon le traite comme n'importe quelle politique : mêmes vérifications à
l'armement, même approche, mêmes gardes (workspace, governor, homme mort), même plafond de 10 s et
même log par tick (pos, vel, tau, amps, target, kp, kd). **Aucun code du robot n'est modifié.**
Seuls six fichiers de 15 kB sont à copier.

## 2. Ce qu'on sait déjà (préparation du 23/11)

### 2.1 Dans les anciens logs (runs 5 à 9)

`analyze_runs.py`, appliqué aux runs 5 à 9, donne :

| run | kd came G / ondulation | kd came D / ondulation |
|---|---|---|
| 5 | 5.0 / 3.8 A | 5.0 / 4.1 A |
| 6 | 5.0 / 3.6 A | 5.0 / 3.7 A |
| **7** (suspendu, ~0 couple) | **1.3 / 0.24 A** | **5.0 / 3.9 A** |
| **8** (suspendu) | **1.5 / 0.21 A** | **5.0 / 3.4 A** |
| 9 (chute avant rapide) | 2.3 / 5.5 A | 5.0 / 3.8 A |

- Runs 7 et 8 : dans le même run, la came à kd ≈ 1,4 est calme et celle à kd 5 ondule 15 fois plus.
  Sa vitesse change de signe sur 19 % des ticks, contre 2 %, et le pic du spectre du courant est
  vers 40 Hz. Mais le kp diffère aussi entre les deux cames (89 contre 295).
- Sur tous les ticks des cinq runs, l'ondulation moyenne des cames monte avec le kd : 0,8 A sous
  kd 1,5, 1,4 A, 2,2 A, puis 3,0 A au-dessus de kd 4,5. Une régression l'attribue au kd
  (+0,57 A par unité), pas au kp. **Rien de tel sur les cuisses.**
- Le run 9 est un contre-exemple : 5,5 A à kd 2,3, pendant la chute rapide. Dans la politique, kd,
  kp et vitesse bougent ensemble, donc ce n'est qu'un indice. Le test à trame figée sert à trancher.

### 2.2 En simulation (`pick_step.py`, plante `stand_xshape.xml` nominale, sans bruit, retard 12 ms)

- La politique tient debout, et **en régime établi elle colle elle aussi le kd des deux cames et
  de la cuisse G à 5** (100 % des ticks), comme sur le robot. Son kp, en revanche, varie de moins
  de 0,1 N·m/rad d'un tick à l'autre, contre 40 à 55 sur le robot.
  → **Les sauts de kp sont un comportement propre au robot réel.** La politique réagit à quelque
  chose (bruit de mesure, état hors domaine) qu'elle ne voit pas en simulation.
- **Le pas figé ne tient pas debout seul** : il bascule en arrière et tombe en 1,6 s. L'équilibre
  vient de la politique, pas de la posture. → Sur le robot, **le harnais doit retenir le torse**
  pendant tous les runs figés.
- **La politique n'a que deux stances calmes, toutes deux asymétriques.** On l'a cherché sur 48 runs
  simulés, avec des poussées de 10 à 45 N dans 12 directions :
  - sans poussée, ou après une poussée faible, **l'abduction droite est presque lâche** (kp 20,
    kd 0,3) ;
  - après une poussée vers l'avant de 20 N ou plus, c'est la **jambe gauche qui mollit** (came
    kp 35, cuisse kp 21), et les pieds se sont déplacés de 10° par rapport à la pose de départ.

  Aucun tick ne donne une trame propre telle quelle. → On garde **les jambes exactement comme dans
  le pas** de la première stance, et on met **les deux abductions à la trame nominale du bundle**
  (action 0 : cible `nominal_ctrl` ±1,68°, kp 120, kd 4). L'abduction ne fait pas partie de la
  question, et ainsi le robot ne penche pas sur le côté. Vérifié en simulation, trame figée,
  robot libre : roulis 0,00° à 0,5 s, contre +0,3 à +0,4° avec l'abduction brute du pas.
- **En simulation, le kd ne compte presque pas.** La politique complète, avec le kd des
  cames/cuisses plafonné à 3, 2 ou 1, tient toujours debout et encaisse des poussées de ±30 N
  (tangage max 4,7° à kd 5, 5,5° à kd 1). Avec le torse tenu et la trame figée, passer le kd de 5
  à 0 change la pose finale de moins de 0,5°.
  → Si le bruit du robot réel change beaucoup avec le kd, **l'écart est dans le driver** (le kd du
  driver appliqué à une vitesse estimée bruitée), pas dans la politique. **Plafonner kd au
  déploiement ne casserait pas la politique**, du moins en simulation.

Chiffres complets : `tools/kd_validation/step_standing04_xshape_best.json` (section `checks`).

## 3. Les variantes (version 2 : trame « home »)

Ordre du modèle : hip_roll_L, cam_L, thigh_L, hip_roll_R, cam_R, thigh_R.
Cible pour **tous** les bundles : `nominal_ctrl` = −1,68 / +1,63 / +6,08 / +1,68 / −1,74 / −5,89°.
Abductions : kp 120, kd 4 partout.

| bundle | kp jambes | kd jambes | rôle |
|---|---|---|---|
| `kdval_E_kd05.npz` | 200 | **0,5** | le plus calme attendu : premier run (voir §6.1) |
| `kdval_D_kd1.npz` | 200 | **1** | |
| `kdval_C_kd2.npz` | 200 | **2** | |
| `kdval_B_kd5.npz` | 200 | **5** | la trame du bundle telle quelle, le pire cas, la référence |
| `kdval_G_kd1_kphalf.npz` | **100** | 1 | avec F : grille 2 × 2 kp {200, 100} × kd {5, 1} |
| `kdval_F_kd5_kphalf.npz` | **100** | 5 | le kp joue-t-il aussi ? |

À kp 100, la déflexion de gravité ne donne plus que la moitié du couple de soutien : le robot
s'enfonce dans le harnais, qui doit le porter. Les six bundles sont vérifiés dans le contrôleur
déployé et dans le daemon sur MockBus (`test_frozen_bundles.py`, 8 tests OK).

### 3.1 Version 1 (abandonnée) : le pas choisi

Pas : tick 1499 (t = 14,99 s) d'un run de 20 s. C'est le tick le plus proche de l'action médiane
sur 10–20 s, donc un vrai pas d'un run qui tient debout, pas une moyenne. À ce tick : tangage
+1,2°, couples came G +16,5, came D −8,8, cuisse G +3,5, cuisse D −5,1 N·m.

Ordre du modèle : hip_roll_L, cam_L, thigh_L, hip_roll_R, cam_R, thigh_R.

| | hip_roll_L | cam_L | thigh_L | hip_roll_R | cam_R | thigh_R |
|---|---|---|---|---|---|---|
| cible (°) | −1,68 *(nominal)* | +2,78 | +5,55 | +1,68 *(nominal)* | −2,82 | −5,87 |
| (nominal_ctrl, °) | −1,68 | +1,63 | +6,08 | +1,68 | −1,74 | −5,89 |
| kp (N·m/rad) | 120 *(nominal)* | 211 | 133 | 120 *(nominal)* | 105 | 218 |
| kd (N·m·s/rad) | 4 *(nominal)* | **5** | **5** | 4 *(nominal)* | **5** | 1,91 |
| *(pas brut, abduction)* | *−0,92° / 365 / 4,55* | | | *+3,85° / **20** / 0,29* | | |

Les cibles des jambes sont à ~1° de la stance de la politique (`nominal_ctrl`), qui passe déjà
l'armement. Les abductions sont celles du bundle à action 0 (voir 2.2).
`make_frozen_bundles.py --abduction step` refait les bundles avec l'abduction brute du pas.

| bundle | kd jambes (cames + cuisses) | kp jambes | rôle |
|---|---|---|---|
| ~~`kdval_A_step.npz`~~ (supprimé) | celui du pas (5 / 5 / 5 / 1,91) | pas | les jambes du pas exact, sans la dynamique de la politique |
| `kdval_B_kd5.npz` | **5** partout | pas | le pire cas, la référence |
| `kdval_C_kd2.npz` | **2** | pas | |
| `kdval_D_kd1.npz` | **1** | pas | |
| `kdval_E_kd05.npz` | **0,5** | pas | |
| `kdval_F_kd5_kphalf.npz` | 5 | **× 0,5** | le kp joue-t-il aussi ? |

La cible des jambes et la trame d'abduction (nominale) sont les mêmes pour les six bundles. Chaque bundle a été vérifié
dans le contrôleur déployé (trame conforme à 1e-5 près après 1 s, avec des entrées bruitées) et dans
le daemon sur MockBus, avec le worker (`tools/kd_validation/test_frozen_bundles.py`, 8 tests OK).
Entre la rampe de départ et l'arrêt, tous les ticks loggés portent exactement le kp/kd déclaré.

## 4. Préparation (avant d'aller au robot)

1. **Copier les six bundles sur le Pi** (aucun redémarrage : le daemon lit le dossier à chaque
   armement). Depuis le Mac, dans `code/dash-sim2real` :
   ```bash
   scp controller/deploy/bundles/kdval_*.npz nemo@192.168.4.1:running_robot/robot/deploy/bundles/
   ```
   Ce n'est pas un déploiement de code. Le déploiement complet (étape 5 de la migration,
   `deploy_to_pi.sh --check --from deef483`) reste séparé et peut être fait avant ou après.
2. Téléphone fixé à ~20 cm de la **came droite**, application de spectrogramme qui enregistre
   (même installation que `DASH01_plan_tests_bruit_moteurs.md` §2.1). Annoncer le nom du run à voix
   haute au début de chaque run (« B, calme »).
3. Interface web fermée sur tous les appareils. Deux terminaux, comme dans la marche à suivre du
   journal du 20/11 (§6) : A sur le Pi (`cd ~/running_robot/robot`), B prêt pour l'E-STOP.

## 5. Sécurité

- **Harnais en place et tendu pour tous les runs.** Le pas figé n'a plus aucune rétroaction
  d'équilibre : en simulation, il tombe en 1,6 s. Sur le robot, il va s'asseoir sur les talons,
  comme aux runs 7–8 (−6°), ou basculer. Le harnais doit l'arrêter avant ~10°.
- Toutes les gardes du daemon restent actives. Si la pose mesurée approche du bord du workspace, le
  run se termine dans le maintien SET_POS, sans lâcher le robot.
- E-STOP physique à portée de main. Arrêter si un moteur dépasse 60 °C.
- Ctrl+C une fois = arrêt doux (maintien SET_POS debout) ; deux fois = arrêt dur (gains à zéro).

## 6. Les runs

### 6.0 Références (2 min)

- 10 s d'audio robot **LIMP** (ventilateurs, alimentation).
- 10 s d'audio en **SET_POS** debout immobile (`policy_cli.py pose kdval_B_kd5.npz`, puis poser le
  robot). C'est le niveau « sans sifflement ».

### 6.1 Série principale : debout, harnais tendu

**Avant le premier run** : alimentation vérifiée (batterie, BMS, connecteurs, drives 104 et 106), zéro
refait. Pour **chaque** bundle, dans cet ordre (kd croissant, pour que le premier run soit celui qui
tire le moins de courant) : **E, D, C, B, G, F, E (répétition)**. Si le robot s'éteint encore, même
à kd 0,5, c'est l'alimentation et pas le kd : arrêter la série.

1. `python3 tools/policy_cli.py pose kdval_X.npz` puis attendre `arrived`.
2. Poser le robot sur ses pieds, harnais tendu.
3. **Run « calme »** (bruit) :
   `python3 tools/policy_cli.py run kdval_X.npz --allow-uncalibrated-thermal --seconds 10`
   Ne pas toucher le robot pendant les 10 s.
4. Le run finit seul à 10 s, dans le maintien SET_POS debout. Noter le fichier `saved`.
5. **Run « poussées »** (kp et kd effectifs) : refaire 1 à 3, mais à ~3 s, ~5 s et ~7 s, pousser
   **doucement** le torse d'avant en arrière (1 à 2 cm, une demi-seconde), toujours de la même
   façon. Sans mouvement, la régression du kd n'a rien à mesurer.
6. Remplir la fiche (§9) : sifflement oui / faible / non, fréquence du pic audio, remarques.

14 runs, environ 30 min. Si le temps manque, on garde **E, D, B et F en calme**, c'est le cœur du
test.

### 6.2 Optionnel : suspendu, pieds libres

Il permet de séparer « gains seuls » et « chaîne fermée par le pied ». On fait `pose`, on ne pose
**pas** le robot, puis `run` avec B puis D. Comme aux runs 7–8, la politique arme debout. Sans
charge, la jambe va jusqu'à la cible, et la garde de pose mesurée peut terminer le run tôt (dans le
maintien SET_POS). Si cela arrive, faire plutôt le test 1 de `DASH01_plan_tests_bruit_moteurs.md`
(MIT hold du panneau, rôle cam/thigh, kp 200 avec kd 5 → 2 → 1). Ce test n'a pas besoin de
bundle.

### 6.3 Fin

`python3 tools/policy_cli.py limp`, robot tenu par le harnais. Récupérer les logs :
```bash
mkdir -p docs/sim2real/Logs/DASH01_kd_2026-11-24/raw
scp "nemo@192.168.4.1:running_robot/robot/fixed_gait/webui/data/policyruns/policyrun_*kdval*" \
    docs/sim2real/Logs/DASH01_kd_2026-11-24/raw/
```
Les `.npz` dépassent rarement 1 Mio. Le hook de pre-commit refuse ceux qui le dépassent ; ils vont
alors dans `dash-reports`.

## 7. Analyse

```bash
python3 tools/kd_validation/analyze_runs.py docs/sim2real/Logs/DASH01_kd_2026-11-24/raw/policyrun_*.npz \
    --labels E D C B G F E2 --out docs/sim2real/Logs/DASH01_kd_2026-11-24
```
Le script écrit `summary.md`, `summary.json` et des figures (courant et vitesse des quatre
articulations de jambe ; ondulation en fonction du kd). Pour comparer, ajouter les anciens runs
(run 5, runs 6–9) à la même commande. Par articulation :
- **bruit** : ondulation du courant (écart-type autour de la moyenne glissante sur 90 ms), ΔI par
  tick, bruit de vitesse, part des ticks où la vitesse change de signe, pic du spectre (≤ 50 Hz) et
  part de puissance au-dessus de 20 Hz ;
- **gains obtenus** : `kp_eff` et `kd_eff` par moindres carrés de τ = kp (cible − pos) − kd vel + c
  (runs « poussées »), `kp_static` = τ moyen / erreur moyenne, et la part du couple expliquée par
  la loi commandée (`law_R2`) ;
- **lutte** came/cuisse (puissances de signe opposé), tangage moyen et amplitude.

Rappel : le log est à 100 Hz et le sifflement est au-dessus de 50 Hz. Les chiffres en montrent la
trace repliée ; l'audio reste la mesure du sifflement lui-même.

## 8. Lecture des résultats

| Observation | Conclusion | Suite |
|---|---|---|
| B siffle et ondule ; C → D → E de moins en moins ; F ≈ B | **kd est la cause** (H1) | plafonner kd des jambes au déploiement (≤ 1–2, la simulation le tolère) ; réentraîner avec `kd_hi` réduit ; voir si le firmware filtre la vitesse |
| F ondule nettement moins que B (et G moins que D) | le **kp** joue aussi | plafonner aussi kp, ou pénaliser kp élevé |
| B calme, alors que la politique sifflait (runs 5–9) | ce n'est pas la trame, c'est la **dynamique** de la politique (sauts de kp par tick, consignes qui bougent) | pénaliser Δkp et Δcible à l'entraînement, filtrer les gains au déploiement |
| bruit identique de B à E | pas les gains seuls : chaîne fermée ou lutte (H2) | test suspendu (6.2), puis les tests 3–4 de `DASH01_plan_tests_bruit_moteurs.md` |
| `kd_eff` ≪ `kd_cmd`, ou `kp_eff` ≪ `kp_cmd` | le driver ne donne pas les gains demandés | à modéliser dans la simulation (gain effectif, filtre de vitesse) ; cela touche aussi la dérive avant |

Le dernier point fait le lien avec la question du sim-to-real : si les articulations réelles sont
plus molles que la commande, la politique est hors de son domaine, au même titre qu'avec l'écart de
centre de masse.

## 9. Fiche à remplir pendant le test

| run | bundle | calme / poussées | fichier `saved` | sifflement (oui / faible / non) | pic audio (Hz) | T max (°C) | remarques |
|---|---|---|---|---|---|---|---|
| REF | LIMP | — | — | non | presque rien| | le robot est silencieux s'il n'est pas posé au sol|
| REF | SET_POS | — | — | non | presque rien mais un peu plus que limp | | le robot est silencieux s'il n'est pas posé au sol|
| 1 | E | calme | | | | | |
| 2 | E | poussées | | | | | |
| 3 | D | calme | | | | | |
| 4 | D | poussées | | | | | |
| 5 | C | calme | | | | | |
| 6 | C | poussées | | | | | |
| 7 | B | calme | | | | | |
| 8 | B | poussées | | | | | |
| 9 | G | calme | | | | | |
| 10 | G | poussées | | | | | |
| 11 | F | calme | | | | | |
| 12 | F | poussées | | | | | |
| 13 | E | calme (répétition) | | | | | |
| 14 | E | poussées (répétition) | | | | | |
