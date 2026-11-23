# DASH-01 — Journal de travail du 18 au 20 novembre 2026

Suite du journal du 17/11. Sujet : pourquoi la boucle de contrôle ne tient pas 100 Hz quand la
politique d'équilibre `standing04_xshape_best` tourne sur le Raspberry Pi 3B, et comment le corriger.

---

## 1. Point de départ

- Le 17/11, la politique fonctionnait (elle redressait un torse basculé de −7 à −9°), mais la garde
  de rythme a arrêté les runs 3 et 4 : la boucle tenait **60 Hz** et **76 Hz** au lieu de 100.
- Diagnostic retenu ce jour-là : contention du verrou Python (GIL). `ctrl.step` prend 3,5 ms seul
  et, croyait-on, 9,2 ms dans le daemon. Remède prévu : le **plan B**, l'inférence de la politique
  dans son propre processus.

## 2. Décisions prises et pourquoi

| Décision | Alternatives écartées | Raison |
|---|---|---|
| Implémenter le plan B sur une branche à part (`policy-worker`, depuis `homing-v7`), un commit par étape, sans push | travailler sur `homing-v7` | pouvoir revenir en arrière ; questionnaire de cadrage au départ (branche, périmètre, commits, tests) |
| Worker **synchrone** (envoi de l'observation N, attente de la commande N, échéance de 6 ms) | utiliser la commande du tick précédent | la politique a été entraînée avec 6–18 ms de retard actionneur ; un tick de plus en sortirait |
| Un worker qui ne démarre pas **refuse l'armement** | repli silencieux en processus | un repli silencieux ressemblerait à un worker qui échoue |
| Garder `Connection.poll/recv_bytes` | un seul `recv()` avec `SO_RCVTIMEO`, ou `select` + `recv` | essayés et mesurés : aucun gain |
| `/api/policy/arm` attend 25 s dans uiproc au lieu de 4 s | garder 4 s | l'armement attend maintenant le démarrage du worker (numpy : 1–3 s sur le Pi) |
| **Garder le worker actif** (`POLICY_WORKER_VERSIONS` inchangé) malgré l'absence de gain mesuré | le désactiver tout de suite | choix de l'opérateur : chercher d'abord ce qui coûte dans le daemon |
| Corriger l'aperçu « loop rate » de l'interface pour les bundles v3 | laisser cocher `allow_slow_loop` | le bug poussait à cocher une case qui **désactive la garde de rythme** dès que la sonde dépasse 7 ms |
| Pas d'overclock, pas de 3B+ comme solution | changer de matériel tout de suite | +10–15 % au mieux, alors qu'il manque plusieurs ms ; un Pi 4 serait la vraie option matérielle, mais seulement après avoir mesuré |
| Boucle de contrôle à 150 Hz : **non** | | le tick serait encore plus court, et la politique a un `control_dt` fixe de 10 ms |
| Après la phase 1 : **CAN en socket brute, statut à 200 Hz** | statut CAN à 100 Hz | à 100 Hz, la vitesse calculée par différence de positions sur 10 ms donnerait des ticks à 0 ou 2 trames ; avec la socket brute, le gain du 100 Hz devient négligeable |
| **IMU à 120 Hz** (décision de l'opérateur) — **annulée le 20/11, voir plus bas** | 200 Hz ; 100 Hz | à 100 Hz, des ticks sans nouvel échantillon ; à 120 Hz, chaque tick en reçoit au moins un, d'un âge de 0 à 8,3 ms, pour ~0,5 ms de GIL économisé par tick |
| Remettre `right.cam` en émission périodique à 200 Hz | | il répond aux commandes au lieu d'émettre seul (voir §3.5) |
| Plan C écrit dans un fichier, implémentation dans une **nouvelle conversation** | continuer ici | contexte trop long ; le plan porte toutes les mesures |
| *20/11, plan C :* l'implémenter sur `policy-worker` | nouvelle branche `can-raw-imu120` (prévue par le plan) | consigne de l'opérateur ; un commit par étape, faciles à annuler un par un |
| Tests sur le robot **sans aucune commande moteur** : écoute passive de `can0`/`can1`, émission seulement sur une interface **virtuelle** `vcanplanc` (créée puis supprimée), code dans `~/plan_c_scratch` | redémarrer le daemon pour tester ; envoyer des trames à un id libre sur le vrai bus | consigne de l'opérateur : les jambes ne bougent pas, aucune commande n'est envoyée à un moteur |
| `RawCanBus` vide la file avec **`poll(0)`** (défaut) | lire jusqu'à l'exception `BlockingIOError` (premier choix, sur la foi du Mac) ; `select` | mesuré sur le Pi : l'exception coûte **55–58 µs**, plus qu'une trame entière ; `poll(0)` coûte **5 µs** (§3.6) |
| Garder `txqueuelen 100` (fait à chaud et dans `can-up.service`) | revenir à 10 | inoffensif et donne de la marge ; mais **ce n'est pas la cause** du `tx_dropped` de `can0` : aucune commande n'est perdue (§3.6) |
| Déployer `18d5048` avec `--force` | corriger d'abord le Pi à la main | les écarts sur le Pi n'étaient que des docstrings et de la doc, sans une ligne de code ; les versions du Pi sont sauvegardées par le script |
| Garder `DLPF_CFG = 3` | passer à 4 (~24 Hz) | bruit identique à 120 et à 200 Hz : aucun repliement mesurable |
| **Revenir à l'IMU à 200 Hz** (décision de l'opérateur, `cfd33c9` = revert de `fe9d3d6`) | garder 120 Hz ; garder 120 Hz et ajouter un retard capteur à la simulation ; lire l'IMU en phase avec le tick | à 120 Hz, l'âge de l'IMU vu par la boucle passe de **1,4 à 6,2 ms** (p50). La simulation ne modélise **aucun retard capteur**, et le gain n'était que de **3,4 points de CPU**. Le gros gain, celui du CAN (33 → 17 %), ne dépend pas de l'IMU. La lecture en phase reste la piste pour récupérer le CPU sans le retard |
| Ne pas mesurer le `dt` en MANUAL ni faire le run debout | | les moteurs y tiennent ou bougent ; c'est à faire par l'opérateur, ou sur sa demande explicite |

## 3. Ce qui a été accompli

### 3.1 Plan B (branche `policy-worker`, 9 commits depuis `1d8acb7`)

| commit | contenu |
|---|---|
| `20bcbf5` | Chronométrage par poste de chaque tick de contrôle (`measure / ctrl / gov / ws / send / log / total / rtt / wstep`) dans `policy.done`, le run terminé et le statut live |
| `d73752a` | `policy_worker.py` : processus `spawn`, épinglé sur les cœurs 2–3, protocole binaire sans pickle, client avec échéance |
| `5decb44` | Branchement dans le daemon : armement, entrée en RUN, tick (envoi → travail indépendant → attente), retards (3 de suite → arrêt doux), pannes (arrêt dur), arrêt du worker en fin de run, à la sortie de POLICY et à l'arrêt du daemon |
| `5d5483e`, `dfca68e` | Tests daemon : run complet, worker lent, tué, qui lève une exception, démarrage raté, repli, trace identique au bit près |
| `475a0cd` | Bench `controller/tools/bench_policy_worker.py` |
| `52032b6` | Documentation (README webui, commentaires du daemon) |
| `5d28969` | uiproc : 25 s pour `/api/policy/arm` |
| `1478170` | Aperçu « loop rate » : construit les bundles v3 ; ceux servis par le worker sont chronométrés dans le worker (**pas encore déployé**) |

Tests : suite complète à 516 passés, 7 échecs, tous connus. Équivalence bit à bit worker = local
vérifiée deux fois : sur 200 pas d'un bundle synthétique, et sur la trace d'un run MockBus réel.
Rapport : `SimToReal/DASH01_plan_B_RAPPORT.md`.

**Déployé sur le Pi le 19/11** (`5d28969`, les deux services redémarrés).

### 3.2 Le worker ne gagne rien sur le Pi (bench, sans moteurs)

| charge simulée | en processus | worker | réponses en retard |
|---|---|---|---|
| aucune | 99,9 Hz, ctrl p50 3,4 ms | 99,7 Hz, ctrl p50 4,9 ms | 15 |
| légère | 99,1 Hz, max 11,7 ms | 99,2 Hz, max 8,9 ms | 19 |
| défaut (2 fils × 1,5 ms) | 87,6 Hz | 89,1 Hz | 93 |

Le worker calcule un pas en 3,0–3,6 ms sur les cœurs 2–3, mais l'aller-retour et la reprise du GIL
coûtent ~1,5 ms par tick, et il rate son échéance de 6 ms sur 2 à 10 % des ticks. Sur le Mac, avec
l'intervalle GIL du daemon (0,5 ms, déjà réglé dans `server.py`), il n'y avait pas non plus de gain
net. Le Mac ne reproduisait d'ailleurs pas le problème. Le worker partage aussi les cœurs 2–3 avec
uiproc et une interruption SPI.

### 3.3 Relecture des runs 3–4 (`SimToReal/Logs/`, black box du Pi)

| run | sonde à l'armement | `slow_loop` | arrêt |
|---|---|---|---|
| 1 | 9,16 ms | vrai | vitesse angulaire 16,1 rad/s (bug des °/s, corrigé) |
| 2 | 5,32 ms | faux | pose mesurée à 0,9° du bord du workspace |
| 3 | 5,47 ms | faux | 60 Hz, 88 ticks en retard sur 100 |
| 4 | 4,91 ms | faux | 76 Hz, 72 ticks en retard sur 100 |

- Pour les runs 3–4, la case « run the control law SLOWER » était cochée, à cause du bug de
  l'aperçu, mais **sans effet** : la sonde était sous 7 ms, et la garde de rythme était active.
- **Correction des analyses du 17/11** : le « 9,2 ms dans le daemon » vient du run 1, pas des runs 3–4.
  La sonde (~5 ms) ne prédit pas le coût réel d'un tick.
- Run 4 : le tick de **248 ms** a lieu pendant l'**armement** (thread HTTP), pas pendant le RUN.
  Pendant le RUN, environ 1 tick sur 3 dépasse 12 ms, sans périodicité : la boucle est en
  dépassement permanent.

### 3.4 Enquête sur la boucle du daemon (phase 1, lecture seule, robot LIMP)

- **CPU par thread au repos** : boucle CAN **32–36 %** (~3,6 ms par tick sans politique), IMU 12,6–14 %,
  Flask 7,5 %, black box 2,6–4 %. La boucle CAN est préemptée 626 fois par seconde : les cœurs 0–1
  sont saturés. L'IMU et la boucle se disputent le GIL.
- **python-can est le coupable principal** : 88 µs par trame reçue (p50, 245 µs en p90), contre
  **8 µs** avec une socket `AF_CAN` brute, soit 11 fois moins. ~12 trames par tick, donc ~1,6 ms en
  p50 et plus de 4 ms en p90 pour la seule lecture, sans compter les envois.
- Contrôleurs MCP2518FD : 0 erreur. Mais **`can0` perd ~3 trames par seconde en émission**
  (`txqueuelen 10`).
- `time.sleep` et les interruptions : pas en cause.
- Simulation : seul le retard **actionneur** est modélisé (6–18 ms). **Aucun retard capteur.**

Hypothèses encore ouvertes : H2 (le coût réel de l'IMU sur la boucle, non mesuré activement),
H3 (le governor), H4 (la black box), H5 (Flask), H6 (le ramasse-miettes). Le chronométrage par poste
d'un run debout permettra de les départager.

### 3.5 `right.cam` (drive 0x69 sur `can0`)

Mesure passive avec horodatage noyau : `right.abd` et `right.thigh` émettent toutes les **5,00 ms
pile**, indépendamment des commandes. `right.cam` **répond à chaque commande 0,25 à 0,66 ms
après**, avec une période de 9,1 à 10,7 ms, celle de la boucle. C'est un réglage du drive
(émission périodique désactivée), à corriger dans l'outil CubeMars. Effets : sa position a ~1 tick
de retard, et sa télémétrie s'arrête quand la boucle cale. Les « telemetry stale » vus dans la
black box viennent probablement de là.

### 3.6 Plan C (20/11) : CAN en socket brute, IMU essayée à 120 Hz puis remise à 200 Hz

Rapport détaillé : `SimToReal/DASH01_plan_C_RAPPORT.md` (§6 à §8). Données brutes :
`SimToReal/Logs/plan_C/` (`probe_0_before.json`, `probe_1_txq100.json`, `probe_2_after.json`,
`probe_3_imu200.json`, `noise_200hz.json`, `noise_120hz.json`, les deux `.npz` de bruit,
`vcan_check.py`).

**Commits (branche `policy-worker`, non poussés)**

| commit | contenu | sur le Pi |
|---|---|---|
| `c5bccc3` | `controller/tools/can_probe.py` : sonde passive (elle écoute, n'émet jamais) | oui |
| `787475e` | `canio.RawCanBus` : socket `AF_CAN` brute pour le daemon ; retour arrière : `--can-backend python-can` | oui |
| `05cdc16` | doc : `txqueuelen 100` dans l'exemple `can-up.service` | (le réglage lui-même : oui) |
| `fe9d3d6` | IMU à 120 Hz | déployé, puis **annulé** |
| `18d5048` | `RawCanBus` : `poll(0)` par défaut ; README corrigé sur `tx_dropped` | oui |
| `cfd33c9` | **retour à l'IMU à 200 Hz** (revert de `fe9d3d6`, avec un commentaire qui garde les chiffres) | **oui, version actuelle** |

Déploiements (robot en LIMP, sur accord de l'opérateur) : `1478170` → `18d5048`, sauvegarde
`~/deploy_backups/20261120_110735_1478170_to_18d5048` ; puis `18d5048` → `cfd33c9`, sauvegarde
`~/deploy_backups/20261120_112011_18d5048_to_cfd33c9`. Après chacun : LIMP, 6 moteurs vivants à
0 A, aucune erreur CAN. Tests : suite complète sur le Mac à 553 passés, 7 échecs, tous connus ;
tests du plan C sur le Pi à 57/57.

**Coût de lecture CAN sur le Pi** (`can_probe --bench`, trafic réel, vidange toutes les 10 ms)

| méthode | par trame p50 | lecture à vide p50 | vidange d'un bus p50 |
|---|---|---|---|
| python-can `recv(timeout=0)` | 90 µs | 17 µs | **0,84–1,21 ms** |
| socket brute + `poll(0)` | 14 µs | **5 µs** | **0,15–0,19 ms** |
| socket brute + exception | 10 µs | **55–58 µs** | 0,17–0,20 ms |
| socket brute + `select` | 20 µs | 9 µs | 0,24–0,37 ms |

Sur l'interface virtuelle, pour un tick type du daemon (vidange de 12 trames, puis 6 envois) :
python-can **2,11 ms** p50, `RawCanBus` **0,69 ms**. Les octets émis sont **identiques** à ceux de
python-can pour `set_current`, `set_pos` et `force_control`, et les statuts se décodent pareil.

**`tx_dropped` de `can0`** : ~2,4–3,7 par seconde, avec une file de 10 comme de 100. Sur 10 s, le
noyau a accepté 6 012 trames et les interfaces en ont émis 6 013. Le daemon ne voit aucune erreur
d'envoi, et ses commandes reviennent en écho à 300/s sur chaque bus. **Aucune commande n'est
perdue** : le compteur vient du pilote `mcp251xfd` de `can0`. Cause encore ouverte (`can1` : 0).

**Daemon au repos (LIMP), trois états**

| | avant le plan C | CAN brut + IMU 120 Hz | **CAN brut + IMU 200 Hz (actuel)** |
|---|---|---|---|
| CPU de la boucle CAN | 32–33 % | 17,1 % | **17,0 %** |
| CPU du thread IMU | 13,2 % | 9,8 % | **13,4 %** |
| `dt` p90 / p99 | 10,31–10,55 / 11,6–11,8 ms | 10,04 / 10,82 ms | **10,11 / 11,45 ms** |
| `dt` > 12 ms / max | 0,45–0,79 % / 35–41 ms | 0,29 % / 15 ms | **0,59 % / 16 ms** (2 min) |
| âge de l'IMU vu par le tick, p50 / p99 | **1,38 / 5,57 ms** | **6,21 / 10,27 ms** | **3,29 / 8,04 ms** |

**Bruit IMU** (60 s, robot immobile, `DLPF_CFG` 3) : accéléromètre ~2,0–2,1 mg RMS et gyroscope
~0,08 °/s RMS, **identiques** à 200 et à 120 Hz. L'attitude filtrée est un peu plus bruitée à
120 Hz : 0,062° contre 0,055° RMS en tangage.

**Pourquoi l'IMU est revenue à 200 Hz.** L'hypothèse du plan, « un âge de 0 à 8,3 ms », oubliait
que `fast()` est horodaté **avant** la lecture I2C (~1,7 ms). À 120 Hz, l'âge médian a été
multiplié par 4,5, pour un gain de seulement 3,4 points de CPU. Or la politique n'a jamais vu de
retard capteur en simulation.

**Ce que montre le retour à 200 Hz.** 200 Hz contre 100 Hz est un rapport entier : les deux
boucles restent calées sur un déphasage **fixé au démarrage du service**. L'âge médian change donc
d'un redémarrage à l'autre, avec 1,4 ms dans une session et 3,3 ms dans la suivante, sur une plage
possible d'environ 1,7 à 6,7 ms. Lire l'IMU juste avant chaque tick le rendrait petit et constant.

### 3.7 Ce qui tourne sur chaque cœur, et ce qu'on peut encore gagner (20/11, après `cfd33c9`)

Mesures en lecture seule, robot en LIMP. Scripts : `SimToReal/Logs/plan_C/cores.py` (charge par
cœur, par thread et interruptions sur 10 s) et `bench_gov.py` (`ctrl.step` et governor, calcul pur
sur le cœur 3 avec le vrai bundle `standing04_xshape_best`, sans CAN).

**Placement**

- Le daemon (`server.py`) est épinglé sur les cœurs **0–1** (`CPUAffinity=0 1`, `nice -10`).
- uiproc est épinglé sur les cœurs **2–3** (`nice 5`). Le worker de politique s'y place aussi quand
  il démarre.
- Le daemon est un seul processus Python, donc **un seul GIL** : ses 5 threads se partagent au plus
  un cœur de Python. Le second cœur absorbe les attentes hors Python (I2C, `sleep`, appels système).
- Gouverneur de fréquence CPU : `ondemand`. Aucune isolation de cœur au démarrage du noyau.

**Charge au repos, mesurée sur 10 s**

| cœur | charge | ce qui tourne |
|---|---|---|
| 0 | 23 % | daemon : boucle CAN, environnement ; thread noyau `spi1` (temps réel, priorité 50) |
| 1 | 24 % | daemon : IMU, Flask, black box ; thread noyau `spi0` (temps réel, priorité 50) |
| 2 | 9 % | interruptions SPI de `can0` en thread (6,8 %), uiproc (6 %) |
| 3 | 3 % | interruptions SPI de `can1` en thread (7,8 %) |

| thread du daemon | CPU | par période |
|---|---|---|
| boucle CAN (100 Hz) | 17 % | **1,7 ms par tick** (3,3 ms avant le plan C) |
| IMU (200 Hz) | 13 % | 0,65 ms par échantillon, soit 1,3 ms tous les 10 ms |
| Flask | 8 % | 0,8 ms tous les 10 ms |
| black box | 3 % | |
| capteurs lents | 1 % | |
| **total sous le GIL** | **~42 % d'un cœur** | |

Interruptions : elles sont réparties sur **les 4 cœurs**, ceux du daemon compris. USB `dwc_otg` :
~2 000/s par cœur. SPI : ~200/s par cœur et par bus. I2C : ~190/s par cœur. Elles ne coûtent que
~0,2 % de CPU, mais la boucle CAN est préemptée ~360 fois par seconde.

**Un tick avec la politique (estimation, le run debout le confirmera)**

| étape | durée | origine |
|---|---|---|
| lecture CAN et tenue des comptes | ~1,7 ms | mesuré en LIMP |
| observation, garde workspace, journal | < 1 ms ? | **inconnu** |
| `ctrl.step` | **3,05 ms p50 / 3,59 p90** | mesuré, cœur 3 libre |
| governor (`SafetyGovernor.step`) | **0,64 / 0,94 ms** | mesuré, cœur 3 libre. Le « ~1,9 ms » d'avant était sous contention |
| envoi des 6 commandes | ~0,2 ms | mesuré sur le CAN virtuel |

- **Politique dans le daemon** : ~6 ms de Python par tick, plus l'IMU, Flask et la black box, soit
  **~8,5 ms de GIL sur 10**. Ça devrait passer au p50, mais c'est serré au p90. Avant le plan C, on
  dépassait 10 ms.
- **Avec le worker** : le réseau tourne sur les cœurs 2–3, hors GIL, plus ~1,5 ms d'aller-retour.
  Le tick dure ~7,5 ms, pour ~55 % de GIL.
- **Seul `timing_ms` d'un run debout tranchera.**

**Pistes d'optimisation, de la plus rentable à la plus lourde**

| # | piste | gain estimé | effort, risque |
|---|---|---|---|
| 1 | run debout de 3 s avec et sans worker, et lire `timing_ms` | (mesure) | gratuit ; tout le reste en dépend |
| 2 | lire l'IMU **en phase avec le tick**, à 100 Hz, juste avant chaque tick | ~0,6 ms de GIL par tick ; âge IMU constant ~2 ms au lieu de 1,4–6,7 ms selon le démarrage | moyen (`sensehat.py` et daemon). Annexe : ne construire le dictionnaire `sample` qu'à `LOG_HZ` |
| 3 | gouverneur CPU `performance` ; déplacer les threads `spi0`/`spi1` et les interruptions vers les cœurs 2–3 | surtout sur les pics, moins de préemptions | commandes root, pas de code ; surveiller la température |
| 4 | Flask : couper ou ralentir le flux de l'interface pendant POLICY | ~0,6 ms par tick | faible |
| 5 | accélérer `ctrl.step` (onnxruntime, déjà dans les dépendances, ou calculs regroupés) | 3 → ~1 ms ? | moyen ; à vérifier contre les tests golden |
| 6 | governor en calcul scalaire (comme Madgwick, ~40× plus rapide) | 0,64 → ~0,1 ms | `controller/deploy/safety.py`, code de sécurité : seulement sur décision, avec équivalence au bit près |
| 7 | IMU dans son propre processus (cœurs 2–3, presque libres) | −13 % de GIL | gros chantier |
| 8 | matériel (Pi 4 ou 5) | ×2–3 estimé, non mesuré | dernier recours |

Ordre recommandé : 1, puis 2 et 3.

### 3.8 Run 5 (20/11) : premier run debout après le plan C

Analyse complète : `SimToReal/Logs/DASH01_run5_2026-11-20/ANALYSE.md`, avec la black box brute, le
fichier du run et le journal systemd.

- **83 Hz** (runs 3 et 4 : 60 et 76 Hz). La garde de rythme arrête le run après 1,21 s.
- Robot redressé de −6,7° à +0,9° sans dépassement. Il **reste debout**.
- **Ce n'est pas le calcul qui coince.** En p50, `total` vaut 7,6 ms (`ctrl` 3,75 avec le worker,
  `gov` 1,42, `send` 1,17, `ws` 0,59), et les ticks hors perturbation sont à 10 ms pile.
- Les 205 ms perdues ont trois causes, corrélées à la milliseconde au journal :
  - **`GET /api/state_cold`** : 104 ms. uiproc vide son cache à chaque POST, donc à chaque
    `keepalive` (5 fois par seconde) pendant un run. Chaque `/api/state` refait alors ~44
    lectures de la carte SD dans le daemon : 35 à 68 ms sous le GIL.
  - **`keepalive`** : 31 ms, soit ~6 ms par appel.
  - **Un tick de 72 ms** non expliqué : 62 ms. Le ramasse-miettes est suspect.
- Sans les deux causes HTTP, 92,6 Hz. Sans les trois, 98,7 Hz.

| Décision proposée | Raison |
|---|---|
| Corriger d'abord uiproc : ne pas vider le cache de `state_cold` sur `keepalive`, sur `thermal/identify/plan`, ni pendant POLICY | 104 ms sur 205, correction côté affichage seulement, risque faible |
| Puis alléger le `keepalive` et instrumenter le GC | 31 + 62 ms : ce qu'il manque pour viser 99 Hz |

### 3.9 Le sifflement des moteurs pendant la politique (20/11)

Constat de l'opérateur : pendant la politique, les moteurs émettent un bruit strident et aigu.
Analyse : `SimToReal/Logs/ANALYSE_bruit_moteurs.md`. Plan de tests :
`SimToReal/DASH01_plan_tests_bruit_moteurs.md`.

- **Ce n'est pas une limite de courant.** Les cames montent à 22–27 A (un pic à 52 A), alors que les
  mêmes drivers ont sorti ~100 A dans la même session. Pas de plafond, pas de code d'erreur, pas de
  saturation du governor, 54 °C au plus.
- **Le courant des cames ondule en mode MIT** : 4 à 6 A rms en POLICY, contre 0.01–0.07 A en SET_POS
  immobile. À vitesse égale, c'est 4 à 50 fois plus qu'en SET_POS. Les cuisses ondulent 3 à 5 fois
  moins. La vitesse renvoyée par la came change de signe d'un échantillon à l'autre : le rotor vibre.
- **La came et la cuisse se battent à travers le pied.** La politique met la consigne de la came
  3 à 7° devant sa position et celle de la cuisse 2 à 5° derrière. Leurs puissances sont de signe
  opposé 52 à 80 % du temps en POLICY, et 0 à 5 % en SET_POS. Le bruit de la came augmente avec cette
  lutte (corrélation de 0.2 à 0.6), sans que ce soit une preuve de cause.
- **Run 5 (83 Hz, plan C)** : même ondulation des cames (4.6–5.4 A par tick), donc le rythme de la
  boucle n'en est pas la cause. Son fichier de run donne enfin les gains demandés à chaque tick :
  - le **kd des cames est collé à 5**, le maximum, sur 76 % (gauche) et 99 % (droite) des ticks ;
  - le **kp saute de 40 à 55 N·m/rad d'un tick à l'autre** sur les cames. Avec ~3° d'erreur, cela fait
    des marches de couple de ~2–3 N·m rms (jusqu'à 8 N·m), une deuxième source, plus petite ;
  - la politique **baisse le kp de la cuisse gauche** (médiane 86 contre 200 au handover), ce qui
    explique que son courant bouge peu malgré son erreur.
- **Hypothèses** : H1 les gains (kd 5 amplifie le bruit de vitesse du driver et le jeu de la
  transmission de la came), H2 la lutte came/cuisse à travers la chaîne fermée, H3 les deux. La black
  box (100 Hz) ne voit pas le sifflement : il faut des tests avec le son enregistré.
- **Plan de tests** (option « MIT hold » de MANUAL, trims, `/api/blackbox/mark`) :
  1. suspendu, consigne fixe, kp 200 avec kd 5, 2, 1 ;
  2. debout, à la stance d'entraînement (came +3.9°, cuisse −1.5°), kp 200 / kd 5 ;
  3. debout, lutte imposée en deux paliers (jusqu'à came +7°, cuisse −5°) ;
  4. la même lutte avec kd 2 puis 1.

  Point délicat : à l'engagement du MIT debout, la jambe s'affaisse d'environ 6°. Le plan propose
  de l'engager pendant que le harnais porte le poids (option A, sans code) ou d'ajouter un mode qui
  tient la trame de handover (option B).
- **Reprogrammer les drivers CubeMars** : le logiciel « upper computer » AK (firmware 3.0) n'existe
  que pour Windows et demande l'adaptateur R-Link. Sur Mac : un PC Windows (le plus fiable pour
  flasher), une machine virtuelle avec le R-Link transmis en USB (risque de pilote sur Apple
  Silicon, ne pas flasher ainsi), ou Boot Camp sur un Mac Intel. VESC Tool ne gère que les anciens AK
  à firmware VESC. L'exécutable est dans le dépôt : `controller/tools/cubemarstool_v3.2.0.exe`.
  Pour le sifflement, il n'y a peut-être rien à flasher : kp et kd partent dans chaque trame MIT.

### 3.10 Lancement depuis le terminal et runs 6 à 9 (20/11)

**Outil** : `controller/tools/policy_cli.py` (commits `d6bfcc6`, `db57ccc`, `39612bf`, copiés sur le
Pi sans redémarrer le service). Il envoie les mêmes requêtes que les boutons de l'interface :
- `pose FICHIER` : MANUAL et mouvement guidé vers la pose de départ ;
- `run FICHIER --allow-uncalibrated-thermal` : confirmation du harnais, armement, homme mort tenu à
  2,5 Hz, sauvegarde du log, tableau des temps ;
- `stop`, `estop`, `status`.

Les runs sont plafonnés à 10 s. Ctrl+C déclenche un arrêt doux, deux Ctrl+C un arrêt dur. Toutes
les gardes restent dans le daemon.

Deux corrections après le premier essai :
- la pose est atteinte quand toutes les articulations sont à ≤ 0,5° de leur cible pendant 1,5 s.
  Le drapeau `homing` ne compte plus : le daemon le garde levé tant qu'il **maintient** la pose ;
- « criterion MET » n'est plus affiché pour un run arrêté avant la fin.

**Runs 6 à 9** : analyse dans `SimToReal/Logs/DASH01_runs6-9_2026-11-20/ANALYSE.md`.

| run | lancé depuis | rythme | fin | tangage début → fin |
|---|---|---|---|---|
| 6, 7, 8 | web | 88,5–88,8 Hz | garde de rythme | 6 : −6,4 → +7,4° ; 7 et 8 : restent vers −6° |
| **9** | **terminal** | **100,0 Hz**, `total` p90 8,9 ms | **arrêt dur après 0,88 s** | **+0,1 → +14,1°** |

1. **Le rythme est réglé.** Lancée depuis le terminal, la boucle tient 100 Hz, presque tous les ticks
   à 10 ms. Le calcul de la politique n'a pas changé ; seules les interruptions de la page web ont
   disparu.
2. **Les drives `left.abd` (104) et `left.thigh` (106) se sont tus ensemble pendant ~120 ms**.
   `left.cam` (105), sur le même `can1`, continuait. Le bus ne montre aucune erreur, et les drives
   sont repartis seuls. Ils ont **cessé d'émettre** : alimentation, connecteur ou réinitialisation
   à vérifier sur la jambe gauche. Des épisodes « telemetry stale » existent dans des sessions plus
   anciennes. `can1` affiche aussi `rx_dropped` = 5 727, de cause inconnue (`can0` : 0).
3. **La politique ne retient pas le robot vers l'avant.** Dans les runs 5, 6 et 9, le tangage passe
   0° et continue vers l'avant en accélérant (+24°/s à la fin du run 9). Les « redressements »
   depuis −6,5° n'en étaient peut-être pas. Hypothèses : centre de masse plus en avant que dans la
   simulation, biais de la politique ou décalage d'observation (zéro repris aujourd'hui), inversion
   de signe (peu probable).

| Décision proposée | Raison |
|---|---|
| Lancer les runs depuis le terminal, onglet web fermé | 100 Hz tenus, sans toucher au daemon |
| Avant de chercher des runs plus longs : comprendre la dérive en avant | c'est elle qui fait tomber le robot, et le rythme n'est plus en cause |
| Vérifier l'alimentation et les connecteurs des drives 104 et 106 de la jambe gauche | silence simultané de deux drives sur trois, sans erreur de bus |

### 3.11 Pourquoi le robot part en avant : analyse hors ligne (20/11, fin de journée)

Détail : `SimToReal/Logs/DASH01_runs6-9_2026-11-20/ANALYSE.md` §4. Script :
`replay_policy_offline.py`, avec le vrai bundle (md5 `282a0f31…`) dans le vrai contrôleur.

**Faits établis**
- Le rejeu est fidèle : les observations du run 9 redonnent les consignes enregistrées à 0,17° près.
- **Le réflexe de tangage est désactivé** dans `standing04_xshape_best` (`reflex.enable = false`).
  Tout le redressement vient du réseau.
- Quand le robot penche en avant, la politique **corrige dans le bon sens mais peu** (1 à 2° aux
  cuisses pour 14° de tangage). Surtout, elle **assouplit les cuisses** : kp 105 / 207 → 29–62
  N·m/rad au run 9, 47 / 68 au run 6.
- Les hanches cèdent (5 à 6° d'écart à la consigne) et le robot se plie. Les moteurs ne sont pas en
  limite (8 à 13 N·m), et le governor n'a rien bridé.
- **Runs 7 et 8 : couples presque nuls.** Le robot pendait dans le harnais ; ces runs ne disent rien
  de l'équilibre.

**Ce que la politique a vu à l'entraînement** (`BalanceRL/config.py`, `BalanceRL/README.md`)
- Bassin d'équilibre : ±0,02 rad (±1,1°) de cuisse. En simulation, la pose de départ tient seule.
- Variations simulées : montage IMU ±1°, sol ±1°, zéro articulaire ±0,5°, centre de masse ±3 cm.
- Même dans ces conditions, elle tombe seule dans 4 à 7 % des épisodes : elle est fragile par
  construction (semelle de 8,6 cm).

**Le robot réel est hors de ce domaine.** L'IMU est calibrée sur la vraie verticale (inclinomètre,
fait par l'opérateur). Pourtant, dans la pose de départ, le robot réel bascule sur les talons à
**−6,5°**, alors que le robot simulé y tient seul. Le modèle et le robot diffèrent : centre de masse
réel plus en arrière, géométrie du pied, ou pose. Chaque run démarre hors de ce que la politique
connaît, et ce qu'elle fait alors (assouplir les cuisses) n'est pas fiable.

**Décisions**

| Décision | Alternative écartée | Raison |
|---|---|---|
| **Ne pas remettre l'IMU à zéro robot debout** | zéro pris à −6,5° sur les talons | l'IMU est déjà calibrée sur la vraie verticale. Un zéro debout injecterait une erreur de ~6,5°, six fois les ±1° vus à l'entraînement, et cacherait l'écart physique sans le corriger |
| Mesurer l'écart réel avant tout nouveau run | relancer des runs tels quels | sans pose de départ équilibrée, chaque run commence hors du domaine de la politique |
| Si l'écart de centre de masse dépasse ~3 cm : le mettre dans la simulation et réentraîner, plage de ±3 cm centrée sur la vraie valeur | retoucher les capteurs ou la politique à la main | c'est la seule correction qui change la physique vue par la politique |

**Marche à suivre proposée (prochaine session)**
1. Robot dans le harnais, sangle détendue, en MANUAL. Ajuster les cuisses jusqu'à ce que le robot
   tienne sur ses pieds à plat, tangage ≈ 0° (vraie verticale).
2. Noter l'écart de cuisse par rapport à `default_motor_pos` du bundle. Il se convertit en centre de
   masse : ~0,04 rad de cuisse ≈ 3 cm.
3. Écart < ~3 cm : démarrer les runs depuis cette pose équilibrée. Écart plus grand : réentraîner
   avec le centre de masse mesuré.
4. Point ouvert, run 9 : parti droit (+0,1°) mais avec peu de couple au départ (6,5 N·m au cam,
   contre 17 au run 5). Le harnais portait probablement le robot au début. Refaire un run parti
   d'une pose équilibrée, sangle détendue.

Outillage possible : une commande `policy_cli.py` qui affiche en direct le tangage et l'écart de
cuisse pendant la recherche de l'équilibre. Pas encore écrite.

## 4. Recettes utiles

- **CPU par thread du daemon** :
  `P=$(systemctl show -p MainPID --value runningrobot-webui)`, puis lire `/proc/$P/task/*/stat`
  (champs 14+15) à deux instants. Ordre des threads : black box, boucle CAN, IMU, environnement,
  Flask.
- **Statistiques CAN** : `ip -s -d link show can0` (`tx_dropped`, erreurs).
- **Sondes passives** : socket `AF_CAN` brute en lecture seule, `SO_TIMESTAMP` = 29 (numérique, la
  constante manque dans Python sur le Pi). C'est maintenant `controller/tools/can_probe.py` :
  `cd ~/running_robot/robot && ../.venv/bin/python tools/can_probe.py --seconds 20 --bench --threads --json ~/probe.json`.
  La sonde fait l'écoute, le bench de lecture et le CPU par thread, sur les cœurs 2–3.
- **Pertes réelles ou pas** : comparer `/proc/net/can/stats` (trames acceptées par le noyau) à
  `tx_packets` des interfaces sur 10 s.
- **`dt` et âge de l'IMU vus par la boucle** : dans le dernier segment `A_*.bbseg` de la black box,
  lu par `blackbox.read_segment`, les colonnes `dt` et `imu[:, 5]` (âge). Tier A est échantillonné à
  20 Hz.
- **Bruit IMU** : `POST /api/sensors/noise` avec `{"action":"start"}`, 60 s robot immobile, puis
  `{"action":"stop"}`. Ne touche que l'IMU ; le résultat est rangé dans `data/imu_noise/`.
- **Charge par cœur, par thread et interruptions** :
  `scp SimToReal/Logs/plan_C/cores.py nemo@192.168.4.1:` puis `taskset -c 3 python3 cores.py`
  (10 s). Affinités : `systemctl show -p CPUAffinity --value runningrobot-webui`, et
  `taskset -pc <tid>`.
- **Coût de `ctrl.step` et du governor sans CAN** : `SimToReal/Logs/plan_C/bench_gov.py`, lancé
  avec le Python du venv du robot. Il s'épingle seul sur le cœur 3.
- **Tester une émission CAN sans moteur** : `sudo modprobe vcan`,
  `sudo ip link add dev vcanplanc type vcan`, `sudo ip link set vcanplanc up`, puis `ip link del`
  une fois fini.

## 5. Suite

**État en fin de journée (20/11).**
- Le Pi tourne sur `cfd33c9` pour le daemon : CAN brut, IMU à 200 Hz. Le service n'a pas été
  redémarré depuis.
- `tools/` est sur `b25d404` (`policy_cli.py` avec `limp`).
- `can-up.service` : `txqueuelen 100`.
- Dépôt : branche `policy-worker`, HEAD `b25d404`, arbre propre, rien de poussé.

**Priorité n°1 (§3.11)** : mesurer la vraie pose d'équilibre et l'écart de centre de masse, puis
démarrer depuis une pose équilibrée ou réentraîner. Ne pas remettre l'IMU à zéro debout.

Plan C : fait et déployé (§3.6).
- **`right.cam`** en émission périodique à 200 Hz dans l'outil CubeMars (l'opérateur), puis
  `can_probe.py` pour vérifier `periodic 200 Hz`.
- **Run 5 fait (§3.8) : 83 Hz.** Le cache `state_cold` d'uiproc reste à corriger pour les runs
  lancés depuis le web. Le terminal (§3.10) contourne le problème : 100 Hz au run 9.
- **Dérive du tangage vers l'avant** : rejeu hors ligne fait (§3.11). Reste la simulation : le même
  bundle dans BalanceRL, départ à −6,5° ou centre de masse décalé (mujoco et jax à installer sur
  le Mac, ou cluster).
- **Matériel** : le silence de 120 ms des drives 104 et 106 sur `can1`, et le `rx_dropped` de
  `can1`.
- **Retard capteur** : lire l'IMU en phase avec le tick de contrôle (piste 2 de §3.7), et/ou
  randomiser un retard capteur de 0 à 7 ms en simulation.
- **CPU** : les pistes de §3.7, dans l'ordre 1, 2, 3.
- Si le critère n'est pas atteint : chronométrage par poste, hypothèses H3 à H6.
- Ouvert : la cause du `tx_dropped` de `can0` (pilote).
- **Sifflement des moteurs** (§3.9) : faire les 4 tests de `DASH01_plan_tests_bruit_moteurs.md`,
  son enregistré, en harnais. Choisir avant le test 2 entre l'option A (harnais) et l'option B (code).

## 6. Marche à suivre : lancer la politique sans l'interface web

Outil : `tools/policy_cli.py` sur le Pi (version `b25d404`). Toutes les sécurités restent dans le
daemon. Un run dure **10 s au maximum**.

**Avant de commencer**
- Robot dans le harnais, moteurs alimentés, E-STOP physique à portée de main.
- **Onglet de l'interface web fermé**, sur tous les appareils (ordinateur, téléphone).
- Deux terminaux ouverts sur le Mac.

**1. Terminal A (pilotage) : se connecter au Pi**
```
ssh nemo@192.168.4.1
cd ~/running_robot/robot
```

**2. Terminal B (sécurité) : préparer l'arrêt d'urgence**
Taper sans appuyer sur Entrée, et ne l'envoyer qu'en cas de besoin :
```
curl -X POST http://192.168.4.1:8081/api/estop
```

**3. Vérifier l'état (terminal A)**
```
python3 tools/policy_cli.py status
```
Attendu : `mode LIMP`, `estop False`, `can_errors {}`, et une position pour chacun des six moteurs.

**4. Aller à la pose de départ, robot suspendu**
```
python3 tools/policy_cli.py pose standing04_xshape_best.npz
```
Attendre `arrived (every joint within 0.5 deg)`. Le robot tient la pose en MANUAL.

**5. Poser le robot sur ses pieds**, sans retirer le harnais.

**6. Lancer le run**
```
python3 tools/policy_cli.py run standing04_xshape_best.npz --allow-uncalibrated-thermal
```
- Répondre `y` à la question (harnais, E-STOP, interface fermée).
- Une ligne s'affiche par seconde : la phase, le temps écoulé, le rythme.
- **Ctrl+C une fois = arrêt doux** : le robot est reposé en douceur.
- **Ctrl+C deux fois = arrêt dur** : gains à zéro, le robot tombe dans le harnais.
- Si le terminal ou le Wi-Fi se coupe, le daemon fait un arrêt doux seul, en 1,5 s.

**7. Lire le résumé affiché à la fin**
- `saved` : nom du fichier du run (`data/policyruns/policyrun_….npz`) ;
- `ended` : pourquoi le run s'est terminé ;
- `rate` : rythme réel ;
- tableau des temps par étape ;
- `criterion` : `MET` ou `not met` si le run est allé au bout, `NOT EVALUATED` s'il s'est arrêté
  avant.

**8. Relâcher les moteurs**, robot tenu par le harnais
```
python3 tools/policy_cli.py limp
```
Répondre `y`.

**9. Récupérer les logs** : me demander « pull les logs », ou, depuis le Mac :
```
scp "nemo@192.168.4.1:running_robot/robot/fixed_gait/webui/data/policyruns/policyrun_*" .
```

**Options utiles**
- `--seconds 5` : un run plus court (10 au maximum) ;
- `--no-worker` : la loi de commande dans le daemon, pour comparer ;
- `stop` : arrêt doux, `stop --hard` : arrêt dur ;
- `estop` : E-STOP depuis le terminal A.

**Si quelque chose bloque**
- `another client is controlling the robot` : l'interface web a encore la main. La fermer et
  attendre 15 s, ou ajouter `--takeover`.
- `pose refused …` ou `arm refused …` : le message vient du daemon et dit ce qui manque
  (calibration, workspace, une case à confirmer…).
- `the robot is in LIMP, not MANUAL` : refaire l'étape 4.
- Après un E-STOP, pour le lever : `curl -X POST http://192.168.4.1:8081/api/estop/clear`, puis
  reprendre à l'étape 3.
