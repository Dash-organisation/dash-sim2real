# DASH-01 — Plan C : rapport d'exécution

2026-11-20. Plan : `SimToReal/DASH01_plan_C_can_imu.md`. Dépôt : `code/DASH-01`.

## 1. Ce qui est fait

Cinq commits sur la branche **`policy-worker`**, au-dessus de `1478170`. Rien n'est poussé.
**Le Pi tourne sur `cfd33c9` depuis le 20/11 : CAN brut, IMU revenue à 200 Hz (§7–8).** Les tests sur le robot (§6) n'ont envoyé **aucune trame sur
`can0` ni sur `can1`** et n'ont pas redémarré le service.

| étape | commit | contenu |
|---|---|---|
| 0 | `c5bccc3` | `controller/tools/can_probe.py` : sonde passive (elle écoute, n'émet jamais) |
| 1 | `787475e` | `canio.RawCanBus` : socket `AF_CAN` brute pour le daemon, interrupteur `--can-backend` |
| 2 (doc) | `05cdc16` | exemple `can-up.service` dans `webui/README.md` : `txqueuelen 100` |
| 3 | `fe9d3d6` | IMU à 120 Hz, un seul réveil par échantillon |
| 1 bis | `18d5048` | `RawCanBus` vide la file avec `poll(0)` par défaut (mesuré sur le Pi, §6) ; correction du README sur `tx_dropped` |

`graphify update code` a été relancé après les modifications.

### Étape 0 — `can_probe.py`

- **listen** (par défaut). Par interface : les trames reçues par seconde et les trames émises par
  cet hôte, c'est-à-dire les commandes du daemon, vues par la boucle locale de socketcan
  (`MSG_DONTROUTE`). Aussi les trames d'erreur, le delta de `tx_dropped` et `txqueuelen` (lus dans
  `/sys/class/net`). Par drive : la période de statut, en horodatage noyau (`SO_TIMESTAMP` = 29),
  et le délai depuis la dernière commande. Un verdict en sort : `periodic 200 Hz`,
  `replies to commands` (le cas de `right.cam`) ou `silent`.
- **`--bench`** : le coût par trame d'une vidange toutes les 10 ms, avec une socket séparée par
  méthode : python-can `recv(timeout=0)`, brute/exception, brute/`poll`, brute/`select`, et
  `canio.RawCanBus` lui-même.
- **`--threads`** : le CPU par thread du daemon et ses changements de contexte (`/proc/<pid>/task`),
  sur la même fenêtre que l'écoute. Le pid vient de systemd.
- Épinglée par défaut sur les cœurs 2–3. `--json` garde les chiffres pour comparer avant et après.
- Un test vérifie que le fichier ne contient aucun appel `send`.

### Étape 1 — socket CAN brute

- `RawCanBus(channel)` : socket `CAN_RAW` non bloquante.
  - `recv(timeout)` renvoie un `RawFrame` à `__slots__` (`arbitration_id`, `is_extended_id`,
    `data`) ou `None`.
  - Il expose aussi `send_ext(arb_id, data)`, `send(msg)`, `shutdown()` et `channel`.
- `_send(bus, arb_id, data)` : un `RawCanBus` emballe la trame lui-même, sans `can.Message`.
  `can.Bus` et `MockBus` reçoivent un objet message comme avant.
- **Octets émis identiques** à `build_can_frame` de python-can, `len8_dlc` compris. C'est vérifié
  pour `set_current`, `set_pos` (y compris la saturation int32) et `force_control`.
- `ENOBUFS`, `EAGAIN`, `ENETDOWN` et l'écriture courte passent par le **même** `except` qu'avant :
  `send_errors()`, `send_stats()`, le crochet du daemon, le recul (backoff) et `_can_watch`.
- **Interrupteur** :
  - `canio.CANIO_BACKEND = "raw"` est le défaut du daemon.
  - `server.py --can-backend python-can` permet le retour arrière.
  - `open_buses()` garde `backend="python-can"` par défaut, donc `run_policy.py`,
    `thermal_calibrate.py` et `tools/*` sont inchangés.
  - Le repli d'arrêt d'urgence de `server.py` suit le backend du daemon.
  - Le backend est noté dans l'événement black box `daemon.preflight` et dans la bannière de
    démarrage.

### Étape 3 — IMU à 120 Hz

- `sensehat.IMU_HZ = 120.0` est la **seule source de vérité** : `--imu-hz` prend cette valeur par
  défaut, et l'unité systemd ne passe pas l'option. Un test le vérifie.
- **Audit** : `CAPTURE_S`, `SEQ_SETTLE_S`, `NOISE_MAX_S`, le `beta` de Madgwick (gain par seconde,
  multiplié par `dt`) et `imunoise` (fréquence mesurée) étaient déjà en secondes. **Une seule
  hypothèse liée à 200 Hz** : le `sleep` de la boucle IMU était plafonné à 5 ms. Sans effet à
  200 Hz, il réveillait le thread **deux fois par échantillon** à 120 Hz (mesuré en test : 236
  réveils pour 120 échantillons), et chaque réveil reprend le GIL. Le plafond est maintenant d'une
  période.
- `ODR_HZ` reste à 225 : chaque lecture renvoie un échantillon neuf, âgé d'au plus 4,4 ms.
  `DLPF_CFG` reste à 3, comme le plan le demande ; un commentaire explique le risque de repliement
  (coupure à 51 Hz, Nyquist à 60 Hz).

## 2. Tests

Commande du plan, avec le Python système (sans python-can) :
`python3 -m pytest -q controller/fixed_gait/webui/tests controller/deploy/tests`

| état | passés | échecs | ignorés |
|---|---|---|---|
| après l'étape 1 | 539 | 7 | 17 |
| après l'étape 3 (HEAD) | **547** | **7** | 17 |

Les 7 échecs sont exactement ceux du §3.2 : `test_blackbox::test_rotation_and_budget_enforcement`,
4 de `test_policy_run` et 2 de `test_golden`. Les deux tests instables n'ont pas échoué.

Nouveaux tests :
- `test_can_probe.py` (5) ;
- `test_canio_raw.py` (23). 7 d'entre eux ont besoin de python-can et sont ignorés sans lui. Ils
  ont été lancés **dans un venv avec python-can 4.6.1** (la version du Pi) : 23/23 passés ;
- `test_imu_rate.py` (5) ;
- la fixture de `test_mountcal.py`, maintenant à 120 et à 200 Hz.

## 3. Écarts au plan

1. **Branche** : sur ta consigne, tout est sur `policy-worker`, pas sur une nouvelle branche
   `can-raw-imu120`.
2. **Lecture à vide** : mesurée sur le Pi (§6). L'exception y coûte 55–58 µs, `poll(0)` 5 µs.
   **`poll` est le défaut** (`18d5048`).
3. **Trames d'erreur et RTR** : python-can recevait les trames d'erreur, que `_drain` jetait comme
   non étendues. `RawCanBus` ne s'y abonne pas (filtre noyau par défaut) et saute les trames RTR.
   La sonde, elle, compte les erreurs.
4. **Repli sans `AF_CAN`** : sur une machine sans socket CAN, le backend brut bascule en mock, comme
   le faisait déjà l'absence de python-can. Une interface autre que socketcan bascule sur
   python-can.
5. Au-delà du plan : la correction du `sleep` de la boucle IMU, décrite ci-dessus.

## 4. Procédure pour l'opérateur

Le robot doit être suspendu ou couché à chaque redémarrage du service : un redémarrage relâche les
moteurs (0 A).

> **Fait le 20/11 (§6)** : la mesure de référence (4.1), `txqueuelen` (4.2) et le bruit IMU de
> référence à 200 Hz. Il reste 4.3, 4.4 et 4.5, qui demandent de redémarrer le daemon ou de faire
> bouger les jambes. HEAD est maintenant `18d5048`.

### 4.1 Mesure de référence (étape 0), avant tout changement du daemon

```bash
cd ~/Desktop/EPFL/Projets/Dash-01/code/DASH-01
./controller/deploy_to_pi.sh --check --to c5bccc3
./controller/deploy_to_pi.sh --to c5bccc3          # NE PAS redémarrer : le daemon actuel reste
ssh nemo@192.168.4.1
cd ~/running_robot/robot
../.venv/bin/python tools/can_probe.py --seconds 10 --bench --threads --ip --json ~/probe_0_before.json
```

`--to c5bccc3` déploie aussi `1478170`, la correction de l'aperçu v3 dans `server.py`, qui ne prend
effet qu'au redémarrage. Relever, robot LIMP : les trames par tick, `tx_dropped`, le verdict de
`right.cam`, le coût python-can contre la socket brute, et le CPU de la boucle CAN (~32 %) et de
l'IMU (~13 %).

### 4.2 Étape 2 — file d'émission (root)

```bash
sudo ip link set can0 txqueuelen 100
sudo ip link set can1 txqueuelen 100
sudoedit /etc/systemd/system/can-up.service   # ajouter les deux ExecStart=... txqueuelen 100 (README webui)
sudo systemctl daemon-reload
```

Vérifier : `ip -s -d link show can0`. **Correction (§6)** : `tx_dropped` augmente toujours avec
une file de 100, et ce ne sont pas des commandes perdues. Ce critère ne s'applique donc pas.

### 4.3 Étape 4 — `right.cam` (outil CubeMars)

Comparer le drive 105 de `can0` avec celui de `can1` (`left.cam`), puis activer l'émission
périodique du statut à 200 Hz. Vérifier avec `can_probe.py` : `right.cam` doit afficher
`periodic 200 Hz`, une période p50 de 5,00 ms et un délai après commande réparti sur 0–10 ms.

### 4.4 Déployer les étapes 1 et 3

```bash
./controller/deploy_to_pi.sh --check
./controller/deploy_to_pi.sh --restart       # HEAD = 18d5048
journalctl -u runningrobot-webui -n 30       # bannière : "socketcan can0/can1, raw backend"
```

**Retour arrière du CAN seul** : ajouter `--can-backend python-can` à l'`ExecStart` de l'unité
installée, puis lancer `daemon-reload` et redémarrer le service. **Retour arrière complet** :
`./controller/deploy_to_pi.sh --rollback`. **IMU seule** : ajouter `--imu-hz 200` à l'`ExecStart`.

### 4.5 Étape 5 — mesures et validation

1. Robot LIMP : relancer la commande de 4.1 avec `--json ~/probe_1_after.json`. Attendu : la boucle
   CAN passe de ~32 % à ~15–20 %, l'IMU de ~13 % à ~8 %, et la ligne `canio.RawCanBus` du bench
   tombe vers ~8 µs par trame.
2. Panneau de bruit IMU : un enregistrement immobile de 60 s à 120 Hz, à comparer au dernier à
   200 Hz. Ne passer à `DLPF_CFG = 4` que si le bruit augmente nettement.
3. Black box : la colonne `dt` en LIMP et en MANUAL. Visé : p50 10,0 ms, p90 ≤ 10,5 ms.
4. Run debout de 3 s comme les runs 3–4, une fois avec `"worker": false` et une fois sans. Lire
   `policy.done` : `realised_hz`, puis `timing_ms` (`measure`, `ctrl`, `gov`, `ws`, `send`, `log`,
   `total`). **Succès si : ≥ 99 Hz, `total` p90 < 10 ms, aucun arrêt pour le rythme.**
5. Envoyer les deux JSON de la sonde et le `policy.done` pour l'analyse.

## 5. Questions ouvertes

- **Lecture à vide** : laquelle est la moins chère sur le Pi, exception ou `select` ? Réponse dans
  `--bench`. Si c'est `select`, passer `empty_check="select"` dans `open_buses`, en une ligne.
- **Repliement IMU** : le bruit change-t-il à 120 Hz avec `DLPF_CFG = 3` ? Réponse avec le panneau
  de bruit.
- **Si le critère n'est pas atteint**, l'étape suivante est le chronométrage par poste : governor
  (`gov`, H3), ramasse-miettes (H6), black box (H4), flux UI (H5).
- **Le worker** : une fois le CAN allégé, le worker apporte-t-il enfin quelque chose ? La
  comparaison avec `"worker": false` de 4.5.4 répond. `POLICY_WORKER_VERSIONS` n'a pas été modifié.
- **Hors périmètre, à décider plus tard** : passer les outils hors daemon sur la socket brute.
  `open_buses(..., backend="raw")` suffit pour ceux qui passent par `canio`.

## 6. Tests sur le robot (2026-11-20, avec ton accord)

**Contrainte : aucune commande à un moteur, aucune jambe ne bouge.** Robot en **LIMP** tout du long.
Le daemon déjà en marche (`1478170`, pid 22515) a continué son flux normal à courant nul, que je
n'ai pas touché. Je n'ai rien émis sur `can0` ni sur `can1`, je n'ai pas redémarré de service et je
n'ai pas changé de mode.
- Le code a été copié dans **`~/plan_c_scratch`**, pas dans `~/running_robot`.
- pytest a été installé dans `~/plan_c_scratch/pylib`, pas dans le venv du robot.
- Les seuls envois de trames ont eu lieu sur une interface **virtuelle** `vcanplanc`, créée puis
  supprimée, sans aucun matériel derrière.

Résultats bruts dans `SimToReal/Logs/plan_C/`.

### 6.1 Mesure de référence (`probe_0_before.json`, `probe_1_txq100.json`)

| | mesuré |
|---|---|
| trafic | can0 500 trames/s reçues, can1 600, soit 11 trames par tick ; 299/s émises par bus |
| `right.cam` | **répond aux commandes** : période 10,1 ms (max 34,5), délai 0,12–0,66 ms. Les 5 autres : 5,00 ms |
| CPU au repos | boucle CAN **32–33 %** (345–463 préemptions/s), IMU **13,2 %**, Flask 7,5 %, black box 2,7 % |
| python-can, vidange d'un bus | 90 µs par trame, **0,84–1,21 ms** par tick et par bus |
| socket brute (`poll`) | 14 µs par trame, 5 µs à vide, **0,15–0,19 ms** par tick et par bus |
| socket brute (exception) | 10 µs par trame, **55–58 µs à vide** |
| `canio.RawCanBus` (`poll`) | 20 µs par trame, 8 µs à vide, 0,23–0,25 ms par tick et par bus |

### 6.2 Étape 2 : `txqueuelen 100`, fait

- Appliqué à chaud sur `can0` et `can1`, sans perturbation : les interfaces sont restées UP et
  ERROR-ACTIVE.
- Ajouté aux deux `ExecStart` de `/etc/systemd/system/can-up.service`, suivi d'un `daemon-reload`.
  Sauvegarde : `~/plan_c_scratch/can-up.service.bak`.
- **Le `tx_dropped` de `can0` n'a pas changé** : 3,0/s avec 100 contre 2,2/s avec 10. Sur 10 s, le
  cœur CAN du noyau a accepté 6 012 trames (`/proc/net/can/stats`) et les interfaces en ont émis
  6 013. Le daemon n'a vu aucune erreur d'envoi (`can_errors` vide), et ses commandes ont leur
  écho à 298,8/s sur chaque bus, autant sur `can0` que sur `can1`. **Aucune commande n'est
  perdue.** Ce compteur vient du pilote `mcp251xfd` de `can0`, sans trame d'un programme qui y
  corresponde. Une seule socket CAN par bus, celle du daemon. Le pilote a une fois signalé
  `error-warn` 1 sur `can0`. Cause encore ouverte ; elle est peut-être liée à `right.cam`.

### 6.3 `RawCanBus` sur un vrai noyau, interface virtuelle `vcanplanc` (`vcan_check.py`)

- **Octets identiques** à python-can 4.6.1 pour `set_current` (0 et −3,217 A), `set_pos`
  (−123,4567°, et la saturation int32) et `force_control`. Une trame de statut se décode à
  l'identique avec les deux backends.
- **Tick du daemon** (vidange de 12 trames, puis 6 envois), cœur 2 :

| backend | vidange p50 | 6 envois p50 | tick p50 / p90 |
|---|---|---|---|
| python-can | 1,43 ms | 0,68 ms | **2,11 / 2,31 ms** |
| RawCanBus, `poll` | 0,48 ms | 0,21 ms | **0,69 / 0,79 ms** |
| RawCanBus, exception | 0,47 ms | 0,21 ms | 0,68 / 0,74 ms |
| RawCanBus, `select` | 0,58 ms | 0,21 ms | 0,79 / 0,84 ms |

  Gain attendu : **~1,4 ms par tick** sous le GIL, avant toute contention. Avec la contention, le
  gain réel sera plus grand, puisque les coûts p90 de python-can sont 2 à 3 fois plus élevés.

### 6.4 Tests unitaires sur le Pi

`test_canio_raw`, `test_can_probe`, `test_imu_rate` et `test_mountcal` (avec l'IMU **mock**, qui ne
touche pas le bus I2C), lancés avec le Python du robot (3.13.5, python-can 4.6.1) sur les cœurs 2–3
en `nice` : **57/57 passés**.

### 6.5 Bruit IMU de référence à 200 Hz (`noise_200hz.json`, `.npz`)

60 s, robot immobile (`still: True`), 199,96 Hz, `DLPF_CFG` 3 :
- accéléromètre : **2,05 / 2,13 / 2,02 mg RMS** ;
- gyroscope : **0,087 / 0,084 / 0,077 °/s RMS** ;
- tangage : 0,055° RMS ; roulis : 0,050° RMS ;
- gigue 0,74 ms, plus grand trou 12,5 ms.

C'est la référence à comparer après le passage à 120 Hz.

### 6.6 Non fait, parce que c'est contraire à la consigne

- **Déployer et redémarrer le daemon** (4.4). Le service émet un courant nul aux 6 moteurs à son
  arrêt, pendant son pré-vol et en LIMP. C'est ce qu'il fait déjà en ce moment, mais un redémarrage
  serait **mon** action, et la première émission par le nouveau code. Même chose pour le CPU
  « après », la colonne `dt` en LIMP et le bruit IMU à 120 Hz. Lancer une deuxième IMU à côté du
  daemon pour l'éviter reconfigurerait la puce et fausserait les lectures du daemon.
- **`right.cam`** (4.3) : l'outil CubeMars écrit dans la configuration du drive.
- **Run debout** (4.5.4) : les jambes bougent.

Pour la suite : `./controller/deploy_to_pi.sh --restart`, avec le robot suspendu ou couché, puis
relancer `can_probe.py` et l'enregistreur de bruit. Le reste de la procédure du §4 s'applique tel
quel.

## 7. Déploiement et mesures « après » (2026-11-20, avec ton accord, robot en LIMP)

- `deploy_to_pi.sh --force --restart --yes` : `1478170` → `18d5048`, 9 fichiers. Sauvegarde :
  `~/deploy_backups/20261120_110735_1478170_to_18d5048`.
  - `--force` était nécessaire : sur le Pi, `IMU.md`, `tools/README.md`, `sensehat.py` et
    `mountcal.py` avaient d'anciennes **docstrings et docs**, sans aucune différence de code.
  - Bannière au démarrage : `socketcan can0/can1, raw backend` et `IMU thread at 120 Hz`. Mode LIMP,
    les 6 moteurs vivants à 0 A, aucune erreur CAN, aucune erreur dans le journal.
- Il n'y a eu ni changement de mode ni mouvement. **MANUAL et le run debout n'ont pas été faits**
  (les moteurs y tiennent ou bougent).

### 7.1 CPU par thread au repos (`probe_2_after.json`)

| thread | avant | après | attendu (plan) |
|---|---|---|---|
| boucle CAN | 32–33 % (539–590 attentes, 345–463 préemptions/s) | **17,1 %** (267 attentes, 360 préemptions/s) | 15–20 % |
| IMU | 13,2 % | **9,8 %** | ~8 % |
| Flask | 7,5 % | 7,9 % | |
| black box | 2,7 % | 2,7 % | |

### 7.2 Rythme de la boucle en LIMP (black box Tier A, un tick sur 5)

| | avant (2 segments, ~1 h) | après (~4 min) |
|---|---|---|
| `dt` p50 | 10,00 ms | 10,00 ms |
| `dt` p90 | 10,31 / 10,55 ms | **10,04 ms** |
| `dt` p99 | 11,61 / 11,81 ms | **10,82 ms** |
| > 12 ms | 0,45 / 0,79 % | **0,29 %** |
| max | 40,8 / 34,5 ms | **15,0 ms** |
| `slip` (compteur du daemon) | 1,2 % | 0,2 % |

L'objectif LIMP (p50 10,0 ms, p90 ≤ 10,5 ms) est atteint. MANUAL n'a pas été mesuré.

### 7.3 Bruit IMU, 60 s, robot immobile, `DLPF_CFG` 3

| | 200 Hz | 120 Hz |
|---|---|---|
| accéléromètre RMS (mg) | 2,05 / 2,13 / 2,02 | 1,99 / 2,13 / 2,10 |
| gyroscope RMS (°/s) | 0,087 / 0,084 / 0,077 | 0,087 / 0,084 / 0,076 |
| tangage / roulis RMS (°) | 0,055 / 0,050 | 0,062 / 0,064 |
| gigue / plus grand trou | 0,74 / 12,5 ms | 1,09 / 15,2 ms |

**Pas de repliement mesurable** : on garde `DLPF_CFG = 3`. L'attitude filtrée est 15 à 25 % plus
bruitée, avec moins de mises à jour de Madgwick par seconde. Cela reste à 0,06°.

### 7.4 ⚠ Âge de l'IMU vu par la boucle de contrôle (colonne `imu.age` de la black box)

| | 200 Hz (avant) | 120 Hz (après) |
|---|---|---|
| p50 | **1,38 ms** | **6,21 ms** |
| p90 | 3,72 ms | 7,88 ms |
| p99 | 5,57 ms | 10,27 ms |
| max | 15,9 ms | 19,0 ms |

L'hypothèse du plan, « un âge de 0 à 8,3 ms », oubliait deux choses. L'horodatage de `fast()` est
pris **avant** la lecture I2C (~1,7 ms) et le filtre, si bien qu'un échantillon a déjà ~2 ms quand il
est publié. Et à 200 Hz, la boucle lisait en moyenne un échantillon bien plus frais que ne le prévoit
le calcul. Le passage à 120 Hz ajoute donc **~5 ms de retard capteur médian**, alors que la
simulation **n'en modélise aucun** (§1.2 du plan). En échange, il fait gagner ~3,4 points de CPU
sous le GIL.

**Décision à prendre (la tienne)** :
1. revenir à 200 Hz (`IMU_HZ = 200`, ou `--imu-hz 200` dans l'unité) et garder le seul gain CAN,
   qui est le gros morceau (33 → 17 %) ;
2. garder 120 Hz en assumant ~6 ms de retard capteur, ou ajouter ce retard à la randomisation de
   la simulation et réentraîner ;
3. caler la lecture de l'IMU sur le tick de contrôle (un échantillon lu juste avant chaque tick) :
   les deux avantages, au prix d'une modification de `sensehat.py` et du daemon. Hors périmètre du
   plan C.

À mon avis, il faut l'option 1 avant le prochain run debout, puis l'option 3 si le CPU de l'IMU
compte encore.

### 7.5 Reste à faire

- `right.cam` dans l'outil CubeMars (toi), puis `can_probe.py` pour vérifier `periodic 200 Hz`.
- Le choix de 7.4.
- Le run debout de 3 s, avec et sans worker, et le critère ≥ 99 Hz / `total` p90 < 10 ms. C'est
  la seule mesure qui dira si le plan C suffit sous la charge de la politique.
- `dt` en MANUAL.

## 8. Retour à l'IMU à 200 Hz (décision de l'opérateur, 2026-11-20)

- `cfd33c9` = `git revert fe9d3d6`, plus un commentaire au-dessus de `IMU_HZ` qui garde les
  chiffres de l'essai à 120 Hz. Le code d'avant revient : `IMU_HZ = 200`, le plafond de `sleep` à
  5 ms (un seul réveil par échantillon à 200 Hz), la fixture de `test_mountcal` à 200 Hz.
  `test_imu_rate.py` est retiré. Le backend CAN brut reste.
- Tests webui : 396 passés. 6 échecs : les 5 connus, plus `test_pose_anchor` (instable, connu),
  qui passe seul (13/13).
- Déployé : `18d5048` → `cfd33c9`, avec redémarrage, robot en LIMP. Sauvegarde :
  `~/deploy_backups/20261120_112011_18d5048_to_cfd33c9`. Bannière : `raw backend`,
  `IMU thread at 200 Hz`. LIMP, 6 moteurs vivants à 0 A, aucune erreur CAN.

| au repos, LIMP | avant le plan C | CAN brut + IMU 120 Hz | **CAN brut + IMU 200 Hz (actuel)** |
|---|---|---|---|
| CPU boucle CAN | 32–33 % | 17,1 % | **17,0 %** |
| CPU IMU | 13,2 % | 9,8 % | **13,4 %** |
| `dt` p90 / p99 | 10,31–10,55 / 11,6–11,8 ms | 10,04 / 10,82 ms | **10,11 / 11,45 ms** |
| `dt` > 12 ms | 0,45–0,79 % | 0,29 % | **0,59 %** (2 min seulement) |
| âge IMU p50 / p99 | 1,38 / 5,57 ms | 6,21 / 10,27 ms | **3,29 / 8,04 ms** |

**Ce qui précise 7.4.** À 200 Hz contre 100 Hz, le rapport est exactement 2 : les deux boucles,
cadencées chacune sur l'horloge, restent **calées sur un déphasage fixé au démarrage**. L'âge moyen
dépend donc du redémarrage. On a mesuré 1,4 ms dans une session et 3,3 ms dans celle-ci, et la
plage possible va d'environ 1,7 ms (la lecture I2C) à 6,7 ms. C'est un retard capteur non modélisé,
qui change d'un redémarrage à l'autre. Lire l'IMU juste avant chaque tick de contrôle (option 3 de
7.4) le rendrait petit et constant. Ce sera la prochaine amélioration à envisager, avant ou en même
temps que d'ajouter un retard capteur à la simulation.
