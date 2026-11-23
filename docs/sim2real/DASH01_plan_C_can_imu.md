# DASH-01 — Plan C : CAN en socket brute, IMU à 120 Hz

Version 1, 2026-11-20. Document de passation pour un agent qui part de zéro.
La partie 1 donne le contexte et les mesures ; la partie 2, les décisions et les étapes ; la
partie 3, les règles, les tests et les livrables.
Historique et raisonnement complets : `SimToReal/DASH01_journal_2026-11-20.md`.

---

# PARTIE 1 — Contexte

## 1.1 Le problème

Sur le Raspberry Pi **3B** (4 × Cortex-A53, 1,2 GHz, aarch64), la boucle de contrôle du daemon
doit tenir **100 Hz**, soit un tick de 10 ms. La politique d'équilibre `standing04_xshape_best`
(bundle v3, 100 Hz) n'a tenu que **60 Hz (run 3) et 76 Hz (run 4)** le 17/11. La garde de rythme
(`POLICY_MIN_RATE_FRAC` = 90 %) a arrêté les deux runs. Les sondes à l'armement indiquaient
pourtant 5,47 et 4,91 ms, sous le budget de 7 ms : le coût réel d'un tick est très supérieur à ce
qu'elles mesurent.

Le plan B (inférence dans un processus séparé, branche `policy-worker`) est implémenté et déployé.
**Sur le Pi, il n'apporte pas de gain** : bench à 87,6 contre 89,1 Hz, ~1,5 ms d'aller-retour par
tick, 2 à 10 % de réponses en retard. Il reste actif pour les bundles v3, par décision de
l'opérateur. **Ne pas modifier `POLICY_WORKER_VERSIONS`.**

## 1.2 Mesures (2026-11-19/20, robot LIMP, lecture seule)

**CPU par thread du daemon au repos** (`/proc/<pid>/task`, 10 s) :

| thread | CPU | attentes volontaires/s | préemptions/s |
|---|---|---|---|
| boucle CAN (100 Hz) | **32–36 %** (~3,6 ms par tick, sans politique) | 459 | **626** |
| IMU Sense HAT (200 Hz) | **12,6–14 %** (~0,6 ms par échantillon) | 769 | 109 |
| Flask / flux vers uiproc | 7,5 % | 163 | 49 |
| black box (écriture) | 2,6–4 % | 97 | 24 |

Au repos, l'interpréteur demande déjà ~64 % d'un cœur sous un seul GIL. Avec la politique
(~3,5 ms) et le governor (~1,9 ms), on dépasse 100 %. L'IMU et la boucle CAN se disputent le GIL :
~570 attentes supplémentaires par seconde côté IMU, 3–4 par tick côté boucle. Les cœurs 0–1
(`CPUAffinity=0 1`) sont saturés : 626 préemptions par seconde.

**Coût d'une lecture CAN de trame en attente** (socket séparée, cœurs 2–3 libres, lecture passive) :

| méthode | par trame p50 / p90 | par bus et par tick |
|---|---|---|
| **python-can** `bus.recv(timeout=0)` (ce que fait `daemon._drain`) | **88 / 245 µs** (130 / 318 µs dans une autre mesure) | **~0,8 ms** |
| socket brute `socket.AF_CAN` + `struct.unpack` | **8 / 39 µs** | ~0,1 ms |
| lecture brute à vide (exception `BlockingIOError`) | 55 µs | (une par bus et par tick) |

**Trafic :** ~800 trames/s reçues sur `can0`, ~900 sur `can1`, soit **~12 trames de statut par tick**
au total, plus ~300 trames/s émises par bus. Contrôleurs **MCP2518FD** (`mcp251xfd`) : 0 erreur,
0 trame manquée. **`can0` perd ~3 trames par seconde en émission** (5 546 au total, `txqueuelen 10`).

**`time.sleep` à 100 Hz** sur les cœurs 0–1 : dépassement p50 0,10 ms, p99 0,76 ms, max 1,84 ms.
Les interruptions SPI sont réparties sur les 4 cœurs. Ce n'est **pas** la cause.

**Run 4 (black box)** : le tick de **248 ms** a lieu **pendant l'armement** (travail du thread HTTP
qui monopolise le GIL), pas pendant le RUN. Pendant le RUN : p50 10,9 ms, p90 22,5 ms, environ
1 tick sur 3 au-dessus de 12 ms, sans périodicité. La boucle est en dépassement permanent.

**`right.cam` (drive 0x69 sur `can0`)** : il **répond à chaque commande** (0,25 à 0,66 ms après)
au lieu d'émettre son statut périodiquement. Il tourne donc à ~100 Hz avec une période irrégulière
(9,1–10,7 ms), alors que les autres drives émettent toutes les 5,00 ms pile. Conséquences : sa
position a ~1 tick de retard, et sa télémétrie s'arrête quand la boucle cale. C'est un réglage du
drive, pas du code.

**Simulation (`BalanceRL`)** : retard **actionneur** randomisé de 6–18 ms ; **aucun retard capteur
modélisé**. Toute latence capteur ajoutée sort donc de l'entraînement.

## 1.3 Le dépôt

- Mac : `/Users/tk/Desktop/EPFL/Projets/Dash-01/code/DASH-01`, branche **`policy-worker`**,
  HEAD `1478170`. Le Pi est sur `5d28969` (`1478170`, la correction de l'aperçu v3 dans `server.py`,
  n'est **pas** déployé).
- `controller/X` sur le Mac correspond à `~/running_robot/robot/X` sur le Pi (`nemo@192.168.4.1`,
  sans internet). Déploiement : `./controller/deploy_to_pi.sh` (`--check` d'abord). C'est
  **l'opérateur** qui déploie.
- Fichiers concernés :
  - `controller/fixed_gait/webui/canio.py` : `open_buses` (python-can `can.Bus`, ou `MockBus`),
    `_send` (avec `install_send_error_hook`), `set_pos`, `set_current`, `force_control`,
    `parse_status`, `MockBus`.
  - `controller/fixed_gait/webui/daemon.py` : `_drain` (l.~1965, `bus.recv(timeout=0.0)`,
    `msg.is_extended_id`, `msg.arbitration_id`, `msg.data`) et `_release_and_close`
    (`b.shutdown()`).
  - `controller/fixed_gait/play_trajectory.py` : `Motor`, `drain`. Vérifier qui d'autre lit le bus.
  - `controller/fixed_gait/webui/sensehat.py` : `IMU_HZ = 200.0`, boucle IMU l.~860
    (le `dt` mesuré est utilisé), `ODR_HZ = 225`, `DLPF_CFG = 3` (~51 Hz).
  - `controller/fixed_gait/webui/server.py` : option `--imu-hz` (défaut `sensehat.IMU_HZ`),
    `sys.setswitchinterval(0.0005)`.
  - `controller/fixed_gait/webui/systemd/runningrobot-webui.service` : `ExecStart` (sur le Pi,
    l'unité installée pointe vers `robot/...`).
- Graphe : depuis `Dash-01/`, lancer `graphify query "<question>"` avant d'explorer, et
  `graphify update code` après avoir modifié du code (`CLAUDE.md` du projet).

---

# PARTIE 2 — Plan

## 2.0 Décisions (prises par l'opérateur, ne pas les rediscuter)

1. **CAN reste à 200 Hz** (fréquence de statut des drives). On accélère la **lecture et l'émission**
   en remplaçant python-can par une **socket `AF_CAN` brute** dans le chemin du daemon.
2. **IMU à 120 Hz.** La période de 8,3 ms est plus courte que le tick de 10 ms : chaque tick reçoit
   au moins un nouvel échantillon, d'un âge de 0 à 8,3 ms, sans jamais reprendre l'échantillon
   précédent quand les deux boucles ne sont pas synchronisées.
3. **`right.cam` à 200 Hz en émission périodique**, réglé par l'opérateur dans l'outil CubeMars.
4. `POLICY_WORKER_VERSIONS` **inchangé**.
5. Le statut CAN à 100 Hz est **écarté** : la vitesse est calculée par différence de positions sur
   10 ms, et le battement avec des drives non synchronisés donnerait des ticks à 0 ou 2 trames, donc
   des vitesses à 0 ou doublées.

## 2.1 Étapes

### Étape 0 — Mesure de référence (avant tout changement de code)

Script passif à ajouter, par exemple `controller/tools/can_probe.py`. Il reprend les sondes de la
phase 1 (débit par interface, mélange de trames, coût python-can contre socket brute, période et
délai par drive) et **n'émet jamais**. L'opérateur peut le relancer avant et après chaque étape.
Relever aussi le CPU par thread du daemon au repos (recette dans le journal, §4).
**Commit séparé.**

### Étape 1 — Socket CAN brute dans `canio`

- Nouveau backend `RawCanBus` : `socket(AF_CAN, SOCK_RAW, CAN_RAW)`, `bind((channel,))`,
  non bloquante. Format de trame `struct "=IB3x8s"` (`can_id` avec `CAN_EFF_FLAG`, dlc, data).
- Il expose ce dont le daemon a besoin, avec la même forme que l'objet python-can : `recv(timeout=0)`
  qui renvoie un objet avec `arbitration_id`, `is_extended_id`, `data` (ou `None`), `send(msg)`,
  `shutdown()`. Un objet léger à `__slots__`, sans `can.Message` dans la boucle chaude.
- La lecture à vide coûte ~55 µs par exception. Mesurer l'alternative (`select.select([s], [], [], 0)`
  ou `recv` avec `MSG_DONTWAIT`) et garder la moins chère.
- Envoi : `sock.send(struct.pack(...))`. `ENOBUFS` (file pleine) et les autres `OSError` doivent
  passer par le **même** chemin d'erreur que `_send` aujourd'hui (`install_send_error_hook`,
  `send_errors()`, `send_stats()`) : les pertes doivent rester visibles.
- **Interrupteur** : `CANIO_BACKEND = "raw"` (défaut) ou `"python-can"`, ou une option `--can-backend`
  dans `server.py`, pour revenir en arrière sans toucher au reste. `MockBus` inchangé.
- Les outils hors daemon (`run_policy.py`, `tools/*`, `thermal_calibrate.py`…) gardent python-can
  pour cette fois.
- **Tests** (MockBus, sans matériel) :
  1. décodage identique : une trame `can.Message` et la même trame encodée en octets bruts donnent le
     même `parse_status` ;
  2. encodage identique : les octets de `set_current`, `set_pos` et `force_control` sont les mêmes
     avec les deux backends (capturer sur une paire de sockets `vcan` si disponible, sinon comparer
     les octets produits) ;
  3. la file pleine (`ENOBUFS` simulé) remonte dans `send_errors()` ;
  4. `CANIO_BACKEND = "python-can"` fonctionne toujours ;
  5. toutes les suites existantes restent vertes hormis les échecs connus (§3.2).
- **Commit séparé.**

### Étape 2 — File d'émission CAN (l'opérateur, en root sur le Pi)

`ip link set can0 txqueuelen 100` et `ip link set can1 txqueuelen 100`, à ajouter dans
`/etc/systemd/system/can-up.service`. Mettre aussi à jour l'exemple d'unité dans `webui/README.md`.
Objectif : `tx_dropped` reste à 0.

### Étape 3 — IMU à 120 Hz

- **Audit d'abord** : chercher dans `sensehat.py` (et `imunoise.py`, `mountcal`, la calibration du
  gyro) tout ce qui suppose 200 Hz : constantes en échantillons au lieu de secondes, gains de filtre
  par échantillon, fenêtres de capture, seuils d'immobilité. Les convertir en secondes où c'est
  nécessaire.
- Passer `IMU_HZ` à 120, ou `--imu-hz 120` dans l'unité systemd (choisir une seule source de vérité
  et la documenter).
- **Repliement** : `DLPF_CFG = 3` coupe à ~51 Hz, et la fréquence de Nyquist est de 60 Hz à 120 Hz.
  Garder `DLPF_CFG = 3` et mesurer le bruit avec le panneau de bruit IMU existant, avant et après.
  Ne passer à `DLPF_CFG = 4` (~24 Hz, plus de retard) que si le bruit augmente nettement, et noter
  le retard ajouté.
- Le seuil `POLICY_IMU_STALE_S` (0,2 s) n'est pas affecté.
- **Tests** : le rythme réel de l'IMU en mock à 120 Hz, les calibrations qui fonctionnent toujours,
  et un test qui fixe la valeur choisie.
- **Commit séparé.**

### Étape 4 — `right.cam` (l'opérateur)

Dans l'outil CubeMars : comparer la configuration du drive 105 de `can0` (`right.cam`) avec celle
de `left.cam` (105 sur `can1`) et activer l'émission périodique du statut à 200 Hz. Vérifier
ensuite avec la sonde de l'étape 0 : période de 5,00 ms, délai depuis la commande aléatoire.

### Étape 5 — Mesures et validation (l'opérateur sur le Pi, l'agent pour l'analyse)

1. Robot LIMP : sonde de l'étape 0 et CPU par thread. Attendu : boucle CAN de ~32 % vers ~15–20 %,
   IMU de 12,6 % vers ~8 %.
2. Black box : colonne `dt` en LIMP et MANUAL (p50 10,0 ms, p90 ≤ 10,5 ms visés).
3. Run debout de 3 s comme les runs 3–4, une fois avec `"worker": false` et une fois sans. Lire
   `policy.done` : `realised_hz`, `timing_ms` (`measure`, `ctrl`, `gov`, `ws`, `send`, `log`,
   `total`).
   **Critère de succès : ≥ 99 Hz tenus, `total` p90 < 10 ms, aucun arrêt pour le rythme.**
4. Si ça ne suffit pas, s'attaquer à ce qui reste d'après les postes : governor (`gov`), ramasse-
   miettes (`gc.callbacks`), black box, flux UI. Ce sont les hypothèses H3 à H6 du journal.

---

# PARTIE 3 — Règles, tests, livrables

## 3.1 Règles

- L'agent ne déploie pas, ne redémarre aucun service, n'envoie **aucune trame CAN** et n'appelle pas
  l'API du robot (`/api/...`). Les sondes sur le Pi ne sont permises qu'en **écoute passive**, et
  seulement si l'opérateur le demande. Les commandes root (`txqueuelen`, systemd) et l'outil CubeMars
  relèvent de l'opérateur.
- Ne pas modifier `controller/deploy/` (lois de commande, `safety.py`, `bundle.py`, `jointmap.py`,
  `deploy_map.json`) ni les données de calibration. Ne pas affaiblir de garde.
- Travailler sur une nouvelle branche **`can-raw-imu120`** partant de `policy-worker`. Un commit par
  étape, messages en anglais au style du dépôt, terminés par
  `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`. Ne pas pousser.
- Tests depuis `code/DASH-01` :
  `python3 -m pytest -q controller/fixed_gait/webui/tests controller/deploy/tests` (~4 min).

## 3.2 Échecs de tests déjà connus (ne pas les « corriger »)

`test_policy_run.py` : privileged observation, whole fraction, every nth tick, trained box (4) ;
`deploy/tests/test_golden.py` (2) ; `test_blackbox.py::test_rotation_and_budget_enforcement`,
**rouge même lancé seul, déjà à `1d8acb7`** ; instables sous charge :
`test_pose_anchor.py::test_a_restart_still_refuses_when_an_origin_actually_moved`,
`test_policy_run.py::test_going_limp_ends_the_run_without_throwing_the_log_away`.

## 3.3 Livrables

- Les commits des étapes 0, 1 et 3, avec les tests verts hormis §3.2.
- `SimToReal/DASH01_plan_C_RAPPORT.md` : ce qui est fait, les hashes, les tests, les écarts au plan,
  la procédure de déploiement et de mesure pour l'opérateur (étapes 2, 4 et 5), les questions ouvertes.

## 3.4 Hors périmètre

Les outils hors daemon via la socket brute ; la réécriture scalaire du governor ; la vitesse
calculée sur l'intervalle réel entre trames (ou l'ERPM du driver) ; les changements de matériel
(Pi 4 : 2 à 3 fois plus rapide estimé en Python mono-thread, non mesuré) ; la fin de run « en
mouvement ».
