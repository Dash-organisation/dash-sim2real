# DASH-01 — Plan B : l'inférence de la politique dans son propre processus

Version 2 — 2026-11-17 au soir. Document de passation pour un agent qui travaille **seul, de nuit**.
Partie 1 = contexte et décisions ; partie 2 = plan d'implémentation pas à pas ; partie 3 = règles,
tests, livrables.

---

# PARTIE 1 — Contexte

## 1.1 Le robot et le dépôt

- **DASH-01** : bipède 6 DDL (par jambe : abduction `abd`, `cam`, `thigh`), moteurs CubeMars
  AK60-39 / AKE90-8 sur CAN, IMU Sense HAT (B). Un Raspberry Pi **3B** (4 cœurs Cortex-A53, 1.2 GHz)
  fait tourner un **daemon** Python qui possède le bus CAN, et une interface web.
- **Dépôt (Mac)** : `/Users/tk/Desktop/EPFL/Projets/Dash-01/code/DASH-01`, branche **`homing-v7`**,
  HEAD au moment de la rédaction : `1d8acb7`. Tout le code robot est sous `controller/` :
  - `controller/fixed_gait/webui/daemon.py` — le daemon (`RobotDaemon`), la boucle de contrôle,
    le mode POLICY (`_policy_arm`, `_tick_policy`, …). ~5 000 lignes.
  - `controller/fixed_gait/webui/server.py` — Flask + démarrage du daemon (port 8081).
  - `controller/fixed_gait/webui/uiproc.py` — l'interface opérateur dans **son propre processus**
    (port 8080), qui relaie vers 8081. Bon exemple de « deuxième interpréteur » déjà en production.
  - `controller/deploy/` — la loi de commande, pure numpy, sans dépendance au daemon :
    `bundle.py` (chargement des `.npz`), `controller_balance.py` (`PolicyControllerBalance`, bundles
    v3), `controller_v2.py` (`PolicyControllerV2`, v2), `controller.py` (`PolicyController`, v1),
    `policy_net.py`, `safety.py` (`SafetyGovernor`), `jointmap.py`, `run_policy.py` (exécution sans
    interface, hors daemon).
  - Tests : `controller/fixed_gait/webui/tests/` et `controller/deploy/tests/` (pytest, MockBus).
- **Sur le Pi** (`nemo@192.168.4.1`, pas d'internet) : `controller/X` du dépôt = `~/running_robot/robot/X`.
  Service systemd `runningrobot-webui` (le daemon, drop-in `CPUAffinity=0 1`, `Nice=-10`) et
  `runningrobot-ui` (uiproc, cœurs 2–3). Déploiement : `./controller/deploy_to_pi.sh`
  (voir `controller/README.md`). **L'agent de nuit ne déploie pas** (§3.1).
- `CLAUDE.md` du projet : un graphe `graphify` couvre `code/` ; lancer
  `graphify query "<question>"` avant d'explorer, et `graphify update code` (depuis
  `/Users/tk/Desktop/EPFL/Projets/Dash-01`) après avoir modifié du code.

## 1.2 La politique testée

`standing04_xshape_best.npz` (sur le Mac : `/Users/tk/Desktop/EPFL/Projets/Dash-01/Logs/dash_run1/`,
sur le Pi : `fixed_gait/webui/data/policies/`). Bundle **v3** (`PolicyControllerBalance`), politique
d'équilibre debout, **100 Hz** (`control_dt` 0.01), réseaux : estimateur 438→128→64→3, politique
441→256→256→18. Elle fixe elle-même ses gains kp/kd à chaque tick (force control MIT).

## 1.3 Ce qui s'est passé aujourd'hui (runs debout, robot en harnais)

Quatre essais. Trois bugs d'entrée ont été trouvés et corrigés (déjà commités et déployés) :
IMU retournée de 180° (recalibrée par l'opérateur), gyroscope passé en °/s au lieu de rad/s
(`338d1ca`), couple observé sans le signe de calibration (`fdaaffe`). Plus une garde du workspace
sur la pose mesurée (`40129b2`) et le ralentissement de l'UI pendant un run (« option A », `1d8acb7`).

Depuis, **la politique fonctionne** : partant basculé de −7 à −9° sur les talons, elle redresse le
torse. Mais le **governor arrête chaque run** parce que la boucle n'atteint pas 90 % de 100 Hz :

| Mesure | Valeur |
|---|---|
| `controller.step` seul sur le Pi, cœurs 2–3 libres, pas de throttling | **3.5 ms** p50, p90 4.3 |
| le même, mesuré dans le daemon à l'armement | **9.2 ms** |
| run 3 (sans A) : rythme réel | **60 Hz**, arrêt à 1.69 s |
| run 4 (avec A, flux UI 20 → 2 Hz pendant le run) | **76 Hz**, arrêt à 1.32 s |
| boucle du daemon (colonne `dt` de la black box) en LIMP / MANUAL | 10.0 ms p50, ~11 ms p90 |
| … en POLICY (run 4) | 10.9 ms p50, **22.5 ms p90, 248 ms max** |

Données brutes et analyses : `/Users/tk/Desktop/DASH01_run3_2026-11-17/` et
`/Users/tk/Desktop/DASH01_run4_2026-11-17/` (`ANALYSE.md`, CSV exportés, `raw/`).

## 1.4 Le diagnostic

La boucle de contrôle tourne à **100 Hz** (`TICK_HZ`, `daemon.py` l.71) et la politique s'exécute à
**chaque tick** (`_tick_policy`, l.~4249). Dans un tick de 10 ms : mesure, `ctrl.step` (3.5 ms
seul), `SafetyGovernor.step` (~1.9 ms mesuré en 2026-09), garde workspace, envoi de 6 trames,
log, échantillon black box. Et dans le **même interpréteur Python** (un seul GIL, cœurs 0–1) : le fil
IMU à 200 Hz (`sensehat.py`, filtre d'attitude en Python), l'enregistreur black box, Flask, le flux
vers uiproc. Le GIL ne laisse tourner qu'un fil à la fois : `ctrl.step` attend son tour, d'où 9.2 ms
au lieu de 3.5, et 1 tick sur 10 qui déborde.

**Remède** : sortir `ctrl.step` de l'interpréteur du daemon, dans un processus séparé épinglé sur un
autre cœur. Le daemon ne fait plus qu'envoyer l'observation et attendre la commande, en libérant le
GIL pendant l'attente.

---

# PARTIE 2 — Plan d'implémentation

## 2.0 Décisions (prises — ne pas les rediscuter)

1. **Périmètre** : le worker sert les bundles **v3** (`PolicyControllerBalance`) — c'est ce qui tourne
   sur le robot. v1 et v2 gardent le chemin actuel, en processus. Le choix se fait par une
   constante (`POLICY_WORKER_VERSIONS = (3,)`) et un drapeau par run (`p["worker"]`), pour qu'on
   puisse revenir en arrière sans toucher au code (`POLICY_WORKER_VERSIONS = ()`).
2. **Synchrone, sans latence ajoutée** : dans le tick N, le daemon envoie l'observation N, fait ce
   qui ne dépend pas de la commande, puis **attend la commande N** jusqu'à une échéance
   (`POLICY_WORKER_DEADLINE_MS`, défaut **6.0** ms après l'envoi). L'entraînement a déjà 6–18 ms de
   retard actionneur randomisé ; ajouter un tick (10 ms) entier en sortirait. Le mode « commande
   du tick précédent » n'est **pas** implémenté.
3. **IPC** : `multiprocessing.Pipe(duplex=True)` en `send_bytes` / `recv_bytes`, charge utile =
   `numpy.ndarray.tobytes()` de tableaux float64 de taille fixe (pas de pickle dans la boucle).
   L'attente côté daemon : `conn.poll(timeout)` (libère le GIL). Pas de mémoire partagée en v1 du
   worker : le pipe est plus simple, synchronisé par construction, et détecte la mort du worker
   (EOFError / BrokenPipeError). Coût attendu sur le Pi : ~0.1–0.3 ms aller-retour — à mesurer
   (étape 0/6) ; si > 0.5 ms, mémoire partagée en optimisation ultérieure (hors périmètre).
4. **Démarrage** : `multiprocessing.get_context("spawn")` (jamais `fork` depuis un processus
   multi-fils). Le worker démarre **à l'armement** (dans `_policy_arm`, avant le changement de
   mode), charge le bundle, fait la sonde de timing **chez lui**, répond `READY` avec son `step_ms`.
   Échéance de démarrage 15 s (import numpy sur Pi 3B ≈ 1–3 s). Il s'arrête à la fin du run.
5. **Affinité** : le worker fait `os.sched_setaffinity(0, {2, 3})` s'il le peut (Linux), sinon
   continue (Mac). `OPENBLAS_NUM_THREADS=1` posé **avant** `import numpy` dans le worker.
   Ne pas modifier les services systemd (c'est l'opérateur qui le fera s'il le faut).
6. **La sécurité ne bouge pas** : governor, gardes workspace (`_policy_ws_*`), IMU, rythme
   (`POLICY_MIN_RATE_FRAC`), dead-man, fin de run (`_policy_end`), black box restent dans le
   daemon, inchangés. Le worker ne voit ni le CAN ni la calibration.
7. **Défaillances du worker** (toutes → événement black box `policy.worker` avec la cause) :
   - réponse en retard (échéance dépassée) : le daemon **réutilise la commande précédente** pour ce
     tick, compte `worker_late` ; **3 retards consécutifs → arrêt doux** (raison explicite) ;
     la réponse tardive est lue et **jetée** au tick suivant (vérifier le numéro de séquence) ;
   - worker mort (EOF, pipe cassé, `is_alive()` faux), réponse malformée, séquence incohérente :
     **arrêt dur immédiat** ;
   - le worker attrape toute exception de `ctrl.step`, renvoie un message `ERR` avec le texte, puis
     se termine → arrêt dur côté daemon avec ce texte.

## 2.1 Le protocole

Fichier nouveau : `controller/fixed_gait/webui/policy_worker.py` (côté worker **et** client daemon,
pour qu'un seul fichier décrive le format). Il n'importe rien du daemon (seulement `deploy/`).

Messages daemon → worker (préfixe 1 octet de type, puis charge utile) :

| type | charge utile | effet |
|---|---|---|
| `S` (start) | seq (int64) + obs | `ctrl.start(pos, vel, tau, grav, gyro)` ; répond par l'accusé `A` + seq (aucune commande : le premier pas suit au tick suivant, comme aujourd'hui) |
| `T` (step) | seq (int64) + obs | `cmd = ctrl.step(pos, vel, tau, grav, gyro)` → répond `C` |
| `M` (method) | JSON court `{"name": "set_run"|"set_speed"|"zero_heading"|"start_brake"|"cancel_brake", "args": [...]}` | appelle la méthode du contrôleur (rare, hors boucle chaude : JSON accepté ici) |
| `Q` (quit) | — | sortie propre |

`obs` = float64[24] : `pos[6] vel[6] tau[6] grav[3] gyro[3]` (repère MODÈLE, ordre actionneur — ce que
`_tick_policy` passe déjà à `ctrl.step`).

Messages worker → daemon :

| type | charge utile |
|---|---|
| `R` (ready) | JSON `{"step_ms": p50, "step_p90_ms": …, "pid": …, "affinity": […], "version": 3}` |
| `A` (ack start) | seq |
| `C` (command) | seq (int64) + float64[N] : `target[6] kp[6] kd[6] action[18] phase freq saturated commit run_flag speed_cmd brake_frac vel_est[3] heading_deg step_ms` (N à figer dans une constante, avec un `assert` de taille des deux côtés) |
| `E` (error) | texte UTF-8 |

Côté daemon, `C` est reconstruit en un objet avec **les mêmes attributs** que `CommandBalance`
(`types.SimpleNamespace` ou une petite classe à `__slots__`) pour que le reste de `_tick_policy` et
`_policy_log` ne change pas. Champs utilisés aujourd'hui par le daemon (vérifiés par grep) :
`cmd.target, kp, kd, phase, freq, saturated` et via `getattr` : `commit, run_flag, speed_cmd,
vel_est, brake_frac`. Méthodes appelées sur `p["ctrl"]` : `start, step, set_run, set_speed,
zero_heading, heading_deg` (+ `getattr(p["ctrl"], "braking")`).

## 2.2 Étapes

### Étape 0 — Chronométrage par poste (avant tout changement de comportement)

But : mesurer avant/après. Dans `_tick_policy` (phase `run`), chronométrer avec
`time.perf_counter()` : `measure` (lecture moteurs + `_policy_measure` + `_policy_imu`), `ctrl`
(`ctrl.step`, ou plus tard l'attente du worker), `gov` (`gov.step`), `ws` (gardes workspace),
`send` (`_policy_send`), `log` (`_policy_log`), `total`. Accumuler dans un tableau préalloué du run
(pas d'allocation dans la boucle). À la fin du run : ajouter à l'événement `policy.done` les p50/p90/max
de chaque poste, et `tick_budget` = 1000/ctrl_hz. Exposer les p50/p90 dans le statut du run
(`_policy_pub`, l.~4596) pour le panneau. Tests : un test MockBus vérifie que `policy.done` contient
ces champs et qu'ils sont cohérents (`total ≥ ctrl`). **Commit séparé.**

### Étape 1 — `policy_worker.py` : le processus et son client

- `worker_main(conn, bundle_path, opts)` : pose `OPENBLAS_NUM_THREADS=1`, affinité, importe
  `bundle`, `controller_balance`, charge le bundle, construit le contrôleur, fait la sonde
  (`POLICY_PROBE_TICKS` = 40 pas après 10 d'échauffement, comme `_policy_probe` l.3687), envoie `R`,
  puis boucle `recv_bytes` → traite → répond. Toute exception → `E` puis sortie.
- `class PolicyWorkerClient` (utilisé par le daemon) :
  `start(bundle_path, timeout=15)` → lance le processus (`spawn`, `daemon=True`), attend `R`,
  renvoie le dict de `R` ; `begin(seq, obs)` (envoie `S`, attend `A`) ;
  `post(seq, obs)` (envoie `T`, **n'attend pas**) ; `collect(seq, deadline_s)` → `(cmd | None,
  état)` avec `conn.poll(restant)` ; jette les réponses de séquence plus ancienne ;
  `call(name, *args)` (envoie `M`) ; `stop(timeout=1.0)` (envoie `Q`, `join`, puis `terminate`/`kill`
  si besoin) ; `alive()`.
- Aucun accès disque, réseau, CAN dans le worker.
- **Tests unitaires** (`controller/fixed_gait/webui/tests/test_policy_worker.py`), sans daemon,
  avec un bundle v3 synthétique ou le golden de `controller/deploy/tests/` s'il est v3 (sinon
  `v2_fixture` → regarder s'il existe une fixture v3 ; sinon construire un bundle v3 minimal avec
  `bundle.py` — voir comment `test_policy_run.py` obtient `staged_v3`/`BUNDLE_SRC`) :
  1. **équivalence bit à bit** : 200 pas d'une séquence d'observations aléatoires mais fixe ; les
     commandes du worker (`target, kp, kd, action`) sont **identiques** (`np.array_equal`) à celles
     d'un `PolicyControllerBalance` local nourri des mêmes observations ;
  2. `M set_run/set_speed` appliqués entre deux pas changent la sortie comme en local ;
  3. le worker lève une exception (monkeypatch d'un module de test, ou bundle corrompu) → `E` ;
  4. worker tué (`os.kill`) → `collect` renvoie l'état « mort » sans bloquer au-delà de l'échéance ;
  5. worker lent (option de test `opts["sleep_ms"]`) → `collect` renvoie « en retard », et la
     réponse tardive est jetée au `collect` suivant ;
  6. `stop()` termine toujours le processus (pas de zombie : `exitcode` non `None`).
  **Commit séparé.**

### Étape 2 — Brancher le worker dans le daemon

Dans `daemon.py` :
- **Armement** (`_policy_arm`, l.~3606) : si `b.version in POLICY_WORKER_VERSIONS`, créer le
  client, `start()` ; en cas d'échec → refuser l'armement avec la raison (pas de repli silencieux en
  processus). Remplacer `step_ms = self._policy_probe(ctrl, b)` par le `step_ms` du worker pour ces
  bundles. Le contrôle « slow loop » (l.~3615) compare alors `step_ms` du worker à
  `POLICY_WORKER_DEADLINE_MS` (et non plus au budget en processus) ; garder `allow_slow_loop`.
  Garder un `ctrl` local **non utilisé dans la boucle** seulement si d'autres parties en ont besoin
  (`command_kind`, `v_max`, `cmd_slew`, `has_brake`… lus à l'armement, l.~3570–3585) — sinon
  extraire ces métadonnées du bundle. Stocker le client dans `req["worker"]`.
- **Entrée en run** (`_policy_enter_run`, l.4114) : pour un run worker, `client.begin(seq, obs)` au
  lieu de `p["ctrl"].start(...)` ; v2 `set_speed/set_run` → `client.call(...)` (même si v3 seul est
  servi aujourd'hui, garder la forme générique).
- **Tick** (`_tick_policy`, autour de l.4502 `cmd = p["ctrl"].step(...)`) :
  1. juste après la mesure (`_policy_measure`, `_policy_imu`, `vel`, `tau`) : `client.post(seq, obs)` ;
  2. faire ce qui ne dépend pas de la commande (comptage du rythme, garde de pose mesurée,
     contrôles IMU/âge, etc.) ;
  3. `cmd, st = client.collect(seq, deadline)` ; si `cmd is None` et `st == "late"` → reprendre la
     dernière commande (`p["last_cmd"]`), `p["worker_late"] += 1`, `p["worker_late_run"] += 1`, et
     si `worker_late_run >= POLICY_WORKER_MAX_LATE` (3) → `gov.kill(..., hard=False)` ;
     si `st in ("dead", "error", "bad")` → `gov.kill(..., hard=True)` avec la cause ;
  4. la suite (`gov.step`, gardes, envoi, log) **inchangée**.
  Les méthodes appelées depuis l'UI (`policy_set_run`, `policy_set_speed`, `policy_zero_heading`,
  l.3728–3812) passent par `client.call` pour un run worker. `heading_deg()` : renvoyé dans `C`.
- **Fin de run** (`_policy_end`, l.4152) : `client.stop()` **après** avoir passé les moteurs en
  SET_POS/LIMP (jamais avant), dans un `try/finally`. Événement `policy.worker` avec
  `late_total`, `late_max_run`, aller-retour p50/p90 (mesuré côté daemon : `post` → `collect`).
- **Arrêt du daemon** / E-STOP / changement de mode forcé : s'assurer que le worker est arrêté
  (chercher où `self._pol` est nettoyé et où le mode quitte POLICY).
- `run_policy.py` (exécution sans interface) : **ne pas modifier** dans cette nuit.
**Commit séparé.**

### Étape 3 — Tests daemon (MockBus)

Les tests existants qui arment un bundle v3 (`test_policy_run.py`, `test_policy_ws.py`,
`test_policy_api.py`, fixtures `staged_v3`…) doivent passer **par le worker** sans modification de
leurs assertions. Ajouter dans `tests/test_policy_worker_daemon.py` :
1. un run v3 complet en MockBus via le worker atteint `run` puis `done` proprement ; le worker est
   terminé après ; `policy.done` contient les champs de l'étape 0 et `worker_late_total`;
2. worker ralenti artificiellement (option de test) → gel des commandes puis arrêt doux après
   3 retards ; le robot simulé passe en SET_POS (branche debout) ou LIMP selon le cas existant ;
3. worker tué en plein run → arrêt dur au tick suivant, événement `policy.worker` avec la cause ;
4. armement refusé si le worker ne démarre pas (bundle illisible) ;
5. `POLICY_WORKER_VERSIONS = ()` (monkeypatch) → l'ancien chemin en processus fonctionne toujours
   (non-régression du repli) ;
6. golden : la trace de commandes d'un run MockBus déterministe via le worker = la même via le
   chemin en processus (`np.array_equal` sur `target/kp/kd`).

### Étape 4 — Mesure sur le Mac

Script `controller/tools/bench_policy_worker.py` (hors tests) : boucle 100 Hz simulée avec une
charge parasite configurable en fils Python (pour imiter le fil IMU) ; compare le temps de tick et
le rythme réel **en processus** vs **worker**. Sortie texte. Utile au matin pour l'opérateur ; à
lancer sur le Mac, et documenter la commande pour le Pi (l'opérateur la lancera, §3.1).

### Étape 5 — Documentation

- `controller/fixed_gait/webui/README.md` : section « Policy worker » (pourquoi, comment, comment
  revenir en arrière : `POLICY_WORKER_VERSIONS = ()`).
- Mettre à jour les commentaires de `daemon.py` qui décrivent la sonde et le budget (bloc
  « can this Pi run it? », l.~380–445) : le budget d'un run worker est l'échéance, pas
  10 ms − 3 ms.
- `graphify update code`.

---

# PARTIE 3 — Règles, tests, livrables

## 3.1 Règles absolues (sécurité et périmètre)

- **Ne jamais se connecter au Pi** (`ssh`, `scp`, `rsync`, `deploy_to_pi.sh`), ne jamais appeler
  l'API web du robot (`/api/...`), ne jamais lancer `run_policy.py` sans `--mock`. Aucune commande
  moteur, aucun redémarrage de service. Le déploiement et les essais sont faits par l'opérateur.
- Ne pas modifier : `controller/deploy/controller_balance.py`, `controller_v2.py`, `controller.py`,
  `policy_net.py`, `bundle.py`, `safety.py`, `jointmap.py`, `deploy_map.json`, les fichiers de
  calibration / workspace / données (`fixed_gait/webui/data/`), les services systemd.
- Ne pas affaiblir une garde existante (governor, workspace, IMU, rythme, dead-man). Le worker
  ajoute des arrêts, il n'en retire aucun.
- Travailler sur une **nouvelle branche** `policy-worker` partant de `homing-v7`. Commits petits, un
  par étape, messages en anglais au style du dépôt (corps qui explique le pourquoi), terminés par :
  `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`. **Ne pas pousser** (`git push`) ni
  fusionner dans `homing-v7`.
- Python du Mac : `python3` (`/opt/anaconda3/bin/python3`, numpy présent, **pas de mujoco**).
  Lancer les tests depuis `code/DASH-01` :
  `python3 -m pytest -q controller/fixed_gait/webui/tests controller/deploy/tests`
  (~4 min). Les suites ciblées : `test_policy_run.py test_policy_ws.py test_policy_api.py
  test_policy_imu_units.py test_policy_torque_sign.py test_ui_quiet.py`.

## 3.2 Échecs de tests déjà connus (ne pas les « corriger », ne pas s'en inquiéter)

- Toujours rouges, antérieurs à aujourd'hui (vieux bundles sans `imp_leg_ix`, golden figé) :
  `test_policy_run.py::test_a_privileged_observation_bundle_can_never_be_deployed`,
  `::test_a_bundle_whose_rate_is_not_a_whole_fraction_of_the_loop_is_refused`,
  `::test_a_bundle_slower_than_the_loop_by_a_whole_number_runs_on_every_nth_tick`,
  `::test_a_command_outside_the_trained_box_is_clamped_and_says_so`,
  `controller/deploy/tests/test_golden.py` (2 tests).
- Instables sous la charge de la suite complète, verts seuls :
  `test_blackbox.py::test_rotation_and_budget_enforcement`,
  `test_pose_anchor.py::test_a_restart_still_refuses_when_an_origin_actually_moved`,
  `test_policy_run.py::test_going_limp_ends_the_run_without_throwing_the_log_away`.
- Règle : à la fin, **aucun nouveau test rouge** par rapport à cette liste (relancer seul un test
  instable avant de conclure).

## 3.3 Critères d'acceptation (fin de nuit)

1. Étapes 0 à 3 commitées sur `policy-worker`, suites de tests vertes hormis §3.2.
2. Test d'équivalence bit à bit worker = local : vert.
3. Tests de défaillance (lent, tué, erreur, démarrage raté) : verts, arrêts doux/durs conformes à §2.0.7.
4. Repli `POLICY_WORKER_VERSIONS = ()` : vert.
5. Étape 4 : chiffres du Mac (en processus vs worker, avec charge parasite) dans le rapport.

## 3.4 Livrable : rapport du matin

Écrire `/Users/tk/Desktop/DASH01_plan_B_RAPPORT.md` :
- ce qui est fait, étape par étape, avec les hashes de commit ;
- résultats des tests (commandes et sorties résumées) ;
- chiffres de l'étape 4 ;
- tout écart au plan et pourquoi ;
- **la procédure de validation pour l'opérateur** sur le Pi, dans l'ordre :
  1. `git checkout policy-worker` puis `./controller/deploy_to_pi.sh --check` puis déploiement
     (robot **suspendu et LIMP**, redémarrage du service) ;
  2. commande du benchmark de l'étape 4 à lancer sur le Pi (sans moteurs) ;
  3. un run debout de 3 s comme les runs 3–4 (Pose politique, robot immobile, harnais détendu) ;
  4. ce qu'il faut lire dans `policy.done` (rythme réel, postes de l'étape 0, `worker_late_total`)
     et le critère de succès : **≥ 99 Hz tenus**, `total` p90 < 10 ms, aucun arrêt pour rythme ;
  5. retour arrière : `POLICY_WORKER_VERSIONS = ()` ou `deploy_to_pi.sh --rollback <nom>`.
- questions ouvertes pour l'opérateur.

## 3.5 Hors périmètre (noté pour plus tard, ne pas faire)

- Fin de run « en mouvement » : le maintien SET_POS rigide fait tomber un robot qui bascule déjà
  (run 3). À traiter séparément.
- Balancement à 0.75 Hz en SET_POS (semelles TPU + boucle de position des drivers), robot qui
  démarre basculé sur les talons (−7 à −9°), cuisse droite à 4× le couple de la simulation.
- Garde « robot immobile avant armement ».
- `run_policy.py` via le worker ; mémoire partagée à la place du pipe ; Pi 4/5.
