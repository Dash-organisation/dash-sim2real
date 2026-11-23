# DASH-01 — Plan B : rapport d'implémentation

2026-11-18. Branche **`policy-worker`**, créée depuis `homing-v7` @ `1d8acb7`. **Rien n'est poussé**,
rien n'est fusionné, rien n'est déployé, aucune connexion au Pi.

## 1. Ce qui est fait (étapes et commits)

| étape | commit | contenu |
|---|---|---|
| 0 | `20bcbf5` | Chronométrage par poste de chaque tick de contrôle en RUN (`measure / ctrl / gov / ws / send / log / total / rtt`, puis `wstep`) dans un tampon préalloué. p50/p90/max dans `policy.done` et dans le run terminé, p50/p90 sur les 200 derniers ticks dans le statut live, plus `tick_budget_ms` et `realised_hz`. |
| 1 | `d73752a` | `controller/fixed_gait/webui/policy_worker.py` : le protocole (1 octet de type + seq int64 + float64 de taille fixe, sans pickle), le processus (`spawn`, affinité {2,3}, `OPENBLAS_NUM_THREADS=1`, sonde chez lui, `R`/`A`/`C`/`K`/`E`) et `PolicyWorkerClient` (`start/begin/post/collect/call/stop/alive`). |
| 2 | `5decb44` | Branchement dans `daemon.py` : armement via le worker (refus si le démarrage échoue, sans repli silencieux), `begin` à l'entrée en RUN, post → travail indépendant → collect dans le tick, commandes UI par nom, retards / pannes → arrêts doux / durs, arrêt du worker en fin de run, sortie de POLICY et sortie de la boucle. |
| 3 | `5d5483e`, `dfca68e` | `tests/test_policy_worker_daemon.py` (run complet, worker lent, tué, qui lève une exception, démarrage raté, repli, trace identique au bit près). `dfca68e` corrige une assertion sujette à une course (voir §4). |
| 4 | `475a0cd` | `controller/tools/bench_policy_worker.py` : boucle 100 Hz simulée, en processus vs worker, avec une charge parasite en fils Python. |
| 5 | `52032b6` | Section « Policy worker » dans `webui/README.md`, commentaire du bloc « can this Pi run it? » de `daemon.py`. `graphify update code` lancé. |
| + | `5d28969` | `uiproc.py` : l'appel `/api/policy/arm` attend jusqu'à 25 s au lieu de 4 s (voir §4). |

Retour arrière : `POLICY_WORKER_VERSIONS = ()` dans `daemon.py`, ou `"worker": false` dans une seule
demande d'armement (ajout, voir §4).

## 2. Résultats des tests

Commande : `python3 -m pytest -q controller/fixed_gait/webui/tests controller/deploy/tests`

- **Suite complète : 516 passed, 10 skipped, 7 failed (244 s).** Les 7 échecs sont tous dans la liste
  connue (§3.2 du plan) :
  - `test_policy_run.py` × 4 (privileged observation, whole fraction, every nth tick, trained box) :
    rouges aussi à `1d8acb7`, avant toute modification (vérifié en début de session : 4 failed, 108 passed sur les suites ciblées) ;
  - `deploy/tests/test_golden.py` × 2 ;
  - `test_blackbox.py::test_rotation_and_budget_enforcement` : **rouge aussi lancé seul**, contrairement
    à ce qu'annonce le plan. Vérifié dans un worktree à `1d8acb7` : il y échoue de la même manière
    (`expected the segment to rotate`). Problème préexistant, sans lien avec cette branche.
- **Suites ciblées** (`test_policy_run/ws/api/imu_units/torque_sign`, `test_ui_quiet`,
  + les nouvelles) : les 4 échecs connus seulement. Les tests existants qui arment `bal5_s0` (v3)
  passent maintenant **par le worker**, sans qu'on ait touché à leurs assertions.
- **Nouveaux tests** : `test_policy_timing.py` (4), `test_policy_worker.py` (12),
  `test_policy_worker_daemon.py` (9), tous verts, et verts aussi en répétition pour le test du worker tué.

Critères d'acceptation (§3.3) :

1. étapes 0 à 3 commitées, suites vertes hormis la liste connue : **oui** ;
2. équivalence bit à bit worker = local : **oui**, deux fois. Dans `test_policy_worker.py`, 200 pas
   d'un bundle v3 synthétique à réseau aléatoire : `target`, `kp`, `kd`, `action`, `vel_est` identiques.
   Dans `test_policy_worker_daemon.py`, la trace complète d'un run MockBus réel (`bal5_s0`),
   rejouée dans un contrôleur en processus : `target/kp/kd` identiques à chaque tick ;
3. défaillances : en retard → commande gelée puis arrêt doux après 3 retards (fin LIMP, branche hung) ;
   worker tué → arrêt dur en < 30 ticks, avec la cause ; exception → arrêt dur avec son texte ;
   démarrage raté → armement refusé, rien en file : **oui** ;
4. repli `POLICY_WORKER_VERSIONS = ()` : **oui** ;
5. chiffres du Mac : §3.

## 3. Étape 4 — chiffres du Mac (et ce qu'ils ne disent pas)

Mac M-series, 8 cœurs, Python 3.13, numpy 2.1.3, bundle `bal5_s0.npz` (v3, 100 Hz), 8 s par mode.
Sur ce Mac, `ctrl.step` ne prend que **0,09 ms** (3,5 ms sur le Pi). `--step-repeat 35` ajoute donc
35 pas d'un contrôleur de travail, avec le vrai motif d'appels numpy, pour retrouver environ 3 ms
dans les deux modes. « Reste du tick » : 3 ms de Python. Charge : N fils Python à 200 Hz.

| charge | interv. GIL | mode | Hz | ctrl p50 / p90 / max (ms) | tick p90 / max | glissements |
|---|---|---|---|---|---|---|
| aucune | 0,5 ms | en processus | 100,0 | 3,02 / 3,06 / 3,30 | 6,06 / 6,30 | 0 |
| | | worker | 100,0 | 3,02 / 3,08 / 3,23 | 6,08 / 6,23 | 0 |
| 2 × 1,5 ms | 0,5 ms | en processus | 100,0 | 3,03 / 3,10 / 4,58 | 6,11 / 8,57 | 0 |
| | | worker | 100,0 | 3,02 / 3,10 / 3,42 | 7,10 / 7,62 | 0 |
| 3 × 2,5 ms (GIL saturé) | 0,5 ms | en processus | 99,6 | 3,22 / 3,28 / 6,28 | 8,20 / 11,90 | 48 |
| | | worker | 98,4 | 3,97 / 5,48 / 9,22 | 9,29 / 12,22 | 124 |
| 2 × 1,5 ms | 5 ms | en processus | 99,0 | 3,07 / 3,19 / 9,26 | 6,19 / 12,26 | 270 |
| | | worker | 100,0 | 3,98 / 4,17 / 7,34 | 7,17 / 10,34 | 12 |
| 3 × 2,5 ms | 5 ms | en processus | 88,1 | 3,17 / 3,22 / 18,37 | 6,22 / 21,37 | 403 |
| | | worker | 44,5 | 15,05 / 30,06 / 60,11 | 33,07 / 63,11 | 800 |

**Ce qu'on peut en dire honnêtement :**

- **Avec l'intervalle GIL du daemon (0,5 ms, `server.py`), le Mac ne montre aucun gain net du worker.**
  Avec une charge modérée, les deux modes tiennent 100 Hz. Le worker réduit le pire cas de `ctrl`
  (3,4 contre 4,6 ms) mais ajoute environ 1 ms au p90 du tick. Avec le GIL saturé, il fait
  légèrement *moins bien*.
- **Le Mac ne reproduit pas le problème du Pi.** En processus, `ctrl` reste à environ 3 ms même sous
  charge, alors que le Pi est passé de 3,5 à 9,2 ms. 8 cœurs rapides ne sont pas 4 Cortex-A53 avec
  le daemon épinglé sur 0–1. **La mesure décisive est celle du Pi** (procédure §5), et les postes de
  l'étape 0 dans `policy.done` y répondent directement.
- **Le coût caché du worker, c'est la reprise du GIL.** Chaque appel système (`poll`, les deux `read`
  de `recv_bytes`, le `write`) relâche le GIL. Pour le reprendre, il faut attendre jusqu'à
  l'intervalle de commutation si d'autres fils Python tournent. Avec l'intervalle par défaut de
  CPython (5 ms) sous forte charge, l'attente se compte par multiples de 5 ms (p50 15 ms).
  À 0,5 ms, ce coût reste borné.
- J'ai essayé de réduire le nombre de reprises côté client : un seul `recv()` avec `SO_RCVTIMEO`, ou
  `select()` + un seul `recv()`. **Aucun gain mesuré**, j'ai donc gardé `Connection.poll/recv_bytes`
  (détail dans le commit `475a0cd` et dans `policy_worker.py`).
- Une conséquence à connaître : l'état « en retard » ne regarde que si la réponse est arrivée
  avant l'échéance (`poll`). Si l'attente du GIL qui suit dépasse l'échéance, ce n'est pas compté
  comme un retard, mais ça apparaît dans le poste `ctrl` (et `ctrl − wstep`) de `policy.done`.

## 4. Écarts au plan, et pourquoi

1. **`"worker": false` par run** (en plus de `POLICY_WORKER_VERSIONS`). Ça permet un A/B sur le
   robot sans redéployer.
2. **Message `K`** (réponse à `M`) : le plan ne prévoyait pas de réponse à un appel de méthode.
   Or `set_speed` et `start_brake` renvoient une valeur que le daemon utilise. Un appel sans
   réponse dans le délai (`POLICY_WORKER_CALL_MS` = 20 ms) déclenche un arrêt dur : une réponse
   tardive se retrouverait devant la commande suivante.
3. **Format `C` à 49 valeurs** : celui du plan, plus `braking` (au lieu de `getattr(ctrl, "braking")`).
4. **`begin` attend jusqu'à 50 ms** (`POLICY_WORKER_BEGIN_MS`), pas 6 ms : c'est le moment du
   passage de main sur un robot debout, et un tick glissé y est préférable à un arrêt dur.
   Un échec à cet endroit déclenche un arrêt dur.
5. **Premier pas en retard sans commande précédente** : le daemon garde la dernière trame
   réellement envoyée (celle du passage de main ou de la fin d'approche).
6. **Plafond de la sonde à l'armement** : `min(POLICY_WORKER_DEADLINE_MS, policy_max_step_ms(hz))`.
   Le reste du tick reste séquentiel, et le test existant qui force `policy_max_step_ms` passe
   ainsi sans modification.
7. **Garde de pose mesurée déplacée avant la loi de commande**, pour les deux chemins. Elle
   n'utilise pas la commande, donc le comportement est identique, et elle tourne pendant que le worker calcule.
8. **`p["ctrl"]` vaut `None` pour un run worker.** Le contrôleur local ne sert qu'à lire les
   métadonnées à l'armement, pour que rien ne puisse le faire avancer par erreur.
9. **`__main__` masqué pendant le `spawn`.** Sinon l'enfant ré-exécute `server.py` (Flask, daemon,
   Sense HAT), soit plusieurs secondes d'imports sur le Pi.
10. **`uiproc.py` : 25 s pour `/api/policy/arm`.** L'armement attend maintenant le démarrage du
    worker (numpy : 1 à 3 s sur le Pi 3B). Avec le délai de 4 s du proxy, le panneau aurait affiché
    « control process unreachable » pour un armement qui avait bien eu lieu.
11. **Options réservées aux tests et au bench** dans le worker : `sleep_ms`, `raise_at`, `fail_load`, `pad_us`, `pad_steps`.
12. **Chemin `self._pol_req` jamais consommé** (e-stop verrouillé, activation refusée, ré-armement) :
    le worker déjà lancé est arrêté (`_policy_drop_req`).

## 5. Procédure de validation pour l'opérateur (sur le Pi, dans l'ordre)

1. **Déploiement** — robot **suspendu et LIMP** :
   `git checkout policy-worker` → `./controller/deploy_to_pi.sh --check` → déploiement → redémarrage
   de `runningrobot-webui` **et** de `runningrobot-ui` (`uiproc.py` a changé).
2. **Bench sans moteurs** (il ne touche ni le bus ni le daemon) :
   ```
   cd ~/running_robot/robot && taskset -c 0,1 python3 tools/bench_policy_worker.py \
       --bundle fixed_gait/webui/data/policies/standing04_xshape_best.npz --seconds 10
   ```
   puis la même commande avec `--load-threads 0`, et avec `--load-threads 3 --load-us 2500`.
   Vérifier sur la ligne `worker:` que `affinity` vaut `[2, 3]` et que le `probe step` est d'environ 3,5 ms.
3. **Un run debout de 3 s** comme les runs 3–4 (Pose politique, robot immobile, harnais détendu).
   La réponse d'armement doit indiquer `worker: true`, `worker_pid` et `worker_affinity`.
4. **À lire dans `policy.done`** : `realised_hz`, `timing_ms` (en particulier `total.p90`,
   `ctrl.p50/p90`, `wstep.p50`, `rtt.p90`), `worker_late_total`, `worker_failed`, ainsi que
   l'événement `policy.worker` (`cause: "stopped"`).
   **Critère de succès : ≥ 99 Hz tenus, `total` p90 < 10 ms, aucun arrêt pour le rythme.**
   Si ça échoue, comparer `ctrl` et `wstep` : un `wstep` proche de 3,5 ms avec un `ctrl` élevé
   indique que c'est la reprise du GIL qui coûte, pas le calcul.
   Pour un A/B immédiat, refaire le même run avec `"worker": false`.
5. **Retour arrière** : `POLICY_WORKER_VERSIONS = ()` dans `daemon.py`, ou
   `deploy_to_pi.sh --rollback <nom>`.

## 6. Questions ouvertes pour l'opérateur

- **Les chiffres du Mac ne prouvent pas le gain** (§3). Si le Pi ne tient pas non plus ≥ 99 Hz avec le
  worker, le levier suivant est de réduire le travail Python *dans* l'interpréteur du daemon
  (le filtre IMU à 200 Hz, l'enregistreur), ou de passer à la mémoire partagée avec attente sans
  appel système. Ce dernier point est hors périmètre et n'est pas fait.
- Le worker hérite de `Nice=-10` du service, ce qui est voulu. `CPUAffinity=0 1` est un masque
  d'affinité, pas un cpuset, donc le worker peut se replacer sur 2–3. À confirmer via `worker_affinity`
  dans la réponse d'armement. Si un `AllowedCPUs=` (cpuset) est ajouté un jour, il l'en empêchera.
- Le worker démarre à chaque armement (1 à 3 s sur le Pi). Si c'est gênant, il pourrait être
  pré-lancé au chargement d'un bundle. Non fait.
- `test_blackbox.py::test_rotation_and_budget_enforcement` est rouge même lancé seul, déjà à
  `1d8acb7`. À regarder séparément.
