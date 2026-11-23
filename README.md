# dash-sim2real

Le code pour entraîner une policy pour le robot DASH-01 et la faire tourner dessus : la stack robot, le training, le modèle du robot actuel et les journaux sim2real.

## 1. Ce qu'est DASH-01

- Un bipède de 15,14 kg, avec 6 degrés de liberté actionnés : 4× AKE90-8 et 2× AK60-39 en CAN (can0 = jambe droite, can1 = jambe gauche).
- Il est piloté par un Raspberry Pi 3B. La boucle de contrôle tourne à 100 Hz, et l'IMU est une Sense HAT (B).
- Les policies sont entraînées dans MuJoCo avec un PPO maison (JAX), sur SCITAS.
- Le détail du matériel est dans [`controller/README.md`](controller/README.md) et [`controller/IMU.md`](controller/IMU.md).

## 2. État actuel (23/11/2026)

- **Sur le robot.** Le Pi exécute `controller/` à `cfd33c9` de l'ancien repo (CAN brut, IMU à 200 Hz), c'est-à-dire le premier commit de ce repo. La policy déployée est `controller/deploy/bundles/standing04_xshape_best.npz`, entraînée sur `BalanceRL/model/stand_xshape.xml`.
- **Validé sur le robot** : la chaîne de déploiement et la sécurité. Au run 9, lancé depuis le terminal, la boucle tient 100,0 Hz (`total` p90 8,9 ms).
- **Pas validé : la tenue debout.** La policy laisse partir le robot vers l'avant (+0,1° → +14,1° en 0,88 s). Au repos, le robot réel est à −6,5°, alors que le robot simulé tient seul. Voir [`docs/sim2real/DASH01_journal_2026-11-20.md`](docs/sim2real/DASH01_journal_2026-11-20.md), §3.10–3.11.
- **Simulation seulement** : toutes les séries RL ([`docs/rl/EXPERIMENTS.md`](docs/rl/EXPERIMENTS.md)).

## 3. Plan du repo

Phase 1 de la migration : les dossiers gardent les noms de l'ancien repo, pour que les chemins dans le code restent valides.

| Dossier | Contenu |
| --- | --- |
| `controller/` | stack robot : daemon et webui, CAN, IMU, policy worker, `policy_cli`, `deploy_to_pi.sh`. Correspond à `~/running_robot/robot/` sur le Pi |
| `controller/deploy/bundles/` | policies exportées (`.npz`), dont celle qui est déployée |
| `dash-01CAD/` | modèle du robot actuel : export homing v8 (twin du webui), meshes, `dash01.xml` |
| `BalanceRL/` | training de la tenue debout (STAND-03/04/05) et les trois plants `stand_*.xml` |
| `RLframework/` | training de la locomotion (STOP, GAIT, SPEED, WALK, CLEARANCE), sans `results/` |
| `checkpoints/` | `best.msgpack` de STAND-03/04, parents pour un warm-start |
| `docs/sim2real/` | journaux, plans B et C, bruit moteurs, analyses des runs 3 à 9 |
| `docs/rl/` | `EXPERIMENTS.md` (catalogue des séries) et `PROJECT_CONTEXT.md` |
| `docs/experiments/` | records d'expériences (fichiers < 1 Mo ; le reste est dans `dash-reports`) |
| `archive/` | copies figées, rien d'actif |

Les vidéos, rapports et logs bruts sont dans `dash-reports`. Les thèses et les présentations ne sont pas publiées. La CAO viendra dans `dash-cad`.

## 4. Quickstart en simulation (Mac)

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r controller/requirements-webui.txt -r RLframework/requirements.txt pytest

# tests hors ligne : aucun moteur, bus CAN simulé
python -m pytest controller/fixed_gait/webui/tests controller/deploy/tests
python -m pytest BalanceRL/tests RLframework/tests
python RLframework/smoke_test.py

# visualiser le plant de la policy déployée
python -m mujoco.viewer --mjcf BalanceRL/model/stand_xshape.xml
```

## 5. Déployer sur le robot

La procédure complète est dans [`controller/README.md`](controller/README.md), section « Deploying to the robot's Pi ». Seul du code commité est déployé.

```bash
./controller/deploy_to_pi.sh --check   # montre ce qui changerait sur le Pi, n'écrit rien
./controller/deploy_to_pi.sh           # demande confirmation avant de copier, puis avant de redémarrer
```

Le robot doit être en LIMP. Après chaque déploiement : une ligne dans [`CHANGELOG.md`](CHANGELOG.md), et un tag `robot-AAAA-MM-JJ-<n>` sur le commit. Passer la checklist « Avant de merger dans main » (section 8).

## 6. Lancer une policy sans le webui

Avec `controller/tools/policy_cli.py`, sur le Pi. Toutes les sécurités restent dans le daemon, et un run dure 10 s au maximum. La procédure détaillée est dans [`docs/sim2real/DASH01_journal_2026-11-20.md`](docs/sim2real/DASH01_journal_2026-11-20.md), §6.

**Avant de commencer** : le robot est dans le harnais, l'E-STOP physique à portée de main, et l'onglet web fermé sur tous les appareils.

```bash
ssh nemo@192.168.4.1 && cd ~/running_robot/robot
python3 tools/policy_cli.py status
python3 tools/policy_cli.py pose standing04_xshape_best.npz       # MANUAL + pose de départ
python3 tools/policy_cli.py run  standing04_xshape_best.npz --allow-uncalibrated-thermal
python3 tools/policy_cli.py limp                                  # relâche tous les moteurs
```

- **Ctrl+C une fois** : arrêt doux. **Deux fois** : arrêt dur.
- **E-STOP depuis un deuxième terminal** : `curl -X POST http://192.168.4.1:8081/api/estop`.

## 7. Entraîner une nouvelle policy

1. Travailler sur une branche `train/<sujet>`. Le training se lance sur SCITAS (Izar), voir `BalanceRL/README.md` (« Workflow ») et `BalanceRL/slurm/`.
2. Exporter avec la vérification de parité :

   ```bash
   python BalanceRL/export.py --run BalanceRL/runs/<run> --out controller/deploy/bundles/<nom>.npz
   ```

3. Vérifier le bundle (`controller/deploy/verify_export.py`, `BalanceRL/verify_bundle.py`), puis faire une répétition sur bus simulé :

   ```bash
   python controller/deploy/run_policy.py --bundle controller/deploy/bundles/<nom>.npz --mock --max-seconds 5
   ```

4. Déployer depuis la branche (section 5), passer les niveaux 2 et 3 de la checklist, puis merger dans `main` (le code et le bundle, jamais les `runs/` ni les `results/`).

Ajouter une fiche dans [`docs/rl/EXPERIMENTS.md`](docs/rl/EXPERIMENTS.md).

## 8. Contribuer

- **Branches.** `main` doit pouvoir tourner sur le robot à tout moment. Les autres branches sont `train/<sujet>` pour le training, `hw/<sujet>` pour la mécanique, la calibration, le CAN et l'IMU (avec un test banc avant merge), et `exp/<sujet>` pour les essais jetables (jamais mergés tels quels).
- **Messages de commit.** Format `zone: ce qui change`, par exemple `canio: RawCanBus drains with poll(0)`.
- **Avant de merger dans `main`.** Un changement limité à `BalanceRL/`, `RLframework/` ou `docs/` s'arrête au niveau 1. Tout ce qui touche `controller/` va jusqu'au niveau 3.
  - *Niveau 1, hors ligne* : les tests ci-dessus sont verts, sans nouvel échec. `verify_export` passe sur chaque bundle. Le rejeu du run 9 (`docs/sim2real/Logs/DASH01_runs6-9_2026-11-20/replay_policy_offline.py`) reste à 0,17° près.
  - *Niveau 2, Pi, robot en LIMP* : `deploy_to_pi.sh --check` propre. Les deux services sont `active` et les 6 moteurs vus vivants. `can_probe.py` est OK, et `policy_cli limp` relâche tout.
  - *Niveau 3, robot dans le harnais, opérateur présent, E-STOP en main* : la pose de départ est atteinte. La boucle tient au moins 99 Hz, avec `total` p90 ≤ 10 ms. La garde de rythme, le fall-kill et l'E-STOP sont testés une fois par session. Le run est noté dans `CHANGELOG.md` et le commit est tagué.

## 9. Historique et crédits

- **Nathann Morand** et **Max Aebi** : robot V1 et thèses ; `RLframework/` et `BalanceRL/` à l'origine.
- **Léo-Paul** (GitHub `leorizkallah`) : séries RL, de STOP à STAND-05, et la policy déployée.
- **Tim** : rebuild V2, stack robot et sim2real.

La chronologie est dans [`HISTORY.md`](HISTORY.md). L'historique git complet d'avant la migration est dans `DASH-01-archive`.
