# DASH-sim2real — plan de migration et cleanup

Nov 22, 2026 · @TK

## Objectif et décisions

On crée un repo neuf, `Dash-organisation/dash-sim2real`, construit à partir de la branche `policy-worker` de Tim. C'est le code qui tourne aujourd'hui sur le Pi. On y ajoute seulement ce qu'il faut pour réentraîner et redéployer une policy. Tout le reste part dans un repo d'archive en lecture seule.

| Décision | Choix | Pourquoi |
| --- | --- | --- |
| Base du code robot | `policy-worker` @ `b25d404` (Pi sur `cfd33c9`) | seule version validée sur le hardware : CAN brut, IMU 200 Hz, boucle à 100 Hz depuis le terminal (run 9) |
| Base du code RL | `BalanceRL/` et `RLframework/` de `dash-01-v2-speed-discovery` @ `3574ddd`, **tout le training du semestre** | c'est là qu'a été entraînée la policy déployée `standing04_xshape_best` |
| Historique | repartir avec un historique neuf (« Initial import »), l'ancien repo est archivé intact | on part au propre ; le contexte est gardé dans `HISTORY.md` et dans l'archive |
| Branches | `main` = toujours déployable sur le robot ; branches de travail `train/…`, `hw/…`, `exp/…` | pas de PR obligatoire, mais une checklist avant chaque merge dans `main` |
| Données lourdes | dans des repos dédiés : dash-reports (rapports, vidéos, données) et dash-cad (plus tard) | objectif : repo < 100 Mo, contre 3,9 Go aujourd'hui |

**Attention, « déployable » ne veut pas dire « tient debout ».** Au run 9 du 20/11, la boucle tient 100 Hz, mais la policy laisse partir le robot vers l'avant (+0,1° → +14,1° en 0,88 s). Au repos, le robot réel est à −6,5°, alors que le robot simulé tient seul. Ce qui est validé : la chaîne de déploiement et la sécurité. Ce qui ne l'est pas : la tenue debout. Source : `SimToReal/DASH01_journal_2026-11-20.md` §3.10–3.11.

Noms : Léo-Paul et Léopold désignent ici le même compte GitHub, `leorizkallah`, auteur des séries STAND et des séries RL. À corriger si ce sont deux personnes.

## Périmètre : quel repo contient quoi

On range par **usage**, pas par auteur. Tout ce qui sert à faire tourner ou à entraîner le robot actuel va dans `dash-sim2real`, quel que soit l'auteur. Les rapports vont dans `dash-reports`, la CAO dans `dash-cad`, et tout ce qui n'est plus utilisé reste dans `DASH-01-archive`.

Pourquoi pas par auteur : le code de training de Léo-Paul repose sur une base que Nathann a écrite en septembre.

- `RLframework/` a été créé par Nathann le 17/09 (`7180ee5`), et `BalanceRL/` le 24/09 (`356c10f`).
- Léo-Paul a ensuite modifié 81 des 101 `.py` de `RLframework/` et 19 des 26 de `BalanceRL/`.
- Les anciens stacks de Nathann (`training/`, `walk_mit/`, `leg2d/`, `walk_v2/`, `rl/`) ont déjà été supprimés de l'arbre le 17/09 (`0031885`). Ils n'existent plus que dans l'historique, donc dans l'archive.

| Repo | Contenu | Taille estimée | Quand |
| --- | --- | --- | --- |
| `dash-sim2real` | robot stack (Tim), `BalanceRL/` + `RLframework/` (Nathann puis Léo-Paul), les trois plants debout, les 6 checkpoints `best.msgpack` de STAND-03/04 (10 Mo, pour repartir en warm-start), les records d'expériences de Léo-Paul (`STATUS.md`, configs, métriques < 1 Mo), journaux sim2real | < 150 Mo | maintenant |
| `dash-reports` | thèses Morand et Aebi (PDF), midterms (pptx), workspace `report/` de Léo-Paul (md, galeries HTML, figures), vidéos et données lourdes | environ 0,9 Go | maintenant |
| `dash-cad` | `dash-01CAD/`, `dash-01CAD-v2/`, `xshape_closed/`, `robotCADdescription/CAD/`, `Dash_2print/`, sources Inventor | à mesurer | dans un deuxième temps |
| `DASH-01-archive` | l'ancien repo complet : historique de Nathann, stacks legacy, `sprint_runner/walk_mit`, toutes les branches | 3,9 Go, lecture seule | à l'étape 1 |
| `.github` | page d'accueil de l'orga (`profile/README.md`) : quel repo sert à quoi, et par où commencer | quelques ko | maintenant |

Où vont les 914 Mo de fichiers touchés par Léo-Paul depuis septembre :

- **`dash-sim2real`** : le code (1,8 Mo), les md/html (0,8 Mo), les JSON (5,5 Mo), les `stand_*.xml` et les checkpoints best (10 Mo).
- **`dash-reports`** : les vidéos (522 Mo), les données (306 Mo, dont les `progress.csv` et `jsonl` de 20 à 27 Mo), les figures (32 Mo) et les slides (9 Mo).
- **`dash-cad`** : les exports CAD (environ 31 Mo, une fois les plants retirés).

## État des lieux (22/11/2026)

Le code utile est réparti à quatre endroits : deux branches git, un dossier hors git et le Pi. La policy déployée n'est même pas dans la branche qui la fait tourner.

| Source | Contenu | Taille | Ce qu'on en garde |
| --- | --- | --- | --- |
| `policy-worker` (Tim, local, jamais pushée) | robot stack : daemon/webui, CAN brut, IMU 200 Hz, policy worker, `policy_cli`, `deploy_to_pi.sh`, homing v7/v8 — 35 commits au-dessus de `main` | 391 Mo, dont 220 Mo d'artefacts | **base du nouveau repo** |
| `homing-v7` (Tim, local) | 16 commits, déjà inclus dans `policy-worker` | — | rien de plus |
| `dash-01-v2-speed-discovery` (Léo, sur GitHub) | `BalanceRL/` (STAND-03/04/05), `RLframework/` (STOP, GAIT, SPEED, WALK, CLEARANCE), `dash-01CAD-v2/`, `report/`, `docs/experiments/` | 1,22 Go | code RL, les trois plants debout, le bundle déployé, et le catalogue des expériences |
| `Dash-01/SimToReal/` (hors git) | journaux 17/11 et 20/11, rapports plans B et C, plan bruit moteurs, `Logs/` des runs 3 à 9 | 89 Mo | les `.md`, les `ANALYSE.md` et `replay_policy_offline.py` ; les black-boxes brutes partent sur le drive |
| `code/deploy_v8_to_pi.sh` (hors git) | déploiement ponctuel du diff `95ea100..9eaa4e1` (twin v8) | 3 ko | **remplacé** par `controller/deploy_to_pi.sh` (commit `e731bc6`), qui est générique ; à archiver, pas à garder actif |
| Pi `nemo@192.168.4.1` | `~/running_robot` = clone de `nathmo/running_robot`, dossier `robot/` = notre `controller/` ; daemon sur `cfd33c9` | — | rien à importer, mais `deploy_to_pi.sh` y lit le SHA déployé (étape 5) |

Pièges repérés :

- **`controller/deploy/bundles/standing04_xshape_best.npz` n'existe que dans la branche de Léo.** `policy_cli.py` et les tests de `policy-worker` y font référence. Il faut l'importer, sinon `main` ne pourra pas lancer la policy actuelle.
- **Il y a 30 modèles MuJoCo `.xml` et 10 copies de meshes.** Le plan de la policy déployée est `BalanceRL/model/stand_xshape.xml`. Le twin du webui utilise `dash01_homing_v8`.
- **`controller/tools/cubemarstool_v3.2.0.exe` est un binaire tiers commité.** On le sort du repo et on met un lien de téléchargement dans le README.
- **Il y a des `__MACOSX/` et un `xshape_closed/` orphelin à la racine de la branche de Léo.**
- **Suite de tests du 20/11 : 516 passés, 7 échecs connus** (journal §3.1). Il faut les lister et les marquer `xfail` pour qu'un échec nouveau se voie.

## Organisation GitHub

L'organisation compte quatre repos et une page d'accueil (détail dans la section Périmètre). Un nouveau venu fait `git clone dash-sim2real` et peut entraîner puis déployer. Il ne clone `dash-reports` que s'il veut les vidéos et les rapports, et `dash-cad` que pour la mécanique.

| Repo | Rôle | Qui pousse | Statut |
| --- | --- | --- | --- |
| `.github` | page d'accueil de l'orga : carte des repos, « par où commencer », contacts | Tim | actif |
| `dash-sim2real` | base vivante : robot stack, training, modèle du robot actuel | tout le monde | actif |
| `dash-reports` | thèses, midterms, workspace `report/` de Léo-Paul, vidéos | tout le monde | actif |
| `dash-cad` | toute la CAO, les exports et les fichiers à imprimer | Tim | dans un deuxième temps |
| `DASH-01-archive` | l'actuel `leorizkallah/DASH-01`, transféré et renommé | personne | *Archived* (lecture seule) |

**Pourquoi pas `dash-rl-x`, `dash-rl-y` ?** Chaque copie embarque son propre daemon, sa propre couche de sécurité et son propre `deploy_to_pi.sh`. La première correction de sécurité, par exemple un bug de fall-kill, devrait être reportée à la main dans N repos. Un projet RL est une **branche** `train/<nom>` du même repo. Il a son dossier dans `training/` et sa fiche dans `docs/rl/EXPERIMENTS.md`. Un nouveau repo ne se justifie que pour un robot physiquement différent. Si Léo veut continuer en privé, il forke `dash-sim2real` et rapporte ses résultats par branche.

### Branches et tags

| Nom | Usage | Règle |
| --- | --- | --- |
| `main` | ce qui peut tourner sur le robot à tout moment | on n'y merge qu'après la checklist « Avant de merger dans main » |
| `train/<sujet>` | entraînement, ex. `train/stand-xshape-v5` | libre ; merge du code (pas des runs) quand une policy est candidate |
| `hw/<sujet>` | mécanique, calibration, CAN, IMU, ex. `hw/right-cam-periodic` | libre ; test banc obligatoire avant merge |
| `exp/<sujet>` | essai jetable | jamais mergée telle quelle |
| tag `robot-AAAA-MM-JJ-<n>` | posé sur chaque commit déployé sur le Pi | posé par `deploy_to_pi.sh` ou à la main juste après |

Réglages GitHub minimaux :

- Protection légère de `main` : pas de force-push, pas de suppression. Les PR ne sont pas obligatoires.
- Un *Project* kanban pour le backlog.
- *Releases* pour les bundles de policies déployés, avec le `.npz`, le `config.json` et l'empreinte (hash) du plant.

Messages de commit : `zone: ce qui change`, dans le style déjà utilisé par Tim, ex. `canio: RawCanBus drains with poll(0)`. Pas de « plein de trucs ».

## Arborescence cible

On garde le nom `controller/` : `deploy_to_pi.sh` le mappe vers `robot/` sur le Pi, et le renommer casserait le seul déploiement validé. Tout le reste se range en quatre zones : robot, modèle, training et docs.

```text
dash-sim2real/
├── README.md            quickstart : installer, tester, déployer, lancer une policy
├── HISTORY.md           chronologie du projet (thèse → V2 → RL → sim2real)
├── CHANGELOG.md         une ligne par déploiement sur le robot (tag robot-*)
├── .gitignore  .gitattributes  LICENSE
├── controller/          = robot/ sur le Pi — copié tel quel depuis policy-worker
│   ├── fixed_gait/webui/    daemon, safety, policy worker, black-box, UI
│   ├── deploy/              control laws, governor, run_policy, bundles/
│   ├── tools/               policy_cli, can_probe, outils AK (sans le .exe)
│   ├── identification/  model/  IMU.md  MIT_PROTOCOL.md
│   └── deploy_to_pi.sh
├── model/               LE robot actuel, une seule copie
│   ├── cad_export/          dash01_homing_v8 (meshes + xml du twin)
│   └── README.md            quel xml = quel usage, masses mesurées vs CAD
├── training/
│   ├── standing/            = BalanceRL/ (Léo), avec stand_xshape.xml
│   └── locomotion/          = RLframework/ (Léo), code + tests, sans results/
├── docs/
│   ├── sim2real/            journaux, plans B/C, bruit moteurs, analyses de runs
│   ├── rl/EXPERIMENTS.md    catalogue STOP/GAIT/SPEED/WALK/CLEARANCE/STAND
│   └── hardware/            pinout CAN, IDs moteurs, procédure de calibration
└── archive/
    ├── models/standing/     stand_v1.xml, stand_v2.xml, stand_xshape.xml + quiet_*.json
    └── scripts/             deploy_v8_to_pi.sh et autres scripts ponctuels
```

| Ancien chemin | Repo cible et chemin | Source |
| --- | --- | --- |
| `controller/**` (sans `robotCADdescription/CAD/`, `results/`, `*.exe`) | sim2real `controller/**` | `policy-worker` |
| `controller/robotCADdescription/MJCF_OPEN_MUJOCO_B/` | sim2real, gardé tant que `controller/model/dash01.xml` y lit ses meshes | `policy-worker` |
| `dash-01CAD/homing/dash01_homing_v8/`, `dash-01CAD/meshes/`, `dash01.xml` | sim2real (le twin et les plants en dépendent) ; une copie aussi dans `dash-cad` | `policy-worker` |
| `BalanceRL/**` | sim2real `BalanceRL/**`, y compris `model/stand_*.xml` et `quiet_*.json` | Léo-Paul @ gel |
| `RLframework/**` sauf `results/` | sim2real `RLframework/**` | Léo-Paul @ gel |
| `report/data/standing0{3,4}/results/*/best.msgpack` (6 × 1,7 Mo) | sim2real `checkpoints/standing0{3,4}/<plant>/best.msgpack` | Léo-Paul @ gel |
| `controller/deploy/bundles/standing04_xshape_best.npz` | sim2real, idem + GitHub Release | Léo-Paul @ gel |
| `docs/experiments/**` : `.md`, `.json`, `config`, fichiers < 1 Mo | sim2real `docs/experiments/**` (chemins inchangés) | Léo-Paul @ gel |
| `docs/experiments/**` : fichiers ≥ 1 Mo (`progress.csv`…) | reports `docs/experiments/**` (copie complète) | Léo-Paul @ gel |
| `PROJECT_CONTEXT.md` | sim2real `docs/rl/PROJECT_CONTEXT.md`, et résumé dans `docs/rl/EXPERIMENTS.md` | Léo-Paul |
| `report/**` (md, html, figures, videos, data, presentations, scripts) | reports `report/**` (chemins inchangés) | Léo-Paul @ gel |
| `RLframework/results/**` | reports `RLframework/results/**` , pour que les liens des galeries fonctionnent | Léo-Paul @ gel |
| Thèses Morand et Aebi (PDF), `Dash-01/Rendu/*.pptx` | reports `theses/` et `presentations/` | project knowledge + Mac de Tim |
| `Dash-01/SimToReal/*.md`, `Logs/*/ANALYSE.md`, `replay_policy_offline.py` | sim2real `docs/sim2real/` | hors git |
| `Dash-01/SimToReal/Logs/**` brut (`.bbseg`, `.bbdump`, CSV) | reports `sim2real-logs/` (brut) | hors git |
| `code/deploy_v8_to_pi.sh` | sim2real `archive/scripts/` | hors git |
| `dash-01CAD-v2/`, `xshape_closed/`, `robotCADdescription/CAD/`, `Dash_2print/`, `Dash-01/CAD/`, `Dash-01/MuJoCo/` | `dash-cad` (deuxième temps) | divers |
| `sprint_runner/` (walk\_mit de Nathann), `dash-01CAD/homing/{v7,SpiderBotInitPos}`, stacks d'avant le 17/09, `__MACOSX/` | `DASH-01-archive` seulement | — |

Dans `dash-reports`, on garde **les mêmes chemins relatifs** que dans l'ancien repo (`report/`, `docs/experiments/`, `RLframework/results/`). Les galeries HTML de Léo-Paul pointent vers ces trois dossiers, et `build_report.py --frozen` vérifie les hashes des fichiers : changer les chemins casserait les deux.

Cette organisation est un point de départ. Si Léo confirme que `dash-01CAD-v2/` (export v36) est le robot actuel plutôt que `dash01_homing_v8`, c'est lui qui va dans `model/cad_export/`.

## Plan d'exécution

La migration tient en environ une journée, en deux phases.

- **Phase 1 : import mécanique.** On ne déplace aucun fichier que du code référence. On garde donc les noms `BalanceRL/`, `RLframework/` et `dash-01CAD/`, et le robot fonctionne dès le premier push.
- **Phase 2 : rangement.** On passe à l'arborescence cible, un renommage par commit, avec `grep` des chemins et les tests après chacun.

Toutes les commandes se lancent sur le Mac de Tim.

```bash
OLD=~/Desktop/EPFL/Projets/Dash-01/code/DASH-01
NEW=~/Desktop/EPFL/Projets/Dash-01/code/dash-sim2real
LEO=3574ddd   # dash-01-v2-speed-discovery, à vérifier après fetch
```

### Étape 0 — Gel et sauvegarde (Léo + Tim, 30 min)

1. Fixer une date de gel avec Léo. Après cette date, plus aucun push sur `leorizkallah/DASH-01`.
2. Léo ajoute Tim en collaborateur (*Settings → Collaborators*).
3. Tim pousse ses deux branches :

   ```bash
   cd $OLD && git fetch origin && git push -u origin policy-worker homing-v7
   ```
4. Lire sur le Pi le commit réellement déployé : `ssh nemo@192.168.4.1 cat ~/deploy_backups/deployed_commit`. Attendu : `cfd33c9`.

**Validé si** `git ls-remote origin` montre `policy-worker` à `b25d404`, et si on connaît le SHA du Pi.

### Étape 1 — Archiver l'ancien repo (Léo, 10 min)

1. *Settings → Transfer* vers `Dash-organisation`.
2. Renommer en `DASH-01-archive`.
3. Après l'étape 7 seulement : *Settings → Archive this repository*.

On ne republie pas 3,9 Go : l'archive *est* l'ancien repo, avec ses issues et ses redirections.

### Étape 2 — Commit A : `controller/` exactement comme sur le Pi

```bash
mkdir $NEW && cd $NEW && git init -b main
git -C $OLD archive cfd33c9 controller | tar -x
rm -rf controller/robotCADdescription/CAD controller/results controller/tools/cubemarstool_v3.2.0.exe
git add -A && git commit -m "Import controller/ from DASH-01@cfd33c9 (version running on the Pi)"
```

Ce commit doit être identique à ce que le Pi exécute. C'est lui qu'on passera en `--from` au premier déploiement, puisque l'ancien SHA n'existe pas dans le nouveau repo.

### Étape 3 — Commit B : mise à jour vers `b25d404` (`policy_cli`)

```bash
rm -rf controller && git -C $OLD archive b25d404 controller | tar -x
rm -rf controller/robotCADdescription/CAD controller/results controller/tools/cubemarstool_v3.2.0.exe
git add -A && git commit -m "controller: update to DASH-01@b25d404 (policy_cli, limp)"
```

On applique les **mêmes exclusions** qu'en A. Sinon, B « supprime » des fichiers et `deploy_to_pi.sh` voudra les effacer sur le Pi.

### Étape 4 — Commits C à E : modèle, RL et policy déployée

```bash
# C. modèle du robot actuel (chemin inchangé en phase 1)
git -C $OLD archive b25d404 dash-01CAD/homing/dash01_homing_v8 dash-01CAD/meshes dash-01CAD/dash01.xml | tar -x
git add -A && git commit -m "model: import dash-01CAD (homing v8 export) from DASH-01@b25d404"

# D. tout le training du semestre (BalanceRL + RLframework), sans results/
git -C $OLD archive $LEO BalanceRL RLframework PROJECT_CONTEXT.md | tar -x
rm -rf RLframework/results && mkdir -p docs/rl && mv PROJECT_CONTEXT.md docs/rl/
git add -A && git commit -m "training: import BalanceRL and RLframework from DASH-01@$LEO"

# D2. records d'expériences, fichiers < 1 Mo seulement (le reste va dans dash-reports)
git -C $OLD archive $LEO docs/experiments | tar -x
find docs/experiments -type f -size +1M -delete
git add -A && git commit -m "docs: import experiment records (<1 MB files) from DASH-01@$LEO"

# D3. checkpoints best de STAND-03/04, pour le warm-start
for s in standing03 standing04; do for p in v1 v2 xshape; do
  mkdir -p checkpoints/$s/$p
  git -C $OLD show $LEO:report/data/$s/results/$p/best.msgpack > checkpoints/$s/$p/best.msgpack
done; done
git add -A && git commit -m "checkpoints: STAND-03/04 best policies (warm-start parents)"

# E. la policy qui tourne sur le robot
git -C $OLD archive $LEO controller/deploy/bundles/standing04_xshape_best.npz | tar -x
git add -A && git commit -m "deploy: add standing04_xshape_best bundle (trained on BalanceRL/model/stand_xshape.xml)"
```

Avant D3, vérifier où le code de STAND-05 attend ses parents (`grep -rn best.msgpack BalanceRL`). Si c'est sous `report/data/…`, on garde ce chemin dans sim2real plutôt que `checkpoints/`, pour ne rien casser.

À vérifier avant D : comparer `BalanceRL/` et `RLframework/` entre `b25d404` et `$LEO` (`git diff --stat b25d404 $LEO -- BalanceRL RLframework`). Si Tim a modifié un fichier que Léo a aussi modifié, on fusionne à la main.

### Étape 5 — Premier déploiement depuis le nouveau repo (robot en LIMP)

```bash
A=$(git rev-list --max-parents=0 HEAD)          # commit A
./controller/deploy_to_pi.sh --check --from $A   # ne doit lister que le diff A→HEAD
./controller/deploy_to_pi.sh --from $A
git tag robot-$(date +%F)-1 && git push --tags
```

Le script lit déjà `--from` (ligne 45). Il compare les fichiers du Pi à ceux de `--from` et s'arrête au moindre écart. Ensuite, il écrit le nouveau SHA dans `deployed_commit`, et les déploiements suivants n'ont plus besoin de `--from`.

**Validé si** `--check` ne signale aucun fichier du Pi différent de A, et si le smoke test sur robot passe (section « Avant de merger dans main »).

### Étape 6 — Docs, archive, fichiers de repo (commits F à H)

- **F.** `docs/sim2real/` ← `Dash-01/SimToReal/*.md`, `old_journal_plan/*.md`, `Logs/*/ANALYSE.md`, `Logs/DASH01_runs6-9_2026-11-20/replay_policy_offline.py`. Les `.bbseg`, `.bbdump` et CSV bruts vont dans dash-reports/sim2real-logs/ (étape 8), avec un lien dans `docs/sim2real/README.md`.
- **G.** `archive/models/standing/` et `archive/scripts/deploy_v8_to_pi.sh` (section Archive).
- **H.** `README.md`, `HISTORY.md`, `CHANGELOG.md`, `.gitignore`, `.gitattributes` (section suivante), et `docs/rl/EXPERIMENTS.md`, d'abord en squelette puis rempli avec Léo.

### Étape 7 — Publication

```bash
du -sh .git                                   # cible < 100 Mo
git remote add origin https://github.com/Dash-organisation/dash-sim2real.git
git push -u origin main
```

Ensuite : archiver `DASH-01-archive`. Le travail en cours de Léo repart en `train/<sujet>` sur le nouveau repo, ou sur son fork privé.

### Étape 8 — `dash-reports` (Tim + Léo-Paul, 1 h)

On fait un repo git simple, **sans LFS**. L'orga est sur le plan gratuit : Git LFS y inclut 10 GiB de stockage et 10 GiB de téléchargement par mois ([GitHub Docs](https://docs.github.com/en/billing/concepts/product-billing/git-lfs)). Avec un budget à 0 $, un dépassement bloque LFS jusqu'au mois suivant. Un repo d'environ 0,9 Go en LFS consommerait donc le quota en une dizaine de clones. En git simple, il n'y a pas de quota de téléchargement. Aucun fichier ne dépasse 27 Mo, donc on reste loin de la limite de 100 Mo par fichier. On passera à LFS si le repo dépasse 2 à 3 Go.

```bash
REP=~/Desktop/EPFL/Projets/Dash-01/code/dash-reports
mkdir $REP && cd $REP && git init -b main
# Workspace de Léo-Paul, avec les mêmes chemins relatifs que l'ancien repo
git -C $OLD archive $LEO report docs/experiments RLframework/results | tar -x
find . -name __MACOSX -prune -exec rm -rf {} +
git add -A && git commit -m "report: import Léo-Paul's report workspace from DASH-01@$LEO"
# Thèses et présentations
mkdir -p theses presentations
cp ~/…/Master_Thesis_Nathann_Morand_Dash011.pdf ~/…/Masters_Thesis_Max_A__Aebi.pdf theses/
cp ~/Desktop/EPFL/Projets/Dash-01/Rendu/*.pptx presentations/
git add -A && git commit -m "docs: add Morand and Aebi theses, midterm decks"
# Logs bruts des runs sim2real
mkdir -p sim2real-logs && cp -R ~/Desktop/EPFL/Projets/Dash-01/SimToReal/Logs/* sim2real-logs/
git add -A && git commit -m "logs: raw black-box data of runs 3 to 9 (Nov 2026)"
git remote add origin https://github.com/Dash-organisation/dash-reports.git && git push -u origin main
```

**Validé si** `python report/scripts/build_report.py --frozen` se lance sans erreur, et si `report/standing04-videos.html` ouvre ses vidéos dans le navigateur.

Un `README.md` à la racine donne, pour chaque série (STOP, GAIT, CLEARANCE, STAND), le lien vers sa galerie et vers sa fiche dans `dash-sim2real/docs/rl/EXPERIMENTS.md`.

### Étape 9 — Page d'accueil de l'orga (`.github`, 20 min)

Créer le repo public `Dash-organisation/.github` avec un fichier `profile/README.md`. GitHub l'affiche sur la page de l'orga. Contenu, une demi-page :

1. DASH-01 en deux phrases, avec une photo.
2. Les repos et leur rôle (le tableau de la section Organisation).
3. « Tu arrives sur le projet ? » : cloner `dash-sim2real`, lire son README, puis `docs/rl/EXPERIMENTS.md` et `HISTORY.md`.
4. L'état actuel, daté, et les contacts (Tim pour le robot, Léo-Paul pour le training).

### Phase 2 — Rangement (une branche `exp/layout`, un commit par déplacement)

1. `BalanceRL/` → `training/standing/`, puis `RLframework/` → `training/locomotion/`. Mettre à jour les `sys.path`, les imports et les `slurm/*.sbatch`, en partant de `grep -rn "BalanceRL\|RLframework"`.
2. `dash-01CAD/` → `model/cad_export/`, et une seule copie des meshes, référencée par le twin, `controller/model/` et les plants RL.
3. Supprimer `controller/robotCADdescription/MJCF_OPEN_MUJOCO_B/` quand plus rien n'y lit de mesh.
4. Chaque déplacement : tests verts et `deploy_to_pi.sh --check` propre avant de merger dans `main`.

## Reprise du training de Léo-Paul sur `dash-sim2real`

Après la migration, Léo-Paul continue son training sur le nouveau repo, dans une branche `train/<sujet>`. Il récupère le code robot de `main` par un simple `git pull`. Un seul piège : le nouveau repo a un historique neuf. Faire `git pull` depuis son clone actuel de `DASH-01` ne marchera pas, git répond `refusing to merge unrelated histories`. Il repart donc d'un clone neuf. Son code n'est pas perdu : il est importé à l'étape 4.

### Ce que ça impose à la migration

- **On importe sa branche au dernier moment.** À la date de gel, il pousse tout sur `dash-01-v2-speed-discovery`, et `LEO` = la tête de cette branche à ce moment-là, plus forcément `3574ddd`. Ainsi, il n'y a rien à reporter à la main.
- **Ses chemins ne bougent pas en phase 1.** `BalanceRL/`, `RLframework/` et `dash-01CAD/` gardent leur nom, donc ses scripts `slurm/`, ses imports et ses presets de `config.py` marchent tels quels.
- **La phase 2 (renommage en `training/…`) se fait avec lui**, à un moment où aucun job SCITAS n'est en cours. Ses branches `train/` font ensuite un `git merge main` pour suivre.

### Procédure pour Léo-Paul, le jour J

```bash
# 1. Avant le gel : tout pousser sur l'ancien repo
cd ~/chemin/vers/DASH-01 && git push origin dash-01-v2-speed-discovery

# 2. Après la migration : clone neuf, puis sa branche de travail
git clone https://github.com/Dash-organisation/dash-sim2real.git
cd dash-sim2real
git switch -c train/stand05-pose-return     # nom à choisir

# 3. Vérifier que son code est bien là, identique
git diff --no-index ~/chemin/vers/DASH-01/BalanceRL BalanceRL      # doit être vide

# 4. Ramener ses fichiers non versionnés (results/, checkpoints locaux) : ignorés par .gitignore, ils restent hors git
rsync -a ~/chemin/vers/DASH-01/RLframework/results/ RLframework/results/

# 5. Travailler, pousser sa branche
git push -u origin train/stand05-pose-return
```

S'il a commité après le gel, malgré la consigne, on rapatrie ces commits sans mélanger les historiques :

```bash
cd ~/chemin/vers/DASH-01
git format-patch <SHA_du_gel>..HEAD -o /tmp/leo-patches -- BalanceRL RLframework
cd ~/chemin/vers/dash-sim2real && git am /tmp/leo-patches/*.patch
```

### Sur SCITAS

1. Cloner `dash-sim2real` à côté de l'ancien checkout.
2. Adapter dans ses `*.sbatch` le chemin du repo. Ses dossiers d'expériences existants (`/home/rizkalla/running_robot_experiments/<snapshot>/`) ne bougent pas.
3. Ne pas couper un job en cours : il finit sur l'ancien checkout, et seuls les nouveaux jobs partent du nouveau clone.

### Quand une policy entraînée doit aller sur le robot

1. Léo-Paul exporte le bundle dans `controller/deploy/bundles/` sur sa branche `train/…`, avec `verify_export` vert.
2. Il merge `main` dans sa branche, pour avoir le dernier code robot.
3. Tim, ou celui qui opère le robot, déploie **depuis cette branche** avec `deploy_to_pi.sh --to <commit>` et passe les niveaux 2 et 3 de la checklist.
4. Si c'est concluant, on merge la branche dans `main`, code + bundle, mais jamais les `results/`.

Ce qui reste à Tim : tout `controller/`. Ce qui reste à Léo-Paul : `BalanceRL/`, `RLframework/` et `docs/rl/`. Chacun peut toucher à la zone de l'autre, mais prévient l'autre avant de merger dans `main`.

## Archive et catalogue du travail RL

### Les trois plants « debout » de Léo

Ce sont les trois modèles comparés dans STAND-03 et STAND-04. Le xshape reste **actif**, parce que c'est le plant de la policy déployée. V1 et V2 passent en archive, avec une copie figée des trois pour pouvoir reproduire la comparaison.

| Fichier (`BalanceRL/model/`) | Pied | Référence au repos | STAND-04, rejeu 80 essais | Destination |
| --- | --- | --- | --- | --- |
| `stand_v1.xml` | semelle continue (V1 d'origine) | `quiet_v1.json` | 59/80 ; échoue l'essai de poussée fixe à 6,90 s | `archive/models/standing/` |
| `stand_v2.xml` | deux patins | `quiet_v2.json` | 52/80 ; échoue à 6,80 s | `archive/models/standing/` |
| `stand_xshape.xml` | deux patins, masses X **provisoires** | `quiet_xshape.json` | 72/80 ; tient les 20 s | reste dans `BalanceRL/model/` + copie dans `archive/` |

Source : `PROJECT_CONTEXT.md` de Léo, mis à jour le 11/11. Ce sont des résultats **simulation uniquement**. Le générateur `BalanceRL/model/build_three_standing.py` va avec les trois fichiers.

### Catalogue des séries RL (à compléter avec Léo → `docs/rl/EXPERIMENTS.md`)

Léo a déjà une règle claire pour les artefacts dans `docs/experiments/README.md` : le code et les petites métriques sont versionnés, les traces brutes et les checkpoints restent sur SCITAS. On la reprend telle quelle. Ce qui manque, c'est une vue d'ensemble lisible par la génération suivante, qui dise ce qu'on a tenté, ce qu'on en a appris et où sont les données.

| Série | Dates | Question | Résultat connu | Testé sur robot |
| --- | --- | --- | --- | --- |
| STAND-05 pose-return | 11/11 | revenir à la posture de repos après récupération | implémenté, 23 tests ; **pas entraîné** | non |
| STAND-04 | 10–11/11 | tenir debout + pas de récupération après poussée, 3 plants | voir tableau ci-dessus ; `standing04_xshape_best` | **oui, déployé (runs 1–9, 17–20/11)** |
| STAND-03 | 09/11 | tenir debout, 3 plants | best 78/80 (V1), 69/80 (V2), 70/80 (X) | non |
| CLEARANCE-01/02 | 25–27/10 | lever le pied en marche | archivé 12/12 essais de 30 s à 1,50 m/s ; rejeu frais 9/12, écart **non expliqué** | non |
| SPEED-01, WALK-01 | 19/10 | vitesse et marche sur le plant CAD-v2 | à résumer | non |
| GAIT-01 à 11 | 14–19/10 | marche sur le nouveau plant CAD ; shaping bilatéral, drag de contact | vidéos de 0,10 à 0,35 m/s | non |
| STOP-01 à 06 | 02–12/10 | s'arrêter et tenir : PPO, imitation, rehearsal, arrêt en mouvement | STOP-02 v3 : 0/32 holds ; STOP-03 (imitation) : 21/32 | non |
| Avant Léo (Nathann) | 2026 | sprint m1→m6, Fourier gait, walk\_mit, leg2d | 3 m/s en simulation ; mur historique à m3 | non |

Pour chaque série, une fiche de 10 lignes dans `docs/rl/EXPERIMENTS.md` :

- objectif et hypothèse ;
- plant (`.xml` et son hash) ;
- config (preset dans `config.py`) ;
- nombre de steps ;
- critère de succès fixé *avant* ;
- résultat chiffré ;
- ce qui a été appris ;
- chemin SCITAS des artefacts ;
- policy exportée, s'il y en a une ;
- statut.

Il faut vérifier la durée de rétention de `/home/rizkalla/running_robot_experiments/` sur SCITAS. Si Léo quitte le projet ou si son compte expire, les checkpoints disparaissent. Il faut donc copier au moins les `best.msgpack` et les `config.json` de chaque série sur le drive du labo.

## README, .gitignore, historique et gros fichiers

Taille visée : environ 42 Mo pour `controller/` après exclusions, plus le code RL et le modèle, soit moins de 100 Mo au total. Une vidéo, un run ou un checkpoint intermédiaire n'entre jamais dans git. Une policy déployée (environ 1 Mo) y entre, et elle part aussi en GitHub Release.

### README.md (racine) — sections

1. **Ce qu'est DASH-01**, en 5 lignes : 15,14 kg, 4× AKE90-8 + 2× AK60-39, RPi 3B, boucle à 100 Hz, MuJoCo + PPO maison.
2. **État actuel, daté** : ce qui tourne sur le robot, ce qui n'est validé qu'en simulation, et la policy déployée.
3. **Plan du repo** : l'arborescence, avec une ligne par dossier.
4. **Quickstart sim** : `pip install -r …`, lancer les tests, visualiser le plant.
5. **Déployer sur le robot** : un lien vers `controller/README.md` (déjà complet), plus la checklist de sécurité.
6. **Lancer une policy sans le webui** : `policy_cli`, à reprendre du journal du 20/11 §6.
7. **Entraîner une nouvelle policy** : `training/standing` puis export, `verify_export`, bundle, déploiement.
8. **Contribuer** : branches, messages de commit, checklist avant `main`.
9. **Historique et crédits** : Nathann Morand, Max Aebi, Léo, Tim ; un lien vers `HISTORY.md` et vers `DASH-01-archive`.

### .gitignore

```text
# Python
__pycache__/
*.py[cod]
.venv/
venv/
*.egg-info/
.pytest_cache/
# OS / IDE
.DS_Store
__MACOSX/
.vscode/
.idea/
.claude/
# Training outputs (live on SCITAS / drive)
runs/
results/
logs/
wandb/
*.msgpack
*.ckpt
events.out.tfevents.*
# Robot runtime data
controller/fixed_gait/webui/data/blackbox/
*.bbseg
*.bbdump
# Media and CAD sources
*.mp4
*.mov
*.gif
*.ipt
*.iam
*.stp
*.step
*.exe
# Keep deployed policies
!controller/deploy/bundles/*.npz
!checkpoints/**/best.msgpack
```

À vérifier : `controller/fixed_gait/webui/data/` contient trois `.npz` versionnés (workspaces, trajectoires, 1 Mo au total). On les garde, et on n'ignore que `blackbox/`.

### .gitattributes

On reprend celui de l'ancien repo pour `*.sh text eol=lf` (scripts exécutés sur le Pi et sur SCITAS) et pour les binaires (`*.npz`, `*.stl`, `*.png` en `binary`). On enlève les règles `/report/**`, puisque `report/` n'est pas importé.

### HISTORY.md — chronologie

Une frise courte, une ligne par jalon, qui pointe vers l'archive pour le détail :

- thèses Morand et Aebi (V1, sprint RL m1→m6, 3 m/s en simulation) ;
- rebuild V2 de Tim (hanche, pied, homing v7 puis v8, ankle verrouillé) ;
- séries RL de Léo, de STOP à STAND-05 ;
- sim2real de Tim : premier déploiement, plan B (worker), plan C (CAN brut, IMU 200 Hz), runs 1 à 9 ;
- migration vers `dash-sim2real`.

### CHANGELOG.md — un tableau, le plus récent en haut

| Date | Tag | Commit | Ce qui a changé sur le robot | Résultat |
| --- | --- | --- | --- | --- |
| 2026-11-20 | (ancien repo) | `cfd33c9` | IMU à 200 Hz (revert de 120 Hz), CAN brut | run 9 : 100 Hz depuis le terminal, chute vers l'avant après 0,88 s |

La première ligne reprend l'état avant la migration. Ensuite, on ajoute une ligne à chaque `deploy_to_pi.sh`.

## Avant de merger dans main

`main` est valide quand les trois niveaux passent, dans l'ordre. Un changement qui ne touche que `training/` ou `docs/` s'arrête au niveau 1. Tout ce qui touche `controller/` va jusqu'au niveau 3.

### Niveau 1 — Hors ligne (Mac, 5 min)

- [ ] `python -m pytest controller/fixed_gait/webui/tests controller/deploy/tests` : 516 passés, et les 7 échecs connus marqués `xfail` avec leur raison. **Échec = n'importe quel nouvel échec.**
- [ ] `python -m pytest BalanceRL/tests RLframework/tests`, plus `RLframework/smoke_test.py` : verts.
- [ ] `verify_export` sur chaque bundle de `controller/deploy/bundles/` : sortie identique au bit près à celle de la policy d'origine.
- [ ] Rejeu du run 9 avec `replay_policy_offline.py` : consignes à 0,17° près des consignes enregistrées (journal §3.11).

### Niveau 2 — Pi, sans moteurs actifs (robot en LIMP, 10 min)

- [ ] `./controller/deploy_to_pi.sh --check` : liste exactement les fichiers attendus, sans aucun écart du côté du Pi.
- [ ] Déploiement, puis les deux services `active`, et les 6 moteurs vus vivants.
- [ ] `controller/tools/can_probe.py` : `right.abd` et `right.thigh` à 5,00 ms. `right.cam` est un point ouvert tant que l'émission périodique n'est pas réglée dans l'outil CubeMars.
- [ ] `policy_cli limp` relâche bien tous les moteurs.

### Niveau 3 — Robot dans le harnais (opérateur présent, e-stop à la main)

- [ ] Pose de départ atteinte (`arrived (every joint within 0.5 deg)`), lancement depuis le terminal, onglet web fermé.
- [ ] Boucle ≥ 99 Hz, et `timing_ms.total` p90 ≤ 10 ms. Référence : run 9, 100,0 Hz et p90 8,9 ms.
- [ ] La garde de rythme, le fall-kill et l'e-stop web déclenchent toujours un arrêt. On le teste une fois par session, robot suspendu.
- [ ] Le run est écrit dans `CHANGELOG.md` et le commit est tagué `robot-…`.

Critère de réussite « tient debout », à définir séparément. Il ne bloque pas `main`, mais c'est l'objectif de la prochaine branche `train/` : par exemple, tangage dans ±2° pendant 10 s, harnais détendu. Aujourd'hui, ce critère n'est **pas** atteint : la policy part vers l'avant, et l'écart de −6,5° entre le modèle et le robot réel n'est pas résolu.

## Questions ouvertes à trancher avec Léo

- [ ] Léo-Paul et Léopold sont-ils la même personne, le compte `leorizkallah` ? Qui est responsable de `training/` ?
- [ ] Le robot physique actuel correspond-il à `dash01_homing_v8` (Tim, twin du webui) ou à `dash-01CAD-v2/` export v36 (Léo) ? On ne garde qu'un seul `model/` actif.
- [ ] Les masses du xshape sont marquées « provisoires ». Elles sont un bon candidat pour expliquer l'écart de −6,5° entre le modèle et le robot. Qui prèse les pièces et met `stand_xshape.xml` à jour ?
- [ ] SCITAS : quelle est la rétention de `/home/rizkalla/running_robot_*` ? Quels checkpoints faut-il copier sur le drive du labo avant la fin du semestre ?
- [ ] Les vidéos et données de `report/` servent-elles à un rendu ? Si oui, il faut les mettre sur le drive avant d'archiver.
- [ ] Date de gel de l'ancien repo, et qui fait le transfert (Léo, propriétaire).
- [ ] Le repo est-il public ou privé ? La licence actuelle (`LICENSE`) permet-elle de republier le code ? Faut-il l'accord du labo REHAssist ?
- [ ] `walk_mit/` et `sprint_runner/` de Nathann : faut-il garder une version code seul dans `archive/`, ou seulement le lien vers `DASH-01-archive` ?

* [ ] Diffusion des thèses et des rapports : Nathann, Max Aebi et le labo sont-ils d'accord pour les mettre dans `dash-reports` ? Si le repo est public, il faut leur accord explicite ; sinon, on le met en privé.
* [ ] Le `dash-reports` de Léo-Paul contient des scripts de suivi SCITAS (`video-pipeline.json`, PID local). Faut-il les garder actifs, ou les figer en lecture seule ?

## Sources

- Branche `policy-worker` @ `b25d404` et `dash-01-v2-speed-discovery` @ `3574ddd` (clones locaux de Tim), dont `controller/README.md`, `controller/deploy_to_pi.sh`, `PROJECT_CONTEXT.md` et `docs/experiments/README.md`.
- `Dash-01/SimToReal/DASH01_journal_2026-11-20.md`, ainsi que les rapports des plans B et C.
- Project knowledge Dash-01 : `00_INDEX.md`, et les échanges précédents sur le midterm et le premier déploiement.
