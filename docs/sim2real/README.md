# docs/sim2real — journaux et analyses du passage sim → robot

Journaux de bord, plans et rapports du déploiement de `standing04_xshape_best` sur DASH-01 (17–20/11/2026).

| Fichier | Contenu |
| --- | --- |
| `old_journal_plan/DASH01_journal_2026-11-17.md` | premier déploiement, runs 1 à 4 |
| `old_journal_plan/DASH01_plan_B_policy_process.md`, `DASH01_plan_B_RAPPORT.md` | plan B : inférence dans un process séparé (policy worker) |
| `DASH01_plan_C_can_imu.md`, `DASH01_plan_C_RAPPORT.md` | plan C : CAN brut, IMU 200 Hz |
| `DASH01_plan_tests_bruit_moteurs.md`, `Logs/ANALYSE_bruit_moteurs.md` | bruit des moteurs |
| `DASH01_journal_2026-11-20.md` | runs 5 à 9 ; état de référence (100 Hz, chute vers l'avant) |
| `DASH-sim2real_plan_migration.md` | plan de migration vers ce repo |
| `Logs/<run>/ANALYSE.md` | analyse de chaque run |
| `Logs/DASH01_runs6-9_2026-11-20/replay_policy_offline.py` | rejeu hors ligne d'un run (niveau 1 de la checklist) |
| `Logs/plan_C/*.py` | scripts de banc du plan C |

Les chemins `Logs/…` sont ceux du dossier d'origine (`Dash-01/SimToReal/`), pour que les liens des journaux restent valides.

**Données brutes** (`.bbseg`, `.bbdump`, CSV, JSON et NPZ des runs 3 à 9 et du plan C) : elles ne sont pas dans ce repo, mais dans `dash-reports`, sous `sim2real-logs/`, avec la même arborescence que `Logs/` ici.
