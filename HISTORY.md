# HISTORY — chronologie de DASH-01

Une ligne par jalon. Le détail, avec tout l'historique git d'avant la migration, est dans `DASH-01-archive`. Les dates manquantes sont à compléter.

| Quand | Jalon | Où lire la suite |
| --- | --- | --- |
| avant 2026 | **Thèses de Nathann Morand et de Max Aebi** : robot V1, sprint RL m1→m6, Fourier gait, `walk_mit`, `leg2d` ; 3 m/s en simulation, avec un mur historique à m3 | thèses non publiées ; stacks `training/`, `walk_mit/`, `leg2d/`, `walk_v2/`, `rl/` dans `DASH-01-archive` (supprimés de l'arbre le 17/09/2026, `0031885`) |
| 17/09/2026 | Nathann crée `RLframework/` (`7180ee5`) | `RLframework/` |
| 24/09/2026 | Nathann crée `BalanceRL/` (`356c10f`) | `BalanceRL/` |
| à compléter | **Rebuild V2 de Tim** : hanche, pied, homing v7 puis v8, cheville verrouillée | `dash-01CAD/homing/dash01_homing_v8/`, `controller/README.md` |
| 02–12/10/2026 | Léo-Paul, séries **STOP-01 à 06** (s'arrêter et tenir : PPO, imitation, rehearsal) | `docs/rl/EXPERIMENTS.md`, `docs/experiments/` |
| 14–19/10/2026 | **GAIT-01 à 11**, **SPEED-01**, **WALK-01** (marche sur le plant CAD-v2) | idem |
| 25–27/10/2026 | **CLEARANCE-01/02** (lever le pied en marche) | idem |
| 09–11/11/2026 | **STAND-03**, **STAND-04** (trois plants debout), puis **STAND-05** pose-return (implémenté, pas entraîné). La policy `standing04_xshape_best` en sort | `docs/rl/PROJECT_CONTEXT.md`, `BalanceRL/` |
| 17/11/2026 | **Premier déploiement sur le robot** de `standing04_xshape_best`, runs 1 à 4 ; la boucle tient 60 puis 76 Hz au lieu de 100 | `docs/sim2real/old_journal_plan/DASH01_journal_2026-11-17.md` |
| 18–19/11/2026 | **Plan B** : inférence dans un process séparé (policy worker), branche `policy-worker` | `docs/sim2real/old_journal_plan/DASH01_plan_B_RAPPORT.md` |
| 20/11/2026 | **Plan C** : CAN en socket brute, IMU revenue à 200 Hz (`cfd33c9`, version du Pi) ; runs 5 à 9. Run 9 : 100 Hz depuis le terminal, mais la policy laisse partir le robot vers l'avant | `docs/sim2real/DASH01_journal_2026-11-20.md` |
| 22–23/11/2026 | **Migration vers `dash-sim2real`** : historique neuf ; l'ancien repo devient `DASH-01-archive` | `docs/sim2real/DASH-sim2real_plan_migration.md` |
