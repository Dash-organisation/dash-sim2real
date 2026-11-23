# HISTORY — DASH-01 timeline

One line per milestone. The details, with the full git history from before the migration, are in `DASH-01-archive`. Missing dates are still to be filled in.

| When | Milestone | Where to read more |
| --- | --- | --- |
| before 2026 | **Theses of Nathann Morand and Max Aebi**: V1 robot, RL sprint m1→m6, Fourier gait, `walk_mit`, `leg2d`; 3 m/s in simulation, with a historical wall at m3 | theses not published; `training/`, `walk_mit/`, `leg2d/`, `walk_v2/`, `rl/` stacks in `DASH-01-archive` (removed from the tree on 2026-09-17, `0031885`) |
| 2026-09-17 | Nathann creates `RLframework/` (`7180ee5`) | `RLframework/` |
| 2026-09-24 | Nathann creates `BalanceRL/` (`356c10f`) | `BalanceRL/` |
| to be filled in | **Tim's V2 rebuild**: hip, foot, homing v7 then v8, locked ankle | `dash-01CAD/homing/dash01_homing_v8/`, `controller/README.md` |
| 2026-10-02 to 10-12 | Léo-Paul, **STOP-01 to 06** series (stopping and holding: PPO, imitation, rehearsal) | `docs/rl/EXPERIMENTS.md`, `docs/experiments/` |
| 2026-10-14 to 10-19 | **GAIT-01 to 11**, **SPEED-01**, **WALK-01** (walking on the CAD-v2 plant) | same |
| 2026-10-25 to 10-27 | **CLEARANCE-01/02** (lifting the foot while walking) | same |
| 2026-11-09 to 11-11 | **STAND-03**, **STAND-04** (three standing plants), then **STAND-05** pose-return (implemented, not trained). The `standing04_xshape_best` policy comes out of this | `docs/rl/PROJECT_CONTEXT.md`, `BalanceRL/` |
| 2026-11-17 | **First deployment on the robot** of `standing04_xshape_best`, runs 1 to 4; the loop holds 60 then 76 Hz instead of 100 | `docs/sim2real/old_journal_plan/DASH01_journal_2026-11-17.md` |
| 2026-11-18 to 11-19 | **Plan B**: inference in a separate process (policy worker), `policy-worker` branch | `docs/sim2real/old_journal_plan/DASH01_plan_B_RAPPORT.md` |
| 2026-11-20 | **Plan C**: raw CAN socket, IMU back at 200 Hz (`cfd33c9`, the version on the Pi); runs 5 to 9. Run 9: 100 Hz from the terminal, but the policy lets the robot tip forward | `docs/sim2real/DASH01_journal_2026-11-20.md` |
| 2026-11-22 to 11-23 | **Migration to `dash-sim2real`**: fresh history; the old repo becomes `DASH-01-archive` | `docs/sim2real/DASH-sim2real_plan_migration.md` |

The documents under `docs/sim2real/` are in French; [`docs/sim2real/README.md`](docs/sim2real/README.md) summarises each one in English.
