# EXPERIMENTS — catalogue of the RL series

> **Skeleton, to be filled in with Léo-Paul.** Results come from `docs/rl/PROJECT_CONTEXT.md` (updated 2026-11-11). They are **simulation-only** results unless stated otherwise.

Artifact rule (from `docs/experiments/README.md`): code and small metrics are versioned; raw traces and checkpoints stay on SCITAS (`/home/rizkalla/running_robot_experiments/`). Files over 1 MB and the galleries are in `dash-reports`.

## Overview

| Series | Dates | Question | Known result | Tested on the robot |
| --- | --- | --- | --- | --- |
| STAND-05 pose-return | 11/11 | return to the rest posture after a recovery | implemented, 23 tests; **not trained** | no |
| STAND-04 | 10–11/11 | stand, plus a recovery step after a push, 3 plants | V1 59/80, V2 52/80, X 72/80 (80-trial replay); `standing04_xshape_best` | **yes, deployed (runs 1–9, 17–20/11)** |
| STAND-03 | 09/11 | stand, 3 plants | best 78/80 (V1), 69/80 (V2), 70/80 (X) | no |
| CLEARANCE-01/02 | 25–27/10 | lift the foot while walking | archived 12/12 trials of 30 s at 1.50 m/s; fresh replay 9/12, gap **not explained** | no |
| SPEED-01, WALK-01 | 19/10 | speed and walking on the CAD-v2 plant | to be summarised | no |
| GAIT-01 to 11 | 14–19/10 | walking on the new CAD plant; bilateral shaping, contact drag | videos from 0.10 to 0.35 m/s | no |
| STOP-01 to 06 | 02–12/10 | stop and hold: PPO, imitation, rehearsal, stopping while moving | STOP-02 v3: 0/32 holds; STOP-03 (imitation): 21/32 | no |
| Before Léo-Paul (Nathann) | 2026 | sprint m1→m6, Fourier gait, `walk_mit`, `leg2d` | 3 m/s in simulation; historical wall at m3 | no |

## The three standing plants (STAND-03/04)

| File (`BalanceRL/model/`) | Foot | Rest reference | STAND-04, 80-trial replay |
| --- | --- | --- | --- |
| `stand_v1.xml` | continuous sole (original V1) | `quiet_v1.json` | 59/80; fails the fixed-push trial at 6.90 s |
| `stand_v2.xml` | two pads | `quiet_v2.json` | 52/80; fails at 6.80 s |
| `stand_xshape.xml` | two pads, **provisional** X masses | `quiet_xshape.json` | 72/80; holds the full 20 s |

The STAND-03/04 `best.msgpack` checkpoints are in `checkpoints/standing0{3,4}/{v1,v2,xshape}/` (parents for a warm start). A frozen copy of the three plants is in `archive/models/standing/`.

## Entries

Template, one per series:

```text
### <SERIES>
- Goal and hypothesis:
- Plant (.xml and its hash):
- Config (preset in config.py):
- Number of steps:
- Success criterion (set beforehand):
- Numerical result:
- What was learned:
- SCITAS path of the artifacts:
- Exported policy:
- Status:
```

### STAND-05 pose-return
_to be filled in_

### STAND-04
_to be filled in_

### STAND-03
_to be filled in_

### CLEARANCE-01/02
_to be filled in_

### SPEED-01, WALK-01
_to be filled in_

### GAIT-01 to 11
_to be filled in_

### STOP-01 to 06
_to be filled in_
