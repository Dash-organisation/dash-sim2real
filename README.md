# dash-sim2real

Everything needed to train a policy for the DASH-01 robot and run it on the hardware: the robot stack, the training code, the model of the current robot, and the sim-to-real journals.

## 1. What DASH-01 is

- A 15.14 kg biped with 6 actuated degrees of freedom: 4× AKE90-8 and 2× AK60-39 on CAN (can0 = right leg, can1 = left leg).
- It runs on a Raspberry Pi 3B. The control loop runs at 100 Hz, and the IMU is a Sense HAT (B).
- Policies are trained in MuJoCo with an in-house PPO (JAX), on the SCITAS cluster.
- Hardware details: [`controller/README.md`](controller/README.md) and [`controller/IMU.md`](controller/IMU.md).

## 2. Current status (2026-11-23)

- **On the robot.** The Pi runs `controller/` at `cfd33c9` of the old repo (raw CAN, IMU at 200 Hz), which is the first commit of this repo. The deployed policy is `controller/deploy/bundles/standing04_xshape_best.npz`, trained on `BalanceRL/model/stand_xshape.xml`.
- **Validated on the robot:** the deployment chain and the safety layer. In run 9, launched from the terminal, the loop holds 100.0 Hz (`total` p90 8.9 ms).
- **Not validated: standing.** The policy lets the robot tip forward (+0.1° → +14.1° in 0.88 s). At rest, the real robot leans back to −6.5°, while the simulated robot stands on its own. See [`docs/sim2real/DASH01_journal_2026-11-20.md`](docs/sim2real/DASH01_journal_2026-11-20.md), §3.10–3.11 (in French; English summary in [`docs/sim2real/README.md`](docs/sim2real/README.md)).
- **Simulation only:** all RL series ([`docs/rl/EXPERIMENTS.md`](docs/rl/EXPERIMENTS.md)).

## 3. Repository layout

Phase 1 of the migration: folders keep their names from the old repo, so the paths in the code stay valid.

| Folder | Contents |
| --- | --- |
| `controller/` | robot stack: daemon and web UI, CAN, IMU, policy worker, `policy_cli`, `deploy_to_pi.sh`. Maps to `~/running_robot/robot/` on the Pi |
| `controller/deploy/bundles/` | exported policies (`.npz`), including the deployed one |
| `dash-01CAD/` | model of the current robot: homing v8 export (web UI twin), meshes, `dash01.xml` |
| `BalanceRL/` | standing training (STAND-03/04/05) and the three `stand_*.xml` plants |
| `RLframework/` | locomotion training (STOP, GAIT, SPEED, WALK, CLEARANCE), without `results/` |
| `checkpoints/` | STAND-03/04 and QUIET-01v2 `best.msgpack` files, parents for a warm start |
| `report/` | Léo-Paul's report workspace: write-ups, HTML galleries, figures, small data (files < 1 MiB; videos and heavy data are in `dash-reports`) |
| `tools/` | repo tooling: git hooks, `install-hooks.sh`, `sync_reports.sh` |
| `docs/sim2real/` | journals, plans B and C, motor noise, analyses of runs 3 to 9 (in French, with English summaries) |
| `docs/rl/` | `EXPERIMENTS.md` (catalogue of the series) and `PROJECT_CONTEXT.md` |
| `docs/experiments/` | experiment records (files < 1 MB; the rest is in `dash-reports`) |
| `archive/` | frozen copies, nothing active |

Videos, heavy report data and raw robot logs are in [`dash-reports`](https://github.com/Dash-organisation/dash-reports). Theses and presentations are not published. CAD will go to `dash-cad`.

## Working on this repo (training workflow)

Everything is done in this one repo, on one branch. The other repos are only read.

```bash
# once
git clone https://github.com/Dash-organisation/dash-sim2real.git && cd dash-sim2real
uv tool install graphifyy          # optional, saves Claude Code tokens (or: pipx install graphifyy)
tools/install-hooks.sh             # size check before each commit + graphify graph refresh

# each topic
git switch -c train/<topic>        # e.g. train/quiet02
# ... train, evaluate, regenerate reports in report/ as before ...
git add -A && git commit -m "BalanceRL: ..."    # files > 1 MiB are refused, they stay untracked
git push -u origin train/<topic>
git pull origin main               # from time to time, to get the latest robot code
```

- **What gets committed:** code, configs, tests, `docs/experiments/<series>/` records, the `docs/rl/EXPERIMENTS.md` entry, and `report/` write-ups, galleries and figures, as long as each file is under 1 MiB.
- **What stays untracked:** videos, `progress.jsonl`, archives, traces, `runs/`, `results/`. To publish the heavy report files, someone runs `tools/sync_reports.sh --push` from time to time: it copies `report/`, `docs/experiments/` and `RLframework/results/` into a `dash-reports` clone next to this one (`../dash-reports`).
- **Reading other repos:** clone `dash-reports` (videos, old reports, raw robot logs) or `dash-cad` next to this repo, and `git pull` there when needed. You never push to them.
- **Claude Code:** `CLAUDE.md` and `.claude/settings.json` are shared. Claude asks before any `git push`, `ssh`, `curl` or deployment, never force-pushes, and uses the graphify graph instead of reading whole files. Personal settings go in `.claude/settings.local.json` (ignored).

## 4. Quickstart in simulation (Mac)

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r controller/requirements-webui.txt -r RLframework/requirements.txt pytest

# offline tests: no motors, simulated CAN bus
python -m pytest controller/fixed_gait/webui/tests controller/deploy/tests
python -m pytest BalanceRL/tests RLframework/tests
python RLframework/smoke_test.py

# view the plant of the deployed policy
python -m mujoco.viewer --mjcf BalanceRL/model/stand_xshape.xml
```

## 5. Deploying to the robot

The full procedure is in [`controller/README.md`](controller/README.md), section "Deploying to the robot's Pi". Only committed code is deployed.

```bash
./controller/deploy_to_pi.sh --check   # shows what would change on the Pi, writes nothing
./controller/deploy_to_pi.sh           # asks before copying, then before restarting
```

The robot must be in LIMP. After every deployment, add a line to [`CHANGELOG.md`](CHANGELOG.md) and tag the commit `robot-YYYY-MM-DD-<n>`. Go through the "Before merging into main" checklist (section 8).

## 6. Running a policy without the web UI

Use `controller/tools/policy_cli.py` on the Pi. All safety checks stay in the daemon, and a run lasts 10 s at most. The detailed procedure is in [`docs/sim2real/DASH01_journal_2026-11-20.md`](docs/sim2real/DASH01_journal_2026-11-20.md), §6 (in French).

**Before you start:** the robot is in the harness, the physical E-STOP is within reach, and the web UI tab is closed on every device.

```bash
ssh nemo@192.168.4.1 && cd ~/running_robot/robot
python3 tools/policy_cli.py status
python3 tools/policy_cli.py pose standing04_xshape_best.npz       # MANUAL + start pose
python3 tools/policy_cli.py run  standing04_xshape_best.npz --allow-uncalibrated-thermal
python3 tools/policy_cli.py limp                                  # releases every motor
```

- **Ctrl+C once:** soft stop. **Twice:** hard stop.
- **E-STOP from a second terminal:** `curl -X POST http://192.168.4.1:8081/api/estop`.

## 7. Training a new policy

1. Work on a `train/<topic>` branch. Training runs on SCITAS (Izar); see `BalanceRL/README.md` ("Workflow") and `BalanceRL/slurm/`.
2. Export with the parity check:

   ```bash
   python BalanceRL/export.py --run BalanceRL/runs/<run> --out controller/deploy/bundles/<name>.npz
   ```

3. Check the bundle (`controller/deploy/verify_export.py`, `BalanceRL/verify_bundle.py`), then do a dress rehearsal on the simulated bus:

   ```bash
   python controller/deploy/run_policy.py --bundle controller/deploy/bundles/<name>.npz --mock --max-seconds 5
   ```

4. Deploy from the branch (section 5), pass levels 2 and 3 of the checklist, then merge into `main` (code and bundle, never `runs/` or `results/`).

Add an entry to [`docs/rl/EXPERIMENTS.md`](docs/rl/EXPERIMENTS.md).

## 8. Contributing

- **Branches.** `main` must be able to run on the robot at any time. Other branches are `train/<topic>` for training, `hw/<topic>` for mechanics, calibration, CAN and IMU (bench test required before merging), and `exp/<topic>` for throwaway experiments (never merged as is).
- **Commit messages.** Format `area: what changes`, for example `canio: RawCanBus drains with poll(0)`.
- **Before merging into `main`.** A change limited to `BalanceRL/`, `RLframework/` or `docs/` stops at level 1. Anything touching `controller/` goes up to level 3.
  - *Level 1, offline:* the tests above pass, with no new failure. `verify_export` passes on every bundle. The replay of run 9 (`docs/sim2real/Logs/DASH01_runs6-9_2026-11-20/replay_policy_offline.py`) stays within 0.17°.
  - *Level 2, Pi, robot in LIMP:* `deploy_to_pi.sh --check` is clean. Both services are `active` and all 6 motors are seen alive. `can_probe.py` is OK, and `policy_cli limp` releases everything.
  - *Level 3, robot in the harness, operator present, E-STOP in hand:* the start pose is reached. The loop holds at least 99 Hz, with `total` p90 ≤ 10 ms. The rate guard, the fall-kill and the E-STOP are tested once per session. The run is logged in `CHANGELOG.md` and the commit is tagged.

## 9. History and credits

- **Nathann Morand** and **Max Aebi**: V1 robot and theses; original authors of `RLframework/` and `BalanceRL/`.
- **Léo-Paul** (GitHub `leorizkallah`): RL series from STOP to STAND-05, and the deployed policy.
- **Tim**: V2 rebuild, robot stack and sim-to-real.

The timeline is in [`HISTORY.md`](HISTORY.md). The full git history from before the migration is in [`nathmo/running_robot`](https://github.com/nathmo/running_robot) (up to 2026-09-30) and [`leorizkallah/DASH-01`](https://github.com/leorizkallah/DASH-01) (October–November 2026, Léo-Paul's branches). Tim's `policy-worker` branch was not published; its final state is the first two commits of this repo (`cfd33c9`, then `b25d404`), and its commits are described in `docs/sim2real/`.

## License

[Mozilla Public License 2.0](LICENSE), the same as the old DASH-01 repo.
