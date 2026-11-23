# tools/kd_validation — is Kd the sim-to-real problem?

Test plan (French): [`docs/sim2real/DASH01_plan_test_Kd_2026-11-24.md`](../../docs/sim2real/DASH01_plan_test_Kd_2026-11-24.md).

The idea: take one step of `standing04_xshape_best` from a simulated run where it stands, freeze it
(the same MIT frame — target, kp, kd per joint — on every tick), and run it on the robot with only
the leg kd changed (5 / 2 / 1 / 0.5, plus kd 5 with kp halved). The two hip rolls get the bundle's
nominal frame (action 0: `nominal_ctrl`, kp 120, kd 4): both quiet stances of this policy have one
nearly limp hip roll or leg (searched over 48 pushed sim runs), and the abduction is not under test
(`make_frozen_bundles.py --abduction step` keeps the step's own). Each variant is an ordinary v3
bundle whose action head is a constant (`act_w = 0`, `act_b = a*`), so the daemon runs it with every
guard and logs it like any policy. No robot code changes.

| File | Runs on | Purpose |
| --- | --- | --- |
| `sim_closed_loop.py` | Mac, needs `mujoco` | numpy MuJoCo closed loop: the deployed controller + PD drive at 1 kHz with a 12 ms delay on `BalanceRL/model/stand_*.xml` (meshes from `dash-01CAD/homing/dash01_homing_v8/meshes/`) |
| `pick_step.py` | Mac, needs `mujoco` | picks the step and runs the sim checks; writes `step_<bundle>.json` |
| `step_standing04_xshape_best.json` | — | the picked step (tick 1499 of a 20 s run) and the sim results |
| `make_frozen_bundles.py` | anywhere, numpy | writes `controller/deploy/bundles/kdval_*.npz` from the step JSON and checks each through the deployed controller |
| `test_frozen_bundles.py` | anywhere, pytest | the frame is constant and as declared; the daemon arms one on MockBus (worker included) and logs that frame |
| `analyze_runs.py` | anywhere, numpy (+ matplotlib for figures) | robot run logs → current ripple, velocity noise, spectrum, effective kp/kd, cam/thigh fight |

```bash
uv venv /tmp/simenv && uv pip install --python /tmp/simenv/bin/python mujoco numpy
/tmp/simenv/bin/python tools/kd_validation/pick_step.py          # ~1 min
python3 tools/kd_validation/make_frozen_bundles.py
python3 -m pytest tools/kd_validation/test_frozen_bundles.py
python3 tools/kd_validation/analyze_runs.py <policyrun_*.npz> --labels ... --out <dir>
```

Sim findings (nominal plant, no noise): the policy itself pins the cam kd at 5, like on the robot,
but its kp barely moves between ticks (vs 40–55 N·m/rad rms on the robot). The frozen step does not
stand on its own (it falls backwards in 1.6 s): the harness must hold the torso. Capping the leg kd
at 1 does not break the policy in sim (±30 N pushes still recovered).
