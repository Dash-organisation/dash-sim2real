# docs/sim2real — sim-to-real journals and analyses

Journals, plans and reports from deploying `standing04_xshape_best` on DASH-01 (2026-11-17 to 11-20). **The documents are in French**; each one is summarised in English below.

The `Logs/…` paths are the ones from the original folder (`Dash-01/SimToReal/`), so that the links inside the journals stay valid.

**Raw data** (`.bbseg`, `.bbdump`, CSV, JSON and NPZ of runs 3 to 9 and of the plan C benches) is not in this repo. It is in [`dash-reports`](https://github.com/Dash-organisation/dash-reports), under `sim2real-logs/`, with the same layout as `Logs/` here.

## Journals

**[`old_journal_plan/DASH01_journal_2026-11-17.md`](old_journal_plan/DASH01_journal_2026-11-17.md): first deployment, runs 1 to 4.**
First tests of the standing policy on the real robot. Introduces the scripted `deploy_to_pi.sh` and the decision to zero the motors standing. Three input bugs are fixed on the robot: the IMU mount was 180° off, the gyro was in deg/s instead of rad/s, and a calibration sign was missing on the torque observation. A workspace guard stops run 2, so the knee stop is moved mechanically. Runs 3 and 4 show the policy does right the robot from a −7 to −9° heel tilt, but the rate guard stops it because the loop only holds 60 Hz (run 3) and 76 Hz (run 4) instead of 100. The blamed cause is Python GIL contention in the daemon, which leads to plan B.

**[`DASH01_journal_2026-11-20.md`](DASH01_journal_2026-11-20.md): plans B and C, runs 5 to 9. This is the reference state.**
- **Plan B** (inference in a separate process) works and is bit-identical, but brings no gain on the Pi.
- An investigation of the daemon loop leads to **plan C**: a raw CAN socket, and the IMU tried at 120 Hz and then put back to 200 Hz (`cfd33c9`, the version now on the Pi).
- **Runs 5 to 9.** Launched from the web UI, runs reach 83 to 89 Hz. Run 9, launched from the terminal with `policy_cli.py`, holds 100.0 Hz, which settles the loop-rate problem.
- **The policy does not stop the robot tipping forward.** An offline replay (§3.11) shows that it corrects in the right direction but weakly, and that it softens the thigh gains. The real robot also sits at −6.5° on its heels in the start pose, while the simulated robot stands on its own: the robot is outside the domain the policy was trained on.
- **Decisions:** do not re-zero the IMU while standing. Measure the centre-of-mass gap, then retrain if it exceeds about 3 cm.
- §6 is a step-by-step procedure for running the policy without the web UI.

**[`DASH01_journal_2026-11-24.md`](DASH01_journal_2026-11-24.md): the Kd test. The whistle comes from the drives' force-control (MIT) mode.**
- The first frozen run (A) cut the power to the whole robot after 0.87 s. The cause is not proven; the robot may have been charging.
- The frozen-frame runs (kd 0.5 to 5, kp 100 and 200) all whistle the same way, while SET_POS under the same load is nearly silent. The policy, kp and kd are not the cause.
- On the panel (MIT hold), it whistles even at kp = kd = 0.1, at a fixed 2.5 kHz, louder under load. Above some torque the whole leg goes into a **self-sustained oscillation that only LIMP stops**. **Safety rule: no MIT under load until this is fixed.**
- The static stiffness measured in MIT is 1.7 to 1.9 times the commanded kp, a direct sim-to-real gap to confirm with a known mass.
- It also explains, from Nathann's thesis, why the stack uses MIT (bandwidth 6 Hz against 1.9 Hz in servo mode, and a law that matches MuJoCo), and whether SET_POS could work (standing probably, running no). It ends with next steps and a message to CubeMars.

## Plans and reports

**[`old_journal_plan/DASH01_plan_B_policy_process.md`](old_journal_plan/DASH01_plan_B_policy_process.md): plan B.**
Handover document for an overnight agent: run the policy inference in its own process (the policy worker) to get around the GIL. It covers the synchronous protocol, the 6 ms deadline, refusing to arm when the worker fails to start, and the tests.

**[`old_journal_plan/DASH01_plan_B_RAPPORT.md`](old_journal_plan/DASH01_plan_B_RAPPORT.md): plan B implementation report.**
The commits on `policy-worker`: per-stage tick timing, `policy_worker.py`, daemon integration, tests and bench. It also gives the rollback switches.

**[`DASH01_plan_C_can_imu.md`](DASH01_plan_C_can_imu.md): plan C.**
Handover document, with measurements, for replacing python-can with a raw `AF_CAN` socket and running the IMU at 120 Hz to free CPU time in the 10 ms tick.

**[`DASH01_plan_C_RAPPORT.md`](DASH01_plan_C_RAPPORT.md): plan C execution report.**
The work was done in five commits:
- `can_probe.py`, a passive probe;
- `RawCanBus`;
- `txqueuelen 100`;
- the IMU at 120 Hz (later reverted);
- draining the CAN queue with `poll(0)`, measured on the Pi at 5 µs against 55 µs for the exception path.

The CAN share of the CPU drops from 33 to 17 %. On-robot tests sent no frame to any motor.

**[`DASH01_plan_tests_bruit_moteurs.md`](DASH01_plan_tests_bruit_moteurs.md) and [`Logs/ANALYSE_bruit_moteurs.md`](Logs/ANALYSE_bruit_moteurs.md): high-pitched motor noise during the policy.**
The analysis covers runs 3 and 4:
- the noise is not a current limit;
- the cam motors show a 4 to 6 A rms current ripple in MIT mode, against nearly zero in SET_POS;
- the cam and the thigh fight each other through the foot.

The test plan lists hypotheses (gains: kd stuck at 5 and kp jumping between ticks; the cam–thigh fight; the loop rate) and tests to separate them.

**[`DASH01_plan_test_Kd_2026-11-24.md`](DASH01_plan_test_Kd_2026-11-24.md): is Kd the sim-to-real problem? (test of 2026-11-24, branch `hw/Kd_validation`).**
Six bundles (`controller/deploy/bundles/kdval_B` … `G`, constant action head, no robot code change) send one fixed MIT frame. Since the power cut on the first run (24/11) it is the bundle's action 0 — the standing stance `nominal_ctrl`, kp 200 / 120, the handover frame the daemon already sends — instead of a frozen policy step. They differ only in the leg gains: kd 5 / 2 / 1 / 0.5 at kp 200, kd 5 / 1 at kp 100. Each runs 10 s on the robot in the harness, once still (noise) and once with gentle pushes (effective kp/kd). Prepared evidence:
- in runs 7–8, the cam at kd ≈ 1.4 ripples 0.2 A while the cam at kd 5 ripples 3.4–3.9 A;
- in simulation the policy also pins the cam kd at 5, but capping it at 1 does not break the policy;
- the frozen step falls on its own in simulation, so the harness must hold the torso.

Tools and analysis: `tools/kd_validation/`.

**[`DASH-sim2real_plan_migration.md`](DASH-sim2real_plan_migration.md): migration plan.**
How this repo, `dash-reports` and the archive were built from the old DASH-01 repo, plus the branch rules and the checklist before merging into `main`.

## Run analyses

| File | Summary |
| --- | --- |
| [`Logs/DASH01_run3_2026-11-17/ANALYSE.md`](Logs/DASH01_run3_2026-11-17/ANALYSE.md) | run 3: the policy rights a −7° heel tilt, but the loop holds only 60 Hz and the rate guard stops it after 1.69 s |
| [`Logs/DASH01_run4_2026-11-17/ANALYSE.md`](Logs/DASH01_run4_2026-11-17/ANALYSE.md) | run 4, with the web UI throttled: 76 Hz, stopped after 1.32 s |
| [`Logs/DASH01_run5_2026-11-20/ANALYSE.md`](Logs/DASH01_run5_2026-11-20/ANALYSE.md) | run 5, first run after plan C (`cfd33c9`, worker active): 83 Hz; the run file logs the gains the policy requests at every tick |
| [`Logs/DASH01_runs6-9_2026-11-20/ANALYSE.md`](Logs/DASH01_runs6-9_2026-11-20/ANALYSE.md) | runs 6 to 8 from the web UI (88.5–88.8 Hz), run 9 from the terminal (100.0 Hz, tips forward +0.1 → +14.1°, hard stop after 0.88 s); §4 is the offline replay analysis |

## Scripts

| File | Purpose |
| --- | --- |
| [`Logs/DASH01_runs6-9_2026-11-20/replay_policy_offline.py`](Logs/DASH01_runs6-9_2026-11-20/replay_policy_offline.py) | offline replay of a run through the real controller (level 1 of the checklist). Its paths are hard-coded to the original Mac layout |
| [`Logs/plan_C/`](Logs/plan_C/) `cores.py`, `bench_gov.py`, `vcan_check.py` | plan C bench scripts run on the Pi: per-core CPU load from `/proc/stat`; pure compute cost of `ctrl.step` and `SafetyGovernor.step` with the real bundle (no CAN); byte identity and tick cost of python-can vs `RawCanBus` on a virtual CAN interface only |
