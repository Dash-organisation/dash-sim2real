# DASH-01 robot (hardware side)

Everything about the physical robot, self-contained so this folder can move to its own repo:

| folder | what |
|---|---|
| `fixed_gait/` | the Flask web control UI (`fixed_gait/webui/`) plus the teach/replay and workspace-calibration scripts it imports |
| `deploy/` | policy deployment: numpy control laws (v1/v2/balance), safety governor, thermal observer, exported bundles |
| `identification/` | offline dynamic-parameter identification used by the web UI's sys-ID panel |
| `robotCADdescription/` | CAD of the PRE-REBUILD robot: Inventor/STEP source in `CAD/` (still the source for the nine parts the rebuild did not touch) and the `MJCF_OPEN_MUJOCO_B` export `model/dash01.xml` reads its meshes from. The six unreferenced export formats were deleted. The CURRENT robot's CAD is `dash-01CAD/` at the repo root. |
| `model/` | dash01.xml (simulation-ready model, meshes referenced from `robotCADdescription/`), ride-height LUT, reachability plotting |
| `tools/` | AK60/AKE90 motor bring-up, CAN scan and MIT-mode probing utilities |
| `IMU.md` | **measured** properties of the Sense HAT (B) IMU — ranges, resolution, noise, bias, latency, frames, and a ready-to-use sim2real sensor model |
| `requirements-rpi.txt` | Raspberry Pi runtime deps (CAN streaming) |
| `requirements-webui.txt` | web control UI deps |
| `deploy_to_pi.sh` | copies a commit's `controller/` changes to the robot's Pi, with checks, backup and rollback — see below |

Hardware summary: 6 actuated DOF (per leg: hip_roll abduction, cam + thigh driving the sagittal
4-bar with parallel knee, passive ankle spring). Motors: AK60-39 + AKE90-8 over CAN
(can0 = right, can1 = left, IDs 104/105/106). Inertial sensing: Waveshare Sense HAT (B) on the Pi's
I2C bus, mounted under the base — see [IMU.md](IMU.md).

RL / policy training lives in `../RLframework/` (walking) and `../BalanceRL/` (push-recovery
standing), each with its own copy of the plant built from `../dash-01CAD/`.

## Deploying to the robot's Pi

The Pi (`nemo@192.168.4.1` on the robot's hotspot) has **no internet**, and its `~/running_robot`
is a clone of another repo (nathmo/running_robot), so `git pull` is not an option: code is copied
from this repo with rsync. **On the Pi this folder is called `robot/`, not `controller/`:**

| this repo | on the Pi |
|---|---|
| `controller/X` | `~/running_robot/robot/X` |
| service | `runningrobot-webui` runs `~/running_robot/.venv/bin/python robot/fixed_gait/webui/server.py --port 8081` from `~/running_robot` |

### The everyday workflow

```bash
# 1. change the code, run the tests, commit (only committed code is deployed)
python -m pytest controller/fixed_gait/webui/tests controller/deploy/tests
git commit -am "..."

# 2. see what would change on the Pi (writes nothing)
./controller/deploy_to_pi.sh --check

# 3. deploy: asks before copying, then before restarting the service
./controller/deploy_to_pi.sh

# 4. reload the web UI in the browser with Cmd+Shift+R (Ctrl+Shift+R)
```

The script deploys **commits, not the working tree**: it diffs `controller/` between the commit the
Pi runs and the target (`HEAD`, or `--to <commit>`), takes the files from git, and leaves out every
`tests/` folder and the script itself. It then, stopping at the first surprise:

1. reads the commit the Pi runs from `~/deploy_backups/deployed_commit` (or `--from <commit>`);
2. checks the service really runs `~/running_robot/robot/fixed_gait/webui/server.py` — otherwise
   copying would change nothing;
3. fetches the Pi's copy of every file it is about to replace or delete and checks it is still the
   deployed version (line endings ignored — older copies on the Pi are CRLF). A file someone edited
   on the Pi, or a Pi that is not on the commit you think, **aborts with the diffs and copies
   nothing**; `--force` overrides it once you have looked (those files are backed up first);
4. backs up exactly those files to `~/deploy_backups/<date>_<from>_to_<to>/` on the Pi
   (`files.tgz`, `added.txt`, `from_commit`, `info`), named with the Mac's clock — the Pi has no
   RTC/NTP and its date is wrong;
5. rsyncs the new versions, deletes the files the commits removed;
6. checks md5 on the Pi == git, `py_compile` on every `.py`, `json.tool` on every `.json`, then
   records the new commit in `~/deploy_backups/deployed_commit` and `history.log`;
7. restarts the service (only if something other than `fixed_gait/webui/static/` changed — static
   files are read per request), then fails loudly if it is not `active`, restarted itself, or the
   journal shows `Traceback` / `Error` / `No module named` …

**Restarting releases the motors (0 A)**: the robot must be supported or lying down when you answer
yes. The script never commands the motors — no `/api/...` call, no `run_policy.py`, no CAN. When
`deploy/deploy_map.json` is part of a deploy it reminds you that the joint signs are not verified by
it: run `check_direction` with the robot suspended.

### Rolling back

```bash
./controller/deploy_to_pi.sh --rollback            # restore the latest backup, then offer a restart
./controller/deploy_to_pi.sh --rollback 20261117_140000_830017f_to_abc1234
ssh nemo@192.168.4.1 'ls ~/deploy_backups; cat ~/deploy_backups/history.log'
```

A rollback untars the saved files, removes the files that deploy added, and puts the previous commit
back in `deployed_commit`.

### Options

| option | |
|---|---|
| `--check` | dry run: connect, compare, print the plan; nothing is written on the Pi |
| `--to <commit>` | deploy that commit instead of `HEAD` (going back to an older commit works too) |
| `--from <commit>` | the commit the Pi runs, when `deployed_commit` is missing or wrong |
| `--yes` | don't ask before copying (the restart still asks) |
| `--restart` / `--no-restart` | restart without asking / never restart |
| `--force` | copy even when files on the Pi differ from `--from` (they are backed up) |

Environment: `DASH_PI_HOST` (default `nemo@192.168.4.1`), `DASH_PI_DIR` (`running_robot/robot`),
`DASH_PI_PY`, `DASH_PI_SERVICE`, `DASH_PI_BACKUPS`. `DASH_PI_TEST=1` lets you point `DASH_PI_DIR` at
a scratch copy the service does not run, to try the script itself.

### First run

The Pi was deployed by hand to `830017f` on 2026-11-17 (full backup of `robot/` from before that:
`~/controller_backup_2026-08-22_0051.tgz`, Pi clock). `deployed_commit` does not exist yet, so the
first run names the starting commit once:

```bash
./controller/deploy_to_pi.sh --from 830017f
```

### Doing it by hand (what the script automates)

```bash
cd controller
git diff --name-only <pi-commit> <new-commit> -- .                 # files to copy; drop tests/
f=fixed_gait/webui/daemon.py                                         # for each one: Pi == <pi-commit>?
ssh nemo@192.168.4.1 "cat running_robot/robot/$f" | diff --strip-trailing-cr <(git show <pi-commit>:controller/$f) - && echo OK
ssh nemo@192.168.4.1 'tar czf controller_backup_$(date +%F_%H%M).tgz -C running_robot robot'
rsync -avR deploy/x.py fixed_gait/webui/y.py nemo@192.168.4.1:running_robot/robot/
ssh nemo@192.168.4.1 'cd running_robot && .venv/bin/python -m py_compile robot/fixed_gait/webui/daemon.py robot/fixed_gait/webui/server.py'
ssh nemo@192.168.4.1 'sudo systemctl restart runningrobot-webui && sleep 4 && journalctl -u runningrobot-webui -n 80 --no-pager'
```
