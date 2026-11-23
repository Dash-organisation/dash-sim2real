#!/usr/bin/env python3
"""Run a policy from the terminal, without the web UI (2026-11-20).

Why: in run 5 the web page cost the control loop 135 of the 205 ms lost (the daemon re-reading the
cold state from the SD card twice a second, plus the dead-man requests). With no browser open,
none of that happens. Every safety check stays where it is, in the daemon: calibration,
workspace, governor, rate guard, dead-man, black box. This script only sends the same requests the
panel's buttons send.

    python3 policy_cli.py pose standing04_xshape_best.npz     # MANUAL, slow move to the start pose
    python3 policy_cli.py run  standing04_xshape_best.npz --allow-uncalibrated-thermal
    python3 policy_cli.py stop          # soft stop of a running policy (--hard: gains to zero NOW)
    python3 policy_cli.py limp          # release every motor (0 A), after a run
    python3 policy_cli.py estop         # E-STOP
    python3 policy_cli.py status        # mode, pose, last run

`run` asks you to confirm the robot is in the harness (that is the panel's "supported" box; --yes
skips the question), arms, then holds the dead-man (POLICY_DEADMAN_S = 1.5 s in the daemon) for as
long as the run lasts:
  * Ctrl+C once  -> soft stop (target frozen, gains bled out over 0.3 s: the robot is put down)
  * Ctrl+C twice -> hard stop (gains to zero this tick: it drops)
  * the script dies / ssh drops / Wi-Fi drops -> no more keepalive -> the daemon soft-stops in 1.5 s
At the end it saves the 200 Hz run log (data/policyruns/) and prints the timing summary.

Runs last at most MAX_SECONDS (10 s, the operator's limit). Best run ON THE PI, so the dead-man
does not depend on the Wi-Fi, and it pins itself to cores 2-3, away from the daemon:
    ssh nemo@192.168.4.1
    cd ~/running_robot/robot && python3 tools/policy_cli.py pose standing04_xshape_best.npz
From the Mac, add --url http://192.168.4.1:8081. Close the web UI tab: an open panel polls the
daemon (the cost this script exists to avoid) and also holds the control token for 15 s after each
click. Keep `curl -X POST http://192.168.4.1:8081/api/estop` ready in another terminal.

Standard library only (no requests, no numpy).
"""
import argparse
import json
import os
import signal
import socket
import sys
import time
import urllib.error
import urllib.request

MAX_SECONDS = 10.0             # the operator's cap for a run started from here
KEEPALIVE_S = 0.4              # 2.5 Hz; the daemon's dead-man is 1.5 s -> ~3 missed before a stop
APPEAR_GRACE_S = 3.0           # like the panel: "no run in the state" right after arming = not yet
POSE_TIMEOUT_S = 90.0
POSE_TOL_DEG = 0.5             # every joint this close to its target...
POSE_SETTLE_POLLS = 3          # ...for 3 polls in a row (1.5 s) = arrived
TOKEN_FILE = os.path.join(os.path.expanduser("~"), ".dash_policy_cli_token")


def default_url():
    env = os.environ.get("DASH_URL")
    if env:
        return env
    # on the robot's Pi the daemon is local; anywhere else, the hotspot address
    return "http://127.0.0.1:8081" if socket.gethostname() == "running" else "http://192.168.4.1:8081"


class Api:
    def __init__(self, url, timeout=5.0):
        self.url = url.rstrip("/")
        self.timeout = timeout

    def call(self, path, body=None, timeout=None):
        """(http_status, json) -- never raises for an HTTP error status, only for no connection."""
        data = None if body is None else json.dumps(body).encode()
        req = urllib.request.Request(self.url + path, data=data, method="GET" if body is None else "POST",
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=timeout or self.timeout) as r:
                return r.status, json.loads(r.read() or b"{}")
        except urllib.error.HTTPError as e:
            try:
                return e.code, json.loads(e.read() or b"{}")
            except ValueError:
                return e.code, {"ok": False, "error": "HTTP {}".format(e.code)}


def load_token():
    try:
        with open(TOKEN_FILE) as f:
            return f.read().strip() or None
    except OSError:
        return None


def save_token(tok):
    if not tok:
        return
    try:
        with open(TOKEN_FILE, "w") as f:
            f.write(tok)
    except OSError:
        pass


def die(msg, code=1):
    print("!! " + msg)
    sys.exit(code)


def pin_off_daemon_cores():
    if hasattr(os, "sched_setaffinity"):
        try:
            os.sched_setaffinity(0, {2, 3})       # the daemon owns 0-1
        except OSError:
            pass


def fmt_pose(state, targets=None):
    mot = state.get("motors") or {}
    parts = []
    for n in ("right.abd", "right.cam", "right.thigh", "left.abd", "left.cam", "left.thigh"):
        m = mot.get(n) or {}
        p = m.get("pos_norm")
        if p is None:
            parts.append("{} ?".format(n))
        elif targets and n in targets:
            parts.append("{} {:+.1f} ({:+.1f})".format(n, p, p - targets[n]))
        else:
            parts.append("{} {:+.1f}".format(n, p))
    return ", ".join(parts)


# ===================================================================== commands
def cmd_status(api, a):
    st, d = api.call("/api/state")
    if st != 200:
        die("daemon unreachable or error: {}".format(d.get("error", st)))
    man = d.get("manual") or {}
    print("mode {}  estop {}  loop {}  can_errors {}".format(
        d.get("mode"), (d.get("estop") or {}).get("latched"), d.get("loop"), d.get("can_errors")))
    print("homing {} ({})".format(man.get("homing"), man.get("homing_kind")))
    print("pose (normalized deg): " + fmt_pose(d))
    p = d.get("policy")
    if p:
        print("policy: {} phase {} running {} elapsed {} s rate {} Hz".format(
            p.get("file"), p.get("phase"), p.get("running"), p.get("elapsed_s"), p.get("rate_hz")))
    return 0


def cmd_pose(api, a):
    st, d = api.call("/api/manual/policy_pose",
                     {"file": a.file, "slew_dps": a.slew, "token": load_token(),
                      "takeover": bool(a.takeover)})
    if st == 409:
        die("{}\n   (the web panel holds the control token for 15 s after each click: close it, "
            "wait, or pass --takeover)".format(d.get("error")))
    if not d.get("ok"):
        die("pose refused: {}".format(d.get("error")))
    save_token(d.get("token"))
    targets = d.get("targets") or {}
    print("going to the policy pose of {} at {} deg/s (robot hung; lower it onto its soles once "
          "there)".format(a.file, a.slew))
    print("targets: " + ", ".join("{} {:+.1f}".format(k, v) for k, v in targets.items()))
    # Arrival = every joint within POSE_TOL_DEG of its target for POSE_SETTLE_POLLS polls in a row.
    # NOT the `homing` flag: the daemon keeps it up for as long as it HOLDS the pose (it only
    # drops when something else takes over), so it never falls on its own.
    t_end = time.time() + POSE_TIMEOUT_S
    settled = 0
    try:
        while time.time() < t_end:
            time.sleep(0.5)
            st, s = api.call("/api/state")
            if st != 200:
                continue
            print("  {:<7} {}".format(str(s.get("mode")), fmt_pose(s, targets)))
            if s.get("mode") != "MANUAL":
                die("left MANUAL during the move ({}); last reject: {}".format(
                    s.get("mode"), s.get("last_reject")))
            err = pose_error(s, targets)
            settled = settled + 1 if (err is not None and err <= POSE_TOL_DEG) else 0
            if settled >= POSE_SETTLE_POLLS:
                print("arrived (every joint within {:.1f} deg): holding the policy pose in MANUAL. "
                      "Next: `run`.".format(POSE_TOL_DEG))
                return 0
    except KeyboardInterrupt:
        print("\n(stopped watching; the daemon keeps moving to / holding the pose -- `status`)")
        return 130
    die("pose not reached within {:.0f} s (still holding; check `status`)".format(POSE_TIMEOUT_S))


def pose_error(state, targets):
    """Largest |measured - target| over the targeted joints, deg; None if one is unknown."""
    mot = state.get("motors") or {}
    worst = 0.0
    for n, tgt in (targets or {}).items():
        p = (mot.get(n) or {}).get("pos_norm")
        if p is None:
            return None
        worst = max(worst, abs(p - tgt))
    return worst if targets else None


class Interrupts:
    def __init__(self):
        self.n = 0

    def __call__(self, signum, frame):
        self.n += 1


def cmd_run(api, a):
    if a.seconds <= 0 or a.seconds > MAX_SECONDS:
        die("--seconds must be in (0, {:.0f}]".format(MAX_SECONDS))
    st, s = api.call("/api/state")
    if st != 200:
        die("daemon unreachable: {}".format(s.get("error", st)))
    # a standing start begins from the policy pose held in MANUAL; a hung run (kp-40 approach)
    # may start from LIMP, as in the panel
    if s.get("mode") != "MANUAL" and not (a.hung and s.get("mode") == "LIMP"):
        die("the robot is in {}, not MANUAL: run `pose` first".format(s.get("mode")))
    if not a.yes:
        ans = input("Robot in the harness, E-STOP within reach, web UI closed? [y/N] ")
        if ans.strip().lower() not in ("y", "yes", "o", "oui"):
            die("not armed", 2)
    spec = {"file": a.file, "supported": True, "max_seconds": float(a.seconds),
            "token": load_token(), "takeover": bool(a.takeover)}
    if a.allow_uncalibrated_thermal:
        spec["allow_uncalibrated_thermal"] = True
    if a.skip_jointmap_check:
        spec["skip_jointmap_check"] = True
    if a.hung:
        spec["hung"] = True
    if a.no_worker:
        spec["worker"] = False
    pin_off_daemon_cores()
    # Ctrl+C is ours from BEFORE the arm request: one pressed while it is in flight must still end
    # in a stop request, not in a dead script and the 1.5 s dead-man
    intr = Interrupts()
    old = signal.signal(signal.SIGINT, intr)
    try:
        # arming waits for the policy worker to start (numpy: 1-3 s on the Pi 3B)
        st, d = api.call("/api/policy/arm", spec, timeout=30.0)
    except BaseException:
        signal.signal(signal.SIGINT, old)
        raise
    if st == 409 or not d.get("ok"):
        signal.signal(signal.SIGINT, old)
        if st == 409:
            die("{}\n   (close the web panel, wait 15 s, or pass --takeover)".format(d.get("error")))
        die("arm refused: {}".format(d.get("error")))
    save_token(d.get("token"))
    t_arm = time.time()
    print("ARMED: {}  max {:.0f} s  worker {}  -- Ctrl+C = soft stop, twice = hard stop".format(
        a.file, a.seconds, not a.no_worker))
    sent_soft = sent_hard = False
    last_print = 0.0
    try:
        while True:
            t = time.time()
            if intr.n >= 1 and not sent_soft:
                api.call("/api/policy/stop", {})
                sent_soft = True
                print("-> soft stop sent")
            if intr.n >= 2 and not sent_hard:
                api.call("/api/policy/stop", {"hard": True})
                sent_hard = True
                print("-> HARD stop sent")
            if intr.n >= 3:
                print("-> leaving; the dead-man stops whatever is left within 1.5 s")
                return 3
            try:
                st, d = api.call("/api/policy/keepalive", {}, timeout=1.0)
            except (urllib.error.URLError, OSError) as e:
                print("   keepalive failed: {}".format(e))
                d = {}
            p = (d.get("state") or {}).get("policy") if d else None
            running = bool(p and p.get("running"))
            if not running and t - t_arm > APPEAR_GRACE_S:
                break
            if p and t - last_print >= 1.0:
                last_print = t
                print("   {:<9} {:>5.2f} s  {:>5.1f} Hz".format(
                    str(p.get("phase")), float(p.get("elapsed_s") or 0.0),
                    float(p.get("rate_hz") or 0.0)))
            time.sleep(max(0.0, KEEPALIVE_S - (time.time() - t)))
    finally:
        signal.signal(signal.SIGINT, old)

    st, d = api.call("/api/policy/run/save", {})
    if not d.get("ok"):
        print("!! run log not saved: {}".format(d.get("error")))
        return 1
    run = d.get("run") or {}
    print_summary(d.get("file"), run, (d.get("state") or {}).get("mode"))
    return 0


def print_summary(fname, run, mode_now):
    tm = run.get("timing_ms") or {}

    def row(k):
        x = tm.get(k) or {}
        return "{:<8} {:>6} {:>6} {:>6}".format(k, x.get("p50", "-"), x.get("p90", "-"),
                                               x.get("max", "-"))
    hz = run.get("realised_hz")
    tot90 = (tm.get("total") or {}).get("p90")
    print("")
    print("saved   data/policyruns/{}".format(fname))
    print("ended   {}  after {} s, {} ticks, {} late  -> mode now {}".format(
        run.get("exit_reason"), run.get("run_seconds"), run.get("ticks"), run.get("late_ticks"),
        mode_now))
    print("rate    {} Hz (nominal {})   worker {} (late {})".format(
        hz, run.get("nominal_hz"), run.get("worker"), run.get("worker_late_total")))
    print("timing  ms        p50    p90    max")
    for k in ("measure", "ctrl", "gov", "ws", "send", "log", "total", "rtt", "wstep"):
        if k in tm:
            print("        " + row(k))
    ok = hz is not None and hz >= 99.0 and tot90 is not None and tot90 < 10.0
    # a run cut short says nothing about holding the rate (run 9, 2026-11-20: 0.88 s, hard stop on
    # stale telemetry, and the first version of this line still printed MET)
    full = "max run time" in str(run.get("exit_reason") or "")
    if not full:
        print("criterion (>= 99 Hz and total p90 < 10 ms): NOT EVALUATED -- the run was stopped "
              "early ({}); rate so far {} Hz, total p90 {} ms".format(
                  run.get("exit_reason"), hz, tot90))
    else:
        print("criterion (>= 99 Hz and total p90 < 10 ms): {}".format("MET" if ok else "not met"))


def cmd_stop(api, a):
    st, d = api.call("/api/policy/stop", {"hard": bool(a.hard)})
    print("ok" if d.get("ok") else "!! {}".format(d.get("error")))
    return 0 if d.get("ok") else 1


def cmd_limp(api, a):
    """Release every motor (0 A). The robot hangs in the harness or drops: ask first."""
    if not a.yes:
        ans = input("Release ALL motors (LIMP)? The robot will hang in the harness. [y/N] ")
        if ans.strip().lower() not in ("y", "yes", "o", "oui"):
            die("nothing sent", 2)
    st, d = api.call("/api/mode", {"mode": "LIMP"})
    print("LIMP: {}".format("requested" if d.get("ok") else d.get("error")))
    return 0 if d.get("ok") else 1


def cmd_estop(api, a):
    st, d = api.call("/api/estop", {})
    print("E-STOP: {}".format("latched" if d.get("ok") else d.get("error")))
    return 0 if d.get("ok") else 1


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__.split("\n\n", 1)[1])
    ap.add_argument("--url", default=default_url(), help="daemon URL (default %(default)s)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("pose", help="MANUAL + slow move to the bundle's start pose")
    p.add_argument("file")
    p.add_argument("--slew", type=float, default=20.0, help="deg/s (default %(default)s)")
    p.add_argument("--takeover", action="store_true")
    r = sub.add_parser("run", help="arm, hold the dead-man, save the log")
    r.add_argument("file")
    r.add_argument("--seconds", type=float, default=MAX_SECONDS,
                   help="run length, at most {:.0f} (default %(default)s)".format(MAX_SECONDS))
    r.add_argument("--allow-uncalibrated-thermal", action="store_true",
                   help="the panel's 'run with PLACEHOLDER thermal parameters' box")
    r.add_argument("--skip-jointmap-check", action="store_true",
                   help="the panel's 'run with an UNVERIFIED joint map' box")
    r.add_argument("--hung", action="store_true", help="the robot is hung, not standing")
    r.add_argument("--no-worker", action="store_true", help='"worker": false (control law in the daemon)')
    r.add_argument("--yes", action="store_true", help="do not ask for the harness confirmation")
    r.add_argument("--takeover", action="store_true")
    s = sub.add_parser("stop", help="end a running policy")
    s.add_argument("--hard", action="store_true")
    lp = sub.add_parser("limp", help="release every motor (0 A)")
    lp.add_argument("--yes", action="store_true", help="do not ask for confirmation")
    sub.add_parser("estop", help="E-STOP")
    sub.add_parser("status", help="mode, pose, policy")
    a = ap.parse_args(argv)
    api = Api(a.url)
    try:
        return {"pose": cmd_pose, "run": cmd_run, "stop": cmd_stop, "estop": cmd_estop,
                "limp": cmd_limp,
                "status": cmd_status}[a.cmd](api, a)
    except (urllib.error.URLError, OSError) as e:
        die("cannot reach the daemon at {}: {}".format(a.url, e))


if __name__ == "__main__":
    sys.exit(main())
