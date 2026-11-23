"""tools/policy_cli.py against a fake daemon that speaks the same HTTP API (no robot, no Flask)."""
import json
import os
import signal
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(HERE))), "tools")
sys.path.insert(0, TOOLS)

import policy_cli as C                                                  # noqa: E402


class Fake:
    """The daemon's side: records every request, runs a pretend policy for `run_polls` keepalives."""

    def __init__(self, mode="MANUAL", run_polls=6, arm_status=200, arm_body=None):
        self.mode, self.run_polls = mode, run_polls
        self.arm_status, self.arm_body = arm_status, arm_body
        self.calls = []
        self.keepalives = 0
        self.armed = False
        self.stopped = []
        self.cam_pos = 53.05            # within POSE_TOL_DEG of the pose target below
        self.exit_reason = "max run time 10 s reached"

    def snapshot(self):
        pol = None
        if self.armed:
            done = self.keepalives >= self.run_polls or bool(self.stopped)
            pol = {"file": "b.npz", "phase": "done" if done else "run", "running": not done,
                   "elapsed_s": 0.1 * self.keepalives, "rate_hz": 99.5}
        # like the real daemon: `homing` stays up for as long as the pose is HELD
        return {"mode": self.mode, "manual": {"homing": True, "homing_kind": "policy"},
                "motors": {"right.cam": {"pos_norm": self.cam_pos}}, "policy": pol,
                "estop": {"latched": False}, "loop": {}, "can_errors": {}}

    def handle(self, method, path, body):
        self.calls.append((method, path, body))
        if path == "/api/state":
            return 200, self.snapshot()
        if path == "/api/policy/arm":
            if self.arm_status != 200:
                return self.arm_status, self.arm_body
            self.armed = True
            self.mode = "POLICY"
            return 200, {"ok": True, "token": "tok123", "armed": {}, "state": self.snapshot()}
        if path == "/api/policy/keepalive":
            self.keepalives += 1
            return 200, {"ok": True, "state": self.snapshot()}
        if path == "/api/policy/stop":
            self.stopped.append(body)
            return 200, {"ok": True, "state": self.snapshot()}
        if path == "/api/policy/run/save":
            run = {"exit_reason": self.exit_reason, "run_seconds": 10.0, "ticks": 1000,
                   "late_ticks": 3, "realised_hz": 99.7, "nominal_hz": 100.0, "worker": True,
                   "worker_late_total": 0,
                   "timing_ms": {"total": {"p50": 7.1, "p90": 8.9, "max": 11.0}}}
            return 200, {"ok": True, "file": "policyrun_x.npz", "run": run,
                         "state": {"mode": "MANUAL"}}
        if path == "/api/mode":
            self.mode = body["mode"]
            return 200, {"ok": True, "state": self.snapshot()}
        if path == "/api/manual/policy_pose":
            return 200, {"ok": True, "token": "tok-pose", "targets": {"right.cam": 53.0}}
        return 404, {"ok": False, "error": "no route"}


@pytest.fixture
def daemon(tmp_path, monkeypatch):
    monkeypatch.setattr(C, "TOKEN_FILE", str(tmp_path / "token"))
    monkeypatch.setattr(C, "KEEPALIVE_S", 0.02)
    monkeypatch.setattr(C, "APPEAR_GRACE_S", 0.0)
    monkeypatch.setattr(C, "pin_off_daemon_cores", lambda: None)
    holder = {}

    class H(BaseHTTPRequestHandler):
        def _go(self, method):
            n = int(self.headers.get("Content-Length") or 0)
            body = json.loads(self.rfile.read(n)) if n else None
            st, out = holder["fake"].handle(method, self.path, body)
            raw = json.dumps(out).encode()
            self.send_response(st)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def do_GET(self):
            self._go("GET")

        def do_POST(self):
            self._go("POST")

        def log_message(self, *a):
            pass

    srv = ThreadingHTTPServer(("127.0.0.1", 0), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()

    def make(**kw):
        holder["fake"] = Fake(**kw)
        return holder["fake"], "http://127.0.0.1:{}".format(srv.server_address[1])
    yield make
    srv.shutdown()


def test_a_full_run_arms_holds_the_dead_man_and_saves(daemon, capsys):
    fake, url = daemon(run_polls=6)
    rc = C.main(["--url", url, "run", "b.npz", "--allow-uncalibrated-thermal", "--yes"])
    assert rc == 0
    arm = [b for m, p, b in fake.calls if p == "/api/policy/arm"][0]
    assert arm["file"] == "b.npz" and arm["supported"] is True
    assert arm["max_seconds"] == 10.0 and arm["allow_uncalibrated_thermal"] is True
    assert "worker" not in arm and "skip_jointmap_check" not in arm
    assert fake.keepalives >= 6
    assert ("POST", "/api/policy/run/save", {}) in fake.calls
    out = capsys.readouterr().out
    assert "policyrun_x.npz" in out and "criterion (>= 99 Hz and total p90 < 10 ms): MET" in out


def test_never_more_than_ten_seconds(daemon):
    fake, url = daemon()
    with pytest.raises(SystemExit):
        C.main(["--url", url, "run", "b.npz", "--seconds", "10.5", "--yes"])
    assert not any(p == "/api/policy/arm" for _, p, _ in fake.calls)
    assert C.MAX_SECONDS == 10.0


def test_a_shorter_run_and_no_worker_are_passed_through(daemon):
    fake, url = daemon(run_polls=2)
    C.main(["--url", url, "run", "b.npz", "--seconds", "3", "--no-worker", "--yes"])
    arm = [b for m, p, b in fake.calls if p == "/api/policy/arm"][0]
    assert arm["max_seconds"] == 3.0 and arm["worker"] is False


def test_refuses_to_arm_outside_manual(daemon):
    fake, url = daemon(mode="LIMP")
    with pytest.raises(SystemExit):
        C.main(["--url", url, "run", "b.npz", "--yes"])
    assert not any(p == "/api/policy/arm" for _, p, _ in fake.calls)


def test_no_harness_confirmation_no_arm(daemon, monkeypatch):
    fake, url = daemon()
    monkeypatch.setattr("builtins.input", lambda prompt: "n")
    with pytest.raises(SystemExit):
        C.main(["--url", url, "run", "b.npz"])
    assert not any(p == "/api/policy/arm" for _, p, _ in fake.calls)


def test_the_daemon_refusal_is_shown(daemon, capsys):
    fake, url = daemon(arm_status=400, arm_body={"ok": False, "error": "thermal model uncalibrated"})
    with pytest.raises(SystemExit):
        C.main(["--url", url, "run", "b.npz", "--yes"])
    assert "thermal model uncalibrated" in capsys.readouterr().out


def test_ctrl_c_is_a_soft_stop_then_a_hard_one(daemon):
    fake, url = daemon(run_polls=10_000)

    def press_twice():
        time.sleep(0.15)
        os.kill(os.getpid(), signal.SIGINT)
        time.sleep(0.15)
        os.kill(os.getpid(), signal.SIGINT)
    fake.stopped = []
    real = fake.snapshot

    def snap():                                  # keep running through the soft stop
        s = real()
        if s["policy"] and len(fake.stopped) < 2:
            s["policy"]["running"], s["policy"]["phase"] = True, "run"
        return s
    fake.snapshot = snap
    threading.Thread(target=press_twice, daemon=True).start()
    rc = C.main(["--url", url, "run", "b.npz", "--yes"])
    assert rc == 0
    assert fake.stopped[0] == {} and fake.stopped[1] == {"hard": True}


def test_pose_keeps_the_token_for_run(daemon):
    fake, url = daemon(run_polls=1)
    assert C.main(["--url", url, "pose", "b.npz"]) == 0
    pose = [b for m, p, b in fake.calls if p == "/api/manual/policy_pose"][0]
    assert pose["file"] == "b.npz" and pose["slew_dps"] == 20.0
    C.main(["--url", url, "run", "b.npz", "--yes"])
    arm = [b for m, p, b in fake.calls if p == "/api/policy/arm"][0]
    assert arm["token"] == "tok-pose"


# ===================================================================== end to end
# The real Flask app, served on a local port, over a calibrated MockBus daemon and the real bundle:
# the script's requests and its reading of the replies against the daemon's actual API.
from test_blackbox import capture_zero, robot                          # noqa: E402,F401
from test_policy_run import armed, needs_bundle, staged                 # noqa: E402,F401


@pytest.fixture
def served(staged, tmp_path, monkeypatch):                              # noqa: F811
    import types
    from werkzeug.serving import make_server
    import server
    d, cal, fname = staged
    monkeypatch.setattr(C, "TOKEN_FILE", str(tmp_path / "token"))
    monkeypatch.setattr(C, "pin_off_daemon_cores", lambda: None)
    keys = ("daemon", "calib", "mock", "fk", "sense", "wstore", "dyn", "ctl")
    saved = {k: server.STATE[k] for k in keys}
    server.STATE.update(daemon=d, calib=cal, mock=True, ctl={"token": None, "ts": 0.0},
                        fk=types.SimpleNamespace(available=False, try_reload=lambda: None,
                                                 side_verified=lambda side: False),
                        wstore=types.SimpleNamespace(legs={"left": {}, "right": {}},
                                                     source="workspace_test.npz",
                                                     list_files=lambda: []),
                        dyn=types.SimpleNamespace(snapshot=lambda: {}), sense=d.sense)
    monkeypatch.setattr(paths_mod(), "POLICYRUN_DIR", str(tmp_path))
    srv = make_server("127.0.0.1", 0, server.app, threaded=True)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield d, fname, "http://127.0.0.1:{}".format(srv.server_port), tmp_path
    srv.shutdown()
    server.STATE.update(saved)


def paths_mod():
    import paths
    return paths


@needs_bundle
def test_end_to_end_a_hung_run_on_the_mock_daemon(served, capsys):
    d, fname, url, out_dir = served
    rc = C.main(["--url", url, "run", fname, "--hung", "--seconds", "2", "--yes",
                 "--allow-uncalibrated-thermal", "--skip-jointmap-check"])
    out = capsys.readouterr().out
    assert rc == 0, out
    assert "ARMED" in out and "saved   data/policyruns/policyrun_" in out
    saved = [f for f in os.listdir(out_dir) if f.startswith("policyrun_")]
    assert len(saved) == 1
    # normally "max run time 2 s reached"; on a loaded machine the rate guard may end it first --
    # what is under test is the script's sequence, not the mock's timing
    assert "\nended   " in out, out
    assert "timing  ms" in out and "criterion (>= 99 Hz and total p90 < 10 ms)" in out


def test_pose_arrives_on_the_measured_error_although_homing_stays_up(daemon, capsys):
    """Seen on the robot (2026-11-20): every joint at +-0.1 deg, `homing` still True -- the daemon
    holds the pose under that flag. Arrival must come from the error, not from the flag."""
    fake, url = daemon()
    t = time.time()
    assert C.main(["--url", url, "pose", "b.npz"]) == 0
    assert time.time() - t < 3.0                          # 3 polls at 0.5 s, not a timeout
    assert "arrived" in capsys.readouterr().out


def test_pose_still_far_is_not_arrived(daemon, monkeypatch):
    fake, url = daemon()
    fake.cam_pos = 50.0                                   # 3 deg off
    monkeypatch.setattr(C, "POSE_TIMEOUT_S", 1.6)
    with pytest.raises(SystemExit):
        C.main(["--url", url, "pose", "b.npz"])


def test_run_does_not_wait_for_the_homing_flag(daemon):
    fake, url = daemon(run_polls=1)                       # homing True, as on the robot
    assert C.main(["--url", url, "run", "b.npz", "--yes"]) == 0
    assert any(p == "/api/policy/arm" for _, p, _ in fake.calls)


def test_a_run_stopped_early_is_not_called_a_success(daemon, capsys):
    """Run 9 (2026-11-20): 0.88 s, hard stop on stale telemetry -- and the summary said MET."""
    fake, url = daemon(run_polls=1)
    fake.exit_reason = "telemetry stale by 50 ms -- never command a joint we cannot see"
    C.main(["--url", url, "run", "b.npz", "--yes"])
    out = capsys.readouterr().out
    assert "NOT EVALUATED" in out and "MET" not in out.replace("NOT EVALUATED", "")


def test_limp_asks_then_releases(daemon, monkeypatch):
    fake, url = daemon()
    monkeypatch.setattr("builtins.input", lambda prompt: "n")
    with pytest.raises(SystemExit):
        C.main(["--url", url, "limp"])
    assert fake.mode == "MANUAL"
    assert C.main(["--url", url, "limp", "--yes"]) == 0
    assert ("POST", "/api/mode", {"mode": "LIMP"}) in fake.calls and fake.mode == "LIMP"
