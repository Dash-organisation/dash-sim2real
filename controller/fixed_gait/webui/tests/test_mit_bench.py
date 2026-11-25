"""tools/mit_bench.py (free-cam MIT diagnostic): every guard releases, offline.

A scripted FakeBus plays the drive: status frames at 200 Hz, and a `fault` that changes them once
the bench has been sending a real (non-limp) MIT frame for a while. Each test then checks the
sequence the bench sent AFTER the fault: limp_payload frames for the release time, then
SET_CURRENT 0 -- never SET_CURRENT 0 alone (it does not release a force frame).
"""
import copy
import math
import os
import struct
import sys
import time

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(HERE))), "tools")
if TOOLS not in sys.path:
    sys.path.insert(0, TOOLS)

import mit_bench as B                                                  # noqa: E402

canio, mit = B.canio, B.mit
CID = 105
LIMP = bytes(mit.limp_payload())
POS0 = 123.4


class _Frame:
    def __init__(self, arb, data):
        self.arbitration_id, self.data, self.is_extended_id = arb, data, True


class FakeBus:
    """status(t) -> dict(pos, spd, cur, temp, err); `fault(bus, t_mit)` overrides it once real MIT
    frames have been streaming for t_mit seconds, or returns "silent" to stop the status frames."""
    channel = "fake"

    def __init__(self, fault=None, status_hz=200.0, foreign=False, raise_at=None):
        self.fault, self.dt = fault, 1.0 / status_hz
        self.t0 = time.monotonic()
        self.next_status = self.t0
        self.sent = []                       # (t, cmd, data)
        self.mit_since = None
        self.foreign = foreign
        self.raise_at = raise_at             # t_mit at which recv raises once
        self.raised = False
        self.state = {"pos": POS0, "spd": 0.0, "cur": 0.0, "temp": 35, "err": 0}

    def send(self, msg):
        now = time.monotonic()
        cmd = (msg.arbitration_id >> 8) & 0xFF
        data = bytes(msg.data)
        self.sent.append((now, cmd, data))
        if cmd == 8 and data != LIMP and self.mit_since is None:
            self.mit_since = now           # the fault clock; release frames do not reset it

    def _status(self, now):
        st = dict(self.state)
        if self.fault is not None and self.mit_since is not None:
            o = self.fault(self, now - self.mit_since)
            if o == "silent":
                return None
            if o:
                st.update(o)
        data = (struct.pack(">h", int(round(st["pos"] * 10))) +
                struct.pack(">h", int(round(st["spd"] / 10.0))) +
                struct.pack(">h", int(round(st["cur"] * 100))) +
                struct.pack(">b", int(st["temp"])) + bytes([st["err"]]))
        return _Frame(0x2900 | CID, data)

    def recv(self, timeout=0.0):
        now = time.monotonic()
        if (self.raise_at is not None and not self.raised and self.mit_since is not None
                and now - self.mit_since > self.raise_at):
            self.raised = True
            raise RuntimeError("injected")
        if self.foreign and now - self.t0 > 0.05:
            self.foreign = False
            return _Frame((1 << 8) | 104, b"\0\0\0\0")       # somebody's SET_CURRENT
        if now < self.next_status:
            wait = min(self.next_status - now, timeout or 0.0)
            if wait <= 0:
                return None
            time.sleep(wait)
            now = time.monotonic()
            if now < self.next_status:
                return None
        self.next_status += self.dt
        if self.next_status < now:
            self.next_status = now + self.dt
        return self._status(now)

    def shutdown(self):
        pass


MIT_STEP = B._step("T1", "mit", "test", [("still", 1.0, "")], kp=5.0, kd=0.5)


def run(bus, steps=(MIT_STEP,), release_s=0.2, **kw):
    lines = []
    bench = B.Bench(bus, CID, list(steps), release_s=release_s, release_current_s=0.1,
                    pause_s=0.0, out=lines.append, beep=False, **kw)
    return bench, bench.run(), lines


@pytest.fixture(autouse=True)
def short_sync(monkeypatch):
    """SYNC lasts 5 s on the robot; 0.2 s here."""
    s = copy.deepcopy(B.SYNC_STEP)
    s["subs"][0][1] = 0.2
    monkeypatch.setattr(B, "SYNC_STEP", s)


def after_mit(bus):
    """Frames sent after the last real MIT frame."""
    last = max(i for i, (_, c, d) in enumerate(bus.sent) if c == 8 and d != LIMP)
    return bus.sent[last + 1:]


def assert_released(bus, release_s=0.2):
    tail = after_mit(bus)
    assert tail, "nothing sent after the last MIT frame"
    # the release: an unbroken run of limp payloads first ...
    n = 0
    while n < len(tail) and tail[n][1] == 8 and tail[n][2] == LIMP:
        n += 1
    assert n >= 2, "release must start with limp_payload frames, got cmd {}".format(tail[0][1])
    span = tail[n - 1][0] - tail[0][0]
    assert span >= release_s - 2 * 0.011, "limp streamed {:.3f} s < {}".format(span, release_s)
    # ... then SET_CURRENT 0
    assert n < len(tail) and tail[n][1] == 1
    assert struct.unpack(">i", tail[n][2])[0] == 0
    # nothing after the release ever re-engages MIT
    assert all(c != 8 or d == LIMP for _, c, d in tail)


def outcome(table, sid="T1"):
    return next(r for r in table if r["id"] == sid)["outcome"]


# ------------------------------------------------------------------ the plan
def test_full_phase_a_plan_runs_on_mockbus():
    steps = copy.deepcopy(B.PHASE_A)
    for s in steps:
        for sub in s["subs"]:
            sub[1] = 0.15
    bus = canio.MockBus("can1", seed=3)
    bench, table, _ = run(bus, steps, release_s=0.05)
    assert [r["id"] for r in table] == ["SYNC"] + [s["id"] for s in B.PHASE_A]
    assert all(r["outcome"] == "ok" for r in table), [(r["id"], r["outcome"]) for r in table]
    assert bench.tx["mit"] > 0 and bench.tx["servo_rpm"] > 0 and bench.tx["servo_pos"] > 0
    assert all(r["rx_hz"] for r in table)


def test_select_steps_prefixes_and_unknown():
    ids = [s["id"] for s in B.select_steps(B.PHASE_A, "A2,A4b")]
    assert ids == ["A2a", "A2b", "A2c", "A2d", "A2e", "A2f", "A2g", "A4b"]
    with pytest.raises(ValueError):
        B.select_steps(B.PHASE_A, "A9")


def test_plan_constraints():
    for s in B.PHASE_A:
        assert s["kp"] <= B.KP_LIMIT and s["kd"] <= B.KD_LIMIT
    a4a = next(s for s in B.PHASE_A if s["id"] == "A4a")
    assert B.speed_limit(a4a) == pytest.approx(3.0)
    assert abs(B.ERPM_PER_RADS - 1604.2) < 0.1


# ------------------------------------------------------------------ the command
def test_mit_target_is_the_measured_position_and_frames_are_8_bytes(monkeypatch):
    calls = []
    real = mit.pack

    def spy(*a, **k):
        calls.append((a, k))
        return real(*a, **k)
    monkeypatch.setattr(mit, "pack", spy)
    bus = FakeBus()
    _, table, _ = run(bus)
    assert outcome(table) == "ok"
    engaged = [c for c in calls if c[0][1] > 0]            # not the limp_payload pack(0, 0, 0)
    assert engaged and all(c[0][0] == pytest.approx(math.radians(POS0)) for c in engaged)
    mits = [d for _, c, d in bus.sent if c == 8]
    assert mits and all(len(d) == 8 for d in mits)
    real_frames = [d for d in mits if d != LIMP]
    p = mit.unpack(real_frames[0])
    assert p["p_des"] == pytest.approx(math.radians(POS0), abs=mit.resolution()["position_rad"])
    assert p["kp"] == pytest.approx(5.0, abs=0.2) and p["v_des"] == 0.0 and p["tau_ff"] == 0.0
    assert_released(bus)


def test_velocity_step_uses_the_manual_v_range(monkeypatch):
    calls = []
    real = mit.pack

    def spy(*a, **k):
        calls.append(k)
        return real(*a, **k)
    monkeypatch.setattr(mit, "pack", spy)
    step = B._step("T1", "mit", "", [("still", 0.3, "")], kd=1.0, v_des=1.0)
    _, table, _ = run(FakeBus(), [step])
    assert outcome(table) == "ok"
    vel = [k for k in calls if k.get("v_des")]
    assert vel and vel[0]["v_range"] == (-20.0, 20.0) and "tau_ff" not in vel[0]


def test_gains_above_the_limits_are_refused():
    bus = FakeBus()
    step = B._step("T1", "mit", "", [("still", 0.3, "")], kp=250.0, kd=1.0)
    _, table, _ = run(bus, [step])
    assert outcome(table).startswith("refused:")
    assert not any(c == 8 and d != LIMP for _, c, d in bus.sent)


def test_position_outside_the_mit_frame_is_refused():
    bus = FakeBus()
    bus.state["pos"] = 800.0
    _, table, _ = run(bus)
    assert outcome(table).startswith("refused:")
    assert not any(c == 8 and d != LIMP for _, c, d in bus.sent)


def test_set_rpm_frame():
    bus = FakeBus()
    B.set_rpm(bus, CID, 1604)
    _, cmd, data = bus.sent[-1]
    assert cmd == 3 and struct.unpack(">i", data)[0] == 1604


# ------------------------------------------------------------------ every guard releases
@pytest.mark.parametrize("fault,expected", [
    (lambda b, t: {"spd": 6000.0} if t > 0.2 else None, "abort:speed"),
    (lambda b, t: {"cur": 16.0} if t > 0.2 else None, "abort:current"),
    (lambda b, t: {"temp": 61} if t > 0.2 else None, "abort:temp"),
    (lambda b, t: {"err": 3} if t > 0.2 else None, "abort:error"),
    (lambda b, t: "silent" if t > 0.2 else None, "abort:silent"),
])
def test_guard_trips_and_releases(fault, expected):
    bus = FakeBus(fault=fault)
    t_steps = time.monotonic()
    _, table, _ = run(bus)
    assert outcome(table) == expected
    assert_released(bus)
    assert time.monotonic() - t_steps < 3.0


def test_runaway_on_current_ripple():
    def fault(b, t):
        if t > 0.1:
            b._k = getattr(b, "_k", 0) + 1
            return {"cur": 2.0 + (5.0 if b._k % 2 else -5.0)}
    bus = FakeBus(fault=fault)
    step = B._step("T1", "mit", "", [("still", 2.0, "")], kp=50.0, kd=1.0)
    _, table, _ = run(bus, [step])
    row = table[-1]
    assert row["outcome"] == "runaway"
    assert row["detail"]["trigger"] == "iq_std"
    assert row["detail"]["iq_mean"] == pytest.approx(2.0, abs=1.0)
    assert row["t_end"] - row["t_start"] < 1.0
    assert_released(bus)


def test_runaway_on_speed_sign_flips():
    def fault(b, t):
        if t > 0.1:
            b._k = getattr(b, "_k", 0) + 1
            return {"spd": 500.0 if b._k % 2 else -500.0}
    bus = FakeBus(fault=fault)
    step = B._step("T1", "mit", "", [("still", 2.0, "")], kp=50.0, kd=1.0)
    _, table, _ = run(bus, [step])
    assert table[-1]["outcome"] == "runaway"
    assert table[-1]["detail"]["trigger"] == "v_flip"
    assert_released(bus)


def test_speed_noise_at_rest_is_not_a_runaway():
    def fault(b, t):
        b._k = getattr(b, "_k", 0) + 1
        return {"spd": 10.0 if b._k % 2 else -10.0}        # +-1 LSB dither
    _, table, _ = run(FakeBus(fault=fault))
    assert outcome(table) == "ok"


def test_runaway_moves_on_to_the_next_step():
    def fault(b, t):
        mit_now = b.sent and b.sent[-1][1] == 8 and b.sent[-1][2] != LIMP
        if mit_now and t > 0.1:
            b._k = getattr(b, "_k", 0) + 1
            return {"cur": 5.0 if b._k % 2 else -5.0}
    bus = FakeBus(fault=fault)
    s1 = B._step("T1", "mit", "", [("still", 2.0, "")], kp=50.0, kd=1.0)
    s2 = B._step("T2", "current", "", [("still", 0.2, "")])
    bench, table, _ = run(bus, [s1, s2])
    assert [r["outcome"] for r in table] == ["ok", "runaway", "ok"]
    assert bench.stop_reason is None


def test_temperature_stops_the_session():
    bus = FakeBus(fault=lambda b, t: {"temp": 61} if t > 0.1 else None)
    s2 = B._step("T2", "current", "", [("still", 0.2, "")])
    bench, table, _ = run(bus, [MIT_STEP, s2])
    assert [r["id"] for r in table] == ["SYNC", "T1"]
    assert bench.stop_reason == "abort:temp"


def test_ctrl_c_once_ends_the_step_twice_stops_all():
    intr = B.Interrupts()

    def fault(b, t):
        if t > 0.2 and not getattr(b, "_pressed", False):
            b._pressed = True
            intr.count = 1
    bus = FakeBus(fault=fault)
    s2 = B._step("T2", "current", "", [("still", 0.2, "")])
    bench, table, _ = run(bus, [MIT_STEP, s2], interrupts=intr)
    assert [r["outcome"] for r in table] == ["ok", "abort:operator", "ok"]
    assert_released(bus)

    intr2 = B.Interrupts()

    def fault2(b, t):
        if t > 0.2:
            intr2.count = 2
    bus = FakeBus(fault=fault2)
    bench, table, _ = run(bus, [MIT_STEP, s2], interrupts=intr2)
    assert [r["id"] for r in table] == ["SYNC", "T1"]
    assert bench.stop_reason.startswith("operator")
    assert_released(bus)


def test_exception_releases_then_raises():
    bus = FakeBus(raise_at=0.2)
    with pytest.raises(RuntimeError):
        run(bus)
    assert_released(bus)
    # released once, not twice
    tail = after_mit(bus)
    starts = [i for i in range(1, len(tail)) if tail[i][2] == LIMP and tail[i - 1][1] == 1]
    assert not starts


def test_release_streams_limp_for_one_second_then_current_zero():
    bus = FakeBus(fault=lambda b, t: {"spd": 6000.0} if t > 0.1 else None)
    _, table, _ = run(bus, release_s=1.0)
    assert outcome(table) == "abort:speed"
    assert_released(bus, release_s=1.0)
    tail = after_mit(bus)
    n_limp = sum(1 for _, c, d in tail if c == 8 and d == LIMP)
    assert 90 <= n_limp <= 105                      # 1 s at 100 Hz


def test_foreign_commands_refuse_to_start():
    bus = FakeBus(foreign=True)
    bench, table, _ = run(bus)
    assert bench.stop_reason.startswith("another program")
    assert not any(c == 8 and d != LIMP for _, c, d in bus.sent)


# ------------------------------------------------------------------ CLI
def test_dry_run_never_opens_the_bus(monkeypatch, capsys):
    def boom(*a, **k):
        raise AssertionError("bus opened")
    monkeypatch.setattr(canio, "open_buses", boom)
    assert B.main(["--dry-run", "--phase", "A"]) == 0
    assert "dry run" in capsys.readouterr().out


def test_save_roundtrip(tmp_path):
    import json
    import numpy as np
    bench, table, _ = run(FakeBus())
    p = bench.save(str(tmp_path / "s.npz"))
    z = np.load(p, allow_pickle=False)
    meta = json.loads(str(z["meta_json"]))
    assert z["frames"].shape[1] == len(meta["columns"])
    steps = json.loads(str(z["steps_json"]))
    assert [s["id"] for s in steps] == ["SYNC", "T1"]
    assert json.loads(str(z["journal_json"]))[0]["event"] == "preflight"
