"""MANUAL hold law (daemon.hold_settings, /api/manual/hold): SET_POS or the MIT force frame at the
operator's kp/kd per joint role.

The safety-relevant parts: off by default; gains clipped; the MIT frame carries exactly the
operator's gains; going LIMP streams the zero-gain release; and nothing that commands in another
law (switching back to SET_POS, playback & co.) is allowed straight out of an MIT hold, because it
has not been shown on these drives that a servo command supersedes a force frame.

MockBus only -- no hardware.

    python -m pytest controller/fixed_gait/webui/tests/test_hold_law.py -v
"""
import os
import sys
import time

import pytest

import canio
import daemon as daemon_mod
import paths
from test_blackbox import capture_zero, robot, wait_mode      # noqa: F401  (pytest fixtures)
from test_thermal_api import client                           # noqa: F401  (the wired test client)

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", "deploy"))
import mit                                                      # noqa: E402


@pytest.fixture
def frames(monkeypatch):
    """Every force-control and SET_POS frame the daemon sends, as (kind, cid, decoded)."""
    log = []
    fc, sp = canio.force_control, canio.set_pos

    def force_control(bus, cid, payload):
        log.append(("mit", cid, mit.unpack(payload)))
        return fc(bus, cid, payload)

    def set_pos(bus, cid, raw):
        log.append(("pos", cid, raw))
        return sp(bus, cid, raw)
    monkeypatch.setattr(canio, "force_control", force_control)
    monkeypatch.setattr(canio, "set_pos", set_pos)
    return log


def _manual(d, cal):
    capture_zero(d, cal)
    with d.lock:
        d._manual_targets = {}
        d._req_mode = "MANUAL"
    assert wait_mode(d, "MANUAL")


def test_off_by_default_and_gains_are_clipped(robot):
    d, _cal, _b, _dir = robot
    with d.lock:
        assert d._hold_mit is False
    out, why = d.hold_settings(gains={"cam": {"kp": 9999, "kd": -3}})
    assert out is not None, why
    assert out["gains"]["cam"] == {"kp": daemon_mod.HOLD_KP_MAX, "kd": 0.0}
    assert out["mit"] is False


@pytest.mark.parametrize("bad", [{"knee": {"kp": 1}}, {"cam": {"ki": 1}}, {"cam": {"kp": "x"}},
                                 {"cam": {"kp": float("nan")}}])
def test_bad_gains_are_refused_and_change_nothing(robot, bad):
    d, _cal, _b, _dir = robot
    before = d._hold_snapshot()["gains"]
    out, why = d.hold_settings(gains=bad)
    assert out is None and why
    assert d._hold_snapshot()["gains"] == before


def test_servo_hold_sends_set_pos_only(robot, frames):
    d, cal, _b, _dir = robot
    _manual(d, cal)
    time.sleep(0.2)
    kinds = {k for k, _c, _x in frames}
    assert "pos" in kinds and "mit" not in kinds


def test_mit_hold_sends_the_operator_gains_then_limp_releases_them(robot, frames):
    d, cal, _b, _dir = robot
    out, why = d.hold_settings(mit=True, gains={"abd": {"kp": 60, "kd": 1.0},
                                                "cam": {"kp": 150, "kd": 3.0},
                                                "thigh": {"kp": 200, "kd": 4.0}})
    assert out is not None, why
    _manual(d, cal)
    time.sleep(0.6)                                   # through hold-before-move into the hold
    assert d.get_snapshot()["mode"] == "MANUAL", d.get_snapshot().get("estop")
    mine = [(c, x) for k, c, x in frames if k == "mit"]
    assert mine, "no force frame was sent"
    assert not [1 for k, _c, _x in frames[-30:] if k == "pos"], "still sending SET_POS"
    want = {"abd": (60, 1.0), "cam": (150, 3.0), "thigh": (200, 4.0)}
    for n, m in d.by_name.items():
        role = paths.split_name(n)[1]
        last = [x for c, x in mine if c == m.cid][-1]
        assert last["kp"] == pytest.approx(want[role][0], abs=0.2)     # 12-bit wire LSB 0.12
        assert last["kd"] == pytest.approx(want[role][1], abs=0.01)
    assert d.get_snapshot()["manual"]["hold_mit_engaged"] is True

    # switching back to SET_POS mid-hold is refused; LIMP first
    out, why = d.hold_settings(mit=False)
    assert out is None and "LIMP" in why
    ok, why = d._mit_hold_guard()
    assert not ok and "LIMP" in why

    frames.clear()
    with d.lock:
        d._req_mode = "LIMP"
    assert wait_mode(d, "LIMP")
    time.sleep(0.2)
    releases = [x for k, _c, x in frames if k == "mit"]
    assert releases and all(x["kp"] == 0.0 and x["kd"] == 0.0 for x in releases), \
        "LIMP must stream the zero-gain force frame after an MIT hold"
    assert d._mit_engaged is False
    out, why = d.hold_settings(mit=False)
    assert out is not None and out["mit"] is False, why


def test_a_raw_target_outside_the_mit_frame_falls_back_to_set_pos(robot, frames):
    d, _cal, _b, _dir = robot
    d.hold_settings(mit=True)
    m = d.by_name["left.cam"]
    note = d._send_hold("left.cam", m, 900.0)        # 900 deg > the +-720 deg position span
    assert "SET_POS" in note
    assert frames[-1][0] == "pos" and frames[-1][2] == 900.0


def test_endpoint(client):                                    # noqa: F811
    c, _d = client
    r = c.post("/api/manual/hold", json={"gains": {"thigh": {"kp": 120}}})
    assert r.status_code == 200, r.get_json()
    h = r.get_json()["hold"]                     # the snapshot may predate the first loop tick
    assert h["gains"]["thigh"]["kp"] == 120.0
    assert c.post("/api/manual/hold", json={"mit": "yes"}).status_code == 400
    assert c.post("/api/manual/hold", json={"gains": {"knee": {"kp": 1}}}).status_code == 400
