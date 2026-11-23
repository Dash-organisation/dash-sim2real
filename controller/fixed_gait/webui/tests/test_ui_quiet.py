"""The UI goes quiet while a policy is armed or running (2026-11-17).

Run 3 of standing04_xshape: the control law takes 3.5 ms alone on the Pi and 9.2 ms inside the
daemon, whose interpreter also serves the 20 Hz stream to uiproc (a full snapshot + telemetry +
sensors per frame) -- the loop ran at 60 Hz instead of 100 and the governor stopped the run. While
a policy is armed or running the stream drops to STREAM_HZ_POLICY; the browser side (app.js
uiQuiet) stops polling telemetry and sensors."""
import types

import pytest

import daemon as daemon_mod
import server


def _d(mode="MANUAL", pol=None, arming=False):
    return types.SimpleNamespace(mode=mode, _pol=pol, _pol_arming=arming)


@pytest.mark.parametrize("state, quiet", [
    (dict(), False),
    (dict(mode="POLICY"), True),
    (dict(arming=True), True),
    (dict(pol={"phase": "hold"}), True),
    (dict(pol={"phase": "run"}), True),
    (dict(pol={"phase": "done"}), False),        # the SET_POS hold after a run: full rate again
    (dict(mode="LIMP", pol={"phase": "done"}), False),
])
def test_quiet_while_armed_or_running(state, quiet):
    assert daemon_mod.RobotDaemon.policy_quiet_ui(_d(**state)) is quiet


def test_the_stream_slows_down_during_a_policy_run(monkeypatch):
    monkeypatch.setitem(server.STATE, "daemon", types.SimpleNamespace(policy_quiet_ui=lambda: False))
    assert server._stream_period() == pytest.approx(1.0 / server.STREAM_HZ)
    monkeypatch.setitem(server.STATE, "daemon", types.SimpleNamespace(policy_quiet_ui=lambda: True))
    assert server._stream_period() == pytest.approx(1.0 / server.STREAM_HZ_POLICY)
    assert server.STREAM_HZ_POLICY < server.STREAM_HZ


def test_a_broken_quiet_check_never_breaks_the_stream(monkeypatch):
    def boom():
        raise RuntimeError("x")
    monkeypatch.setitem(server.STATE, "daemon", types.SimpleNamespace(policy_quiet_ui=boom))
    assert server._stream_period() == pytest.approx(1.0 / server.STREAM_HZ)
    monkeypatch.setitem(server.STATE, "daemon", None)
    assert server._stream_period() == pytest.approx(1.0 / server.STREAM_HZ)


def test_arming_is_quiet_and_the_flag_always_clears():
    seen = []
    d = _d()

    def arm(spec):
        seen.append(daemon_mod.RobotDaemon.policy_quiet_ui(d))
        if spec == "boom":
            raise RuntimeError("refused loudly")
        return True, "", {}
    d._policy_arm = arm
    assert daemon_mod.RobotDaemon.policy_arm(d, {})[0]
    with pytest.raises(RuntimeError):
        daemon_mod.RobotDaemon.policy_arm(d, "boom")
    assert seen == [True, True]
    assert d._pol_arming is False and not daemon_mod.RobotDaemon.policy_quiet_ui(d)
