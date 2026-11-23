"""A v3 policy run through the worker process, in the daemon, on MockBus (plan B, step 3).

The worker must change WHERE the control law runs and nothing else: the same commands for the same
observations, the same guards, the same ends. What it adds are stops of its own -- a late answer
re-uses the previous command and three in a row soft-stop; a dead, raising or confused worker
hard-stops on the tick it is noticed -- and every one of them has to be real.

MockBus only -- no hardware, no CAN, no robot.
"""
import os
import signal
import time

import numpy as np
import pytest

import daemon as daemon_mod
import policy_worker as PW
from test_blackbox import capture_zero, robot, wait_mode            # noqa: F401 (pytest fixtures)
from test_policy_run import (BUNDLE_SRC, armed, keep_alive_until,   # noqa: F401
                             needs_bundle, spec, staged)
from test_policy_timing import capture_events, done_event


def worker_events(seen, cause=None):
    return [f for k, f in seen if k == "policy.worker" and (cause is None or f.get("cause") == cause)]


def run_worker_of(d):
    p = d._pol
    assert p is not None and p.get("worker") is not None, "this run is not on the worker"
    return p["worker"]


def reaped(w, timeout=5.0):
    if w._reaper is not None:
        w._reaper.join(timeout)
    w.proc.join(timeout)
    return w.proc.exitcode is not None


@pytest.fixture
def roomy_deadline(monkeypatch):
    """The Mac under a full test run is not the robot: a test that is not ABOUT lateness must not
    fail on one late answer."""
    monkeypatch.setattr(daemon_mod, "POLICY_WORKER_DEADLINE_MS", 200.0)


# ===================================================================== 1. a whole run
@needs_bundle
def test_a_v3_run_goes_through_the_worker_and_the_worker_goes_away(staged, roomy_deadline):
    d, _cal, fname = staged
    seen = capture_events(d)
    ok, why, info = d.policy_arm(spec(file=fname, max_seconds=1.0))
    assert ok, why
    assert info["worker"] is True and info["bundle_version"] == 3
    assert info["worker_pid"] and info["worker_pid"] != os.getpid()
    assert wait_mode(d, "POLICY")
    p = keep_alive_until(d, lambda p: p["phase"] in ("run", "done"), timeout=30.0)
    assert p["phase"] == "run" and p["worker"] is True, p.get("exit_reason")
    assert d._pol["ctrl"] is None, "a worker run must not keep a controller it could step by mistake"
    w = run_worker_of(d)
    assert w.alive()
    p = keep_alive_until(d, lambda p: p["phase"] == "done", timeout=10.0)
    assert "max run time" in p["exit_reason"], p["exit_reason"]
    assert reaped(w), "the worker outlived its run"
    ev = done_event(seen)
    assert ev["worker"] is True and ev["worker_late_total"] == 0
    assert set(ev["timing_ms"]) == set(daemon_mod.POLICY_TIMING_COLS)
    assert ev["timing_ms"]["rtt"]["p50"] > 0.0
    assert ev["timing_ms"]["wstep"]["p50"] > 0.0, "the worker's own step time never arrived"
    assert ev["timing_ms"]["total"]["max"] >= ev["timing_ms"]["ctrl"]["max"]
    stopped = worker_events(seen, "stopped")
    assert len(stopped) == 1 and stopped[0]["late_total"] == 0
    assert [f for f in worker_events(seen, "ready")][0]["pid"] == info["worker_pid"]
    assert d.is_alive() and d.loop_error is None


# ===================================================================== 2. a slow worker
@needs_bundle
def test_a_slow_worker_freezes_the_command_then_soft_stops(staged):
    d, _cal, fname = staged
    seen = capture_events(d)
    # every step from the 20th takes 40 ms, against the 6 ms deadline
    d._policy_worker_opts = {"sleep_ms": 40, "sleep_from": 20}
    ok, why, _ = d.policy_arm(spec(file=fname, max_seconds=30.0))
    assert ok, why
    p = keep_alive_until(d, lambda p: p["phase"] == "done", timeout=30.0)
    assert p["reached_run"], p["exit_reason"]
    assert "missed its" in p["exit_reason"] and "3 ticks in a row" in p["exit_reason"], \
        p["exit_reason"]
    ev = done_event(seen)
    assert ev["worker_late_total"] >= daemon_mod.POLICY_WORKER_MAX_LATE
    assert ev["worker_late_max_run"] >= daemon_mod.POLICY_WORKER_MAX_LATE
    # a soft stop, and this MockBus robot hangs (hung branch): the run ends LIMP, as it always has
    assert ev["ended_in"] == "LIMP" and ev["branch"] == "hung"
    assert ev["worker_failed"] is None, "late is a soft stop, not a worker failure"
    late = worker_events(seen, "late")
    assert len(late) == 1
    # while late, the command was FROZEN: the logged targets repeat on the late ticks
    log = d.get_policy()["log"]
    tgt = log[:, 37:43]
    assert any(np.array_equal(tgt[i], tgt[i + 1]) for i in range(len(tgt) - 1))
    assert reaped(run_worker_of(d))


# ===================================================================== 3. a dead worker
@needs_bundle
def test_a_worker_killed_mid_run_is_a_hard_stop_on_the_next_tick(staged, roomy_deadline):
    d, _cal, fname = staged
    seen = capture_events(d)
    ok, why, _ = d.policy_arm(spec(file=fname, max_seconds=30.0))
    assert ok, why
    keep_alive_until(d, lambda p: p["phase"] == "run", timeout=30.0)
    w = run_worker_of(d)
    n_before = d._pol["n"]
    os.kill(w.proc.pid, signal.SIGKILL)
    p = keep_alive_until(d, lambda p: p["phase"] == "done", timeout=3.0)
    assert p["phase"] == "done"
    assert "policy worker dead" in p["exit_reason"], p["exit_reason"]
    assert p["ramp"] == 0.0, "a dead worker is a HARD stop: no ramp"
    assert d._pol["n"] - n_before < 30, "the stop took {} ticks".format(d._pol["n"] - n_before)
    dead = worker_events(seen, "dead")
    # (`alive` in the event may still read True: the EOF can beat waitpid to the news)
    assert len(dead) == 1 and dead[0]["where"] == "step"
    ev = done_event(seen)
    assert ev["worker_failed"] and "dead" in ev["worker_failed"]
    assert not d._last_cmd_raw, "a hard stop leaves the robot limp"
    assert d.is_alive() and d.loop_error is None


@needs_bundle
def test_an_exception_in_the_worker_stops_the_run_with_its_text(staged, roomy_deadline):
    d, _cal, fname = staged
    d._policy_worker_opts = {"raise_at": 30}
    ok, why, _ = d.policy_arm(spec(file=fname, max_seconds=30.0))
    assert ok, why
    p = keep_alive_until(d, lambda p: p["phase"] == "done", timeout=30.0)
    assert p["reached_run"]
    assert "raised at step 30" in p["exit_reason"], p["exit_reason"]
    assert p["ramp"] == 0.0


# ===================================================================== 4. a worker that will not start
@needs_bundle
def test_arming_is_refused_when_the_worker_does_not_start(staged):
    d, _cal, fname = staged
    seen = capture_events(d)
    d._policy_worker_opts = {"fail_load": "simulated: the bundle is unreadable"}
    ok, why, _ = d.policy_arm(spec(file=fname))
    assert not ok
    assert "simulated: the bundle is unreadable" in why and "POLICY_WORKER_VERSIONS" in why
    assert d._pol_req is None and d._pol is None, "a refused arm must not queue a run"
    assert worker_events(seen, "start")
    time.sleep(0.1)
    assert d.get_snapshot()["mode"] != "POLICY"


# ===================================================================== 5. the rollback
@needs_bundle
def test_with_no_worker_versions_the_in_process_path_still_runs(staged, monkeypatch):
    monkeypatch.setattr(daemon_mod, "POLICY_WORKER_VERSIONS", ())
    d, _cal, fname = staged
    seen = capture_events(d)
    ok, why, info = d.policy_arm(spec(file=fname, max_seconds=1.0))
    assert ok, why
    assert info["worker"] is False
    p = keep_alive_until(d, lambda p: p["phase"] == "done", timeout=30.0)
    assert p["reached_run"] and "max run time" in p["exit_reason"], p["exit_reason"]
    assert p["worker"] is False and d._pol["ctrl"] is not None
    ev = done_event(seen)
    assert ev["worker"] is False and ev["timing_ms"]["rtt"]["max"] == 0.0
    assert not worker_events(seen)


@needs_bundle
def test_one_run_can_opt_out_of_the_worker(staged):
    d, _cal, fname = staged
    ok, why, info = d.policy_arm(spec(file=fname, max_seconds=1.0, worker=False))
    assert ok, why
    assert info["worker"] is False
    d.policy_stop(hard=True)


# ===================================================================== 6. the same commands
@needs_bundle
def test_the_commands_of_a_worker_run_are_the_in_process_law_on_the_same_inputs(
        staged, roomy_deadline, monkeypatch):
    """The trace of a MockBus run through the worker, replayed into a controller in this process:
    identical target / kp / kd at every tick. This is the daemon's end-to-end packing -- frame
    order, sequence alignment, the start() -- not just the worker's arithmetic."""
    from bundle import Bundle
    from controller_balance import PolicyControllerBalance
    began, posted, got = [], {}, {}
    real_begin, real_post, real_collect = (PW.PolicyWorkerClient.begin, PW.PolicyWorkerClient.post,
                                           PW.PolicyWorkerClient.collect)

    def begin(self, seq, *obs, **kw):
        began.append([np.array(o, float) for o in obs])
        return real_begin(self, seq, *obs, **kw)

    def post(self, seq, *obs):
        posted[seq] = [np.array(o, float) for o in obs]
        return real_post(self, seq, *obs)

    def collect(self, seq, deadline):
        cmd, st = real_collect(self, seq, deadline)
        if cmd is not None:
            got[seq] = cmd
        return cmd, st
    monkeypatch.setattr(PW.PolicyWorkerClient, "begin", begin)
    monkeypatch.setattr(PW.PolicyWorkerClient, "post", post)
    monkeypatch.setattr(PW.PolicyWorkerClient, "collect", collect)

    d, _cal, fname = staged
    ok, why, _ = d.policy_arm(spec(file=fname, max_seconds=1.5))
    assert ok, why
    p = keep_alive_until(d, lambda p: p["phase"] == "done", timeout=30.0)
    assert p["reached_run"], p["exit_reason"]
    assert d._pol["worker_late"] == 0
    assert len(began) == 1 and len(posted) > 80
    assert sorted(got) == sorted(posted), "a posted observation went unanswered"

    local = PolicyControllerBalance(Bundle.load(BUNDLE_SRC))
    local.start(*began[0])
    for seq in sorted(posted):
        want = local.step(*posted[seq])
        g = got[seq]
        for name in ("target", "kp", "kd"):
            assert np.array_equal(getattr(g, name), getattr(want, name)), (seq, name)
    # and what went to the governor is what the log says the policy commanded before clamping:
    # the log rows are the governor's output, so only check they exist for every answered tick
    assert d.get_policy()["ticks"] >= len(got) - 1


# ===================================================================== the panel waits for it
def test_the_ui_proxy_waits_for_an_arm_long_enough_to_start_the_worker(monkeypatch):
    """uiproc cuts upstream calls off at 4 s, and arming a v3 run now spawns the worker (numpy alone
    is 1-3 s on the Pi 3B). An arm cut off at 4 s would read 'control process unreachable' for a
    run that had in fact been armed."""
    import uiproc
    seen = {}

    class R:
        status = 200
        headers = {}

        def read(self):
            return b"{}"

        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

    def urlopen(req, timeout):
        seen[req.full_url.split("8081")[1]] = timeout
        return R()
    monkeypatch.setattr(uiproc.urllib.request, "urlopen", urlopen)
    uiproc._fetch("/api/policy/arm", method="POST", body=b"{}")
    uiproc._fetch("/api/policy/keepalive", method="POST", body=b"{}")
    assert seen["/api/policy/arm"] > daemon_mod.POLICY_WORKER_START_S
    assert seen["/api/policy/keepalive"] == uiproc.TIMEOUT_S


# ===================================================================== the panel's preflight
@needs_bundle
def test_the_preflight_does_not_fail_a_v3_bundle(staged, monkeypatch):
    """REGRESSION (2026-11-20, on the robot). server._policy_controller built v1/v2 only, so every
    v3 bundle failed the panel's 'loop rate' check with 'the bundle would not build here' -- and
    the panel's answer to that is the allow_slow_loop box, which also disarms the in-run rate
    guard. A worker bundle is timed in the worker at arm time; the preflight says so."""
    import server
    d, _cal, _fname = staged
    monkeypatch.setitem(server.STATE, "daemon", d)
    rate = [c for c in server._policy_preflight(BUNDLE_SRC) if c["name"] == "loop rate"][0]
    assert rate["ok"] is True and rate.get("worker") is True, rate
    # and on the in-process path the v3 controller now builds and is actually timed
    monkeypatch.setattr(daemon_mod, "POLICY_WORKER_VERSIONS", ())
    server._PROBE_CACHE.clear()
    rate = [c for c in server._policy_preflight(BUNDLE_SRC) if c["name"] == "loop rate"][0]
    assert rate["step_ms"] is not None and rate["step_ms"] > 0, rate
    assert "would not build" not in rate["why"]
