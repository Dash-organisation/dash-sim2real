"""The policy worker process and its client, without the daemon (plan B, step 1 -- 2026-11-18).

What has to be true before the daemon may hand its control law to another process:

  * it is the SAME control law: bit for bit, the commands a worker returns are the ones a
    controller in this process would have returned for the same observations;
  * the commands to the law (set_speed, set_run, ...) land between the same two steps;
  * every way the worker can fail comes back as a state, within the deadline, never as a hang or
    an exception in the CAN thread: an exception in the law, a killed process, a slow answer;
  * and stop() always leaves no process behind.

No hardware, no daemon. The bundles are synthetic but go through the real loader and controllers.
"""
import json
import os
import signal
import sys
import time

import numpy as np
import pytest

import paths                                                       # noqa: F401 -- sys.path
import policy_worker as PW

_DEPLOY_TESTS = os.path.join(paths.DEPLOY, "tests")
if _DEPLOY_TESTS not in sys.path:
    sys.path.insert(0, _DEPLOY_TESTS)
import test_balance_deploy as TBD                                  # noqa: E402
import v2_fixture                                                  # noqa: E402

from bundle import Bundle                                          # noqa: E402
from controller_balance import PolicyControllerBalance            # noqa: E402
from controller_v2 import PolicyControllerV2                      # noqa: E402

DEADLINE_S = 0.5            # generous: the Mac under a full test run is not the robot


def write_v3(path, seed=3):
    """A v3 balance bundle with a RANDOM net -- a zero net would make every command the stance and
    bit-exactness would prove nothing."""
    arrays, meta = TBD.arrays_and_meta(zero_net=False, seed=seed)
    np.savez(str(path), meta=np.array(json.dumps(meta, sort_keys=True)), **arrays)
    return str(path)


@pytest.fixture
def v3_path(tmp_path):
    return write_v3(tmp_path / "v3_worker_test.npz")


def observations(n, seed=11):
    """A fixed, random-but-plausible observation stream around the stance."""
    rng = np.random.default_rng(seed)
    out = []
    for _ in range(n):
        pos = TBD.STANCE + rng.normal(0, 0.03, 6)
        vel = rng.normal(0, 0.3, 6)
        tau = rng.normal(0, 3.0, 6)
        grav = np.array([0.0, 0.0, -1.0]) + rng.normal(0, 0.05, 3)
        gyro = rng.normal(0, 0.2, 3)
        out.append((pos, vel, tau, grav, gyro))
    return out


@pytest.fixture
def started():
    """Start a worker on a path; always stop it afterwards, and check nothing is left behind."""
    made = []

    def go(path, opts=None):
        c = PW.PolicyWorkerClient(opts)
        made.append(c)
        c.start(path, timeout=30.0)
        return c
    yield go
    for c in made:
        c.stop(wait=True)
        if c.proc is not None:
            assert c.proc.exitcode is not None, "a worker outlived its test"


def run_worker(c, obs, deadline_s=DEADLINE_S, between=None):
    o0 = obs[0]
    assert c.begin(0, o0[0], np.zeros(6), o0[2], o0[3], o0[4]) == PW.OK, c.why
    cmds = []
    for i, o in enumerate(obs, start=1):
        if between is not None:
            between(i, c)
        assert c.post(i, *o)
        cmd, st = c.collect(i, time.perf_counter() + deadline_s)
        assert st == PW.OK, (i, st, c.why)
        cmds.append(cmd)
    return cmds


def run_local(ctrl, obs, between=None):
    o0 = obs[0]
    ctrl.start(o0[0], np.zeros(6), o0[2], o0[3], o0[4])
    out = []
    for i, o in enumerate(obs, start=1):
        if between is not None:
            between(i, ctrl)
        out.append(ctrl.step(*o))
    return out


# ===================================================================== the wire format
def test_the_frames_have_the_sizes_both_ends_assert():
    assert PW.CMD_N == 49 and PW.OBS_N == 24
    cmd = PW.unpack_cmd(PW.pack_cmd(7, types_ns(), 12.5, True, 1.25, np.empty(PW.CMD_N)))
    assert cmd.seq == 7 and cmd.heading_deg == 12.5 and cmd.braking is True
    assert cmd.step_ms == 1.25 and cmd.target.shape == (6,) and cmd.action.shape == (18,)
    with pytest.raises(ValueError):
        PW.unpack_cmd(b"C" + b"\0" * 10)
    with pytest.raises(ValueError):
        PW.unpack_obs(b"T" + b"\0" * 10)


def types_ns():
    import types
    return types.SimpleNamespace(target=np.arange(6.0), kp=np.ones(6), kd=np.ones(6),
                                 action=np.zeros(18, np.float32), phase=0.0, freq=0.0,
                                 saturated=True, vel_est=np.zeros(3))


# ===================================================================== 1. the same control law
def test_the_worker_is_bit_exact_with_the_controller_in_process(v3_path, started):
    obs = observations(200)
    c = started(v3_path)
    assert c.ready["version"] == 3 and c.ready["step_ms"] > 0
    got = run_worker(c, obs)
    want = run_local(PolicyControllerBalance(Bundle.load(v3_path)), obs)
    assert len(got) == len(want) == 200
    moved = 0
    for k, (g, w) in enumerate(zip(got, want)):
        for name in ("target", "kp", "kd"):
            assert np.array_equal(getattr(g, name), getattr(w, name)), (k, name)
        assert np.array_equal(g.action, np.asarray(w.action, np.float64)), (k, "action")
        assert g.saturated == bool(w.saturated)
        assert np.array_equal(g.vel_est, np.asarray(w.vel_est, np.float64).ravel())
        moved += int(not np.allclose(g.target, TBD.NOMINAL))
    assert moved > 100, "the random net barely moved the command -- the test proves nothing"


# ===================================================================== 2. commands between steps
def test_commands_to_the_law_land_between_the_same_two_steps(tmp_path, started):
    path = v2_fixture.write_v2_bundle(str(tmp_path / "v2_joy.npz"), objective="joystick",
                                      v_max=2.5, v_min=0.0)
    obs = observations(120, seed=5)

    def remote(i, c):
        if i == 20:
            st, res = c.call("set_speed", 1.5, seq=i)
            assert st == PW.OK and res[0] is True, (st, res)
        if i == 70:
            st, res = c.call("set_speed", 0.0, seq=i)
            assert st == PW.OK

    def local(i, ctrl):
        if i == 20:
            assert ctrl.set_speed(1.5)[0] is True
        if i == 70:
            ctrl.set_speed(0.0)
    c = started(path)
    got = run_worker(c, obs, between=remote)
    want = run_local(PolicyControllerV2(Bundle.load(path)), obs, between=local)
    plain = run_local(PolicyControllerV2(Bundle.load(path)), obs)
    for g, w in zip(got, want):
        assert np.array_equal(g.target, w.target) and np.array_equal(g.kp, w.kp)
        assert g.speed_cmd == float(w.speed_cmd)
    assert any(not np.array_equal(w.target, q.target) for w, q in zip(want, plain)), \
        "set_speed changed nothing -- the test cannot tell a dropped call from a delivered one"


def test_only_the_listed_methods_cross(v3_path, started):
    c = started(v3_path)
    with pytest.raises(ValueError):
        c.call("step")


# ===================================================================== 3. the law raises
def test_an_exception_in_the_law_comes_back_with_its_text(v3_path, started):
    c = started(v3_path, {"raise_at": 3})
    o = observations(6)
    assert c.begin(0, o[0][0], np.zeros(6), *o[0][2:]) == PW.OK
    for i in range(1, 4):
        assert c.post(i, *o[i])
        assert c.collect(i, time.perf_counter() + DEADLINE_S)[1] == PW.OK
    c.post(4, *o[4])
    cmd, st = c.collect(4, time.perf_counter() + DEADLINE_S)
    assert cmd is None and st == PW.ERROR
    assert "raised at step 3" in c.why
    c.proc.join(5.0)
    assert c.proc.exitcode is not None, "the worker must exit after reporting the error"


def test_an_unloadable_bundle_is_a_start_failure_with_the_reason(tmp_path):
    bad = tmp_path / "corrupt.npz"
    bad.write_bytes(b"this is not a bundle")
    c = PW.PolicyWorkerClient()
    with pytest.raises(PW.PolicyWorkerError) as e:
        c.start(str(bad), timeout=30.0)
    assert "failed to start" in str(e.value)
    assert c.proc.exitcode is not None


def test_a_failure_while_loading_reports_its_text(tmp_path):
    c = PW.PolicyWorkerClient({"fail_load": "simulated load failure"})
    with pytest.raises(PW.PolicyWorkerError) as e:
        c.start(str(tmp_path / "whatever.npz"), timeout=30.0)
    assert "simulated load failure" in str(e.value)


# ===================================================================== 4. the worker is killed
def test_a_killed_worker_is_dead_within_the_deadline(v3_path, started):
    c = started(v3_path)
    o = observations(4)
    assert c.begin(0, o[0][0], np.zeros(6), *o[0][2:]) == PW.OK
    assert c.post(1, *o[1])
    assert c.collect(1, time.perf_counter() + DEADLINE_S)[1] == PW.OK
    os.kill(c.proc.pid, signal.SIGKILL)
    c.proc.join(5.0)
    t0 = time.perf_counter()
    c.post(2, *o[2])                       # may or may not fail: the pipe can still take bytes
    cmd, st = c.collect(2, time.perf_counter() + 0.05)
    took = time.perf_counter() - t0
    assert cmd is None and st == PW.DEAD, (st, c.why)
    assert took < 0.5, "collect blocked {:.2f} s on a dead worker".format(took)


# ===================================================================== 5. the worker is slow
def test_a_late_answer_is_late_and_then_discarded(v3_path, started):
    c = started(v3_path, {"sleep_ms": 150, "sleep_steps": [2]})
    o = observations(6)
    assert c.begin(0, o[0][0], np.zeros(6), *o[0][2:]) == PW.OK
    assert c.post(1, *o[1])
    assert c.collect(1, time.perf_counter() + DEADLINE_S)[1] == PW.OK
    assert c.post(2, *o[2])
    assert c.collect(2, time.perf_counter() + DEADLINE_S)[1] == PW.OK
    # the worker counts steps from 0 after start(): index 2 is the third, and it sleeps 150 ms
    # against a 20 ms deadline
    assert c.post(3, *o[3])
    t0 = time.perf_counter()
    cmd, st = c.collect(3, time.perf_counter() + 0.02)
    assert cmd is None and st == PW.LATE
    assert time.perf_counter() - t0 < 0.1, "a late answer must not hold the caller past its deadline"
    # the next tick: answer 3 arrives first and is thrown away, then 4 is the one returned
    assert c.post(4, *o[4])
    cmd, st = c.collect(4, time.perf_counter() + DEADLINE_S)
    assert st == PW.OK and cmd.seq == 4
    assert c.stale == 1


def test_an_answer_from_the_future_is_bad(v3_path, started):
    c = started(v3_path)
    o = observations(3)
    assert c.begin(0, o[0][0], np.zeros(6), *o[0][2:]) == PW.OK
    assert c.post(5, *o[1])
    cmd, st = c.collect(4, time.perf_counter() + DEADLINE_S)
    assert cmd is None and st == PW.BAD and "answered observation 5" in c.why


# ===================================================================== 6. no zombies
def test_stop_always_ends_the_process(v3_path):
    c = PW.PolicyWorkerClient()
    c.start(v3_path, timeout=30.0)
    c.stop(wait=True)
    assert c.proc.exitcode is not None and not c.alive()
    # a worker stuck in a long step still goes: terminate after the timeout
    c = PW.PolicyWorkerClient({"sleep_ms": 10_000, "sleep_from": 0})
    c.start(v3_path, timeout=30.0)
    o = observations(2)
    assert c.begin(0, o[0][0], np.zeros(6), *o[0][2:]) == PW.OK
    c.post(1, *o[1])
    t0 = time.time()
    c.stop(timeout=0.2, wait=True)
    assert c.proc.exitcode is not None
    assert time.time() - t0 < 3.0
    # and the non-blocking form (the CAN thread's) reaps on its own thread
    c = PW.PolicyWorkerClient()
    c.start(v3_path, timeout=30.0)
    t0 = time.perf_counter()
    c.stop(wait=False)
    assert time.perf_counter() - t0 < 0.05, "stop(wait=False) must not wait for the exit"
    c._reaper.join(5.0)
    assert c.proc.exitcode is not None
    # stopping twice is harmless, and a stopped client reports dead rather than raising
    c.stop(wait=True)
    assert c.collect(1, time.perf_counter())[1] == PW.DEAD
    assert c.post(1, *o[1]) is False


def test_the_worker_reports_where_it_runs(v3_path, started):
    c = started(v3_path)
    r = c.ready
    assert r["pid"] == c.proc.pid and r["pid"] != os.getpid()
    assert r["openblas_threads"] == "1"
    assert r["step_p90_ms"] >= r["step_ms"] > 0
    if hasattr(os, "sched_setaffinity"):
        assert r["affinity"]
