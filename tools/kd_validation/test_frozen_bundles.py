"""The Kd-validation frozen-step bundles, offline (no hardware):

1. each kdval_*.npz, stepped through the deployed controller on noisy inputs, sends its declared
   MIT frame (meta["frozen_step"]) on every tick once the 80 ms target filter has settled;
2. the daemon arms one on MockBus, through the policy worker, like any v3 bundle, and its run log
   carries that frame in the target / kp / kd columns.

    python -m pytest tools/kd_validation/test_frozen_bundles.py
"""
import glob
import json
import os
import sys
import time

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
CTRL = os.path.join(ROOT, "controller")
TESTS = os.path.join(CTRL, "fixed_gait", "webui", "tests")
for p in (os.path.join(CTRL, "deploy"), os.path.join(CTRL, "fixed_gait"),
          os.path.join(CTRL, "fixed_gait", "webui"), CTRL, TESTS):
    if p not in sys.path:
        sys.path.insert(0, p)

from bundle import Bundle  # noqa: E402
from controller_balance import PolicyControllerBalance  # noqa: E402

BUNDLES = sorted(glob.glob(os.path.join(CTRL, "deploy", "bundles", "kdval_*.npz")))


def declared(b):
    f = b.meta["frozen_step"]
    return (np.asarray(f["target_rad"]), np.asarray(f["kp"]), np.asarray(f["kd"]))


def test_there_are_frozen_bundles():
    assert len(BUNDLES) >= 6, "run tools/kd_validation/make_frozen_bundles.py"


@pytest.mark.parametrize("path", BUNDLES, ids=[os.path.basename(p) for p in BUNDLES])
def test_the_frame_is_constant_and_the_declared_one(path):
    b = Bundle.load(path)
    T, KP, KD = declared(b)
    assert b.meta["reflex"]["enable"] is False, "a reflex would make the frame depend on the IMU"
    c = PolicyControllerBalance(b)
    rng = np.random.default_rng(1)
    q0 = np.asarray(b["default_motor_pos"], float)
    c.start(q0, np.zeros(6), np.zeros(6), np.array([0.0, 0.0, -1.0]), np.zeros(3))
    for i in range(150):
        o = c.step(q0 + 0.05 * rng.standard_normal(6), 3 * rng.standard_normal(6),
                   30 * rng.standard_normal(6), np.array([0.2 * np.sin(i / 5), 0.1, -1.0]),
                   3 * rng.standard_normal(3))
        np.testing.assert_allclose(o.kp, KP, rtol=1e-4)
        np.testing.assert_allclose(o.kd, KD, rtol=1e-4, atol=1e-5)
        if i >= 100:                       # the target filter (80 ms) has long settled
            np.testing.assert_allclose(o.target, T, atol=1e-4)


# ---------------------------------------------------------------- through the daemon (MockBus)
import paths  # noqa: E402
from test_blackbox import robot  # noqa: E402,F401  (pytest fixture)
from test_policy_run import armed, keep_alive_until, spec  # noqa: E402,F401


@pytest.fixture
def staged_frozen(armed):                                             # noqa: F811
    d, cal = armed
    name = "kdval_C_kd2.npz"
    dest = os.path.join(paths.POLICY_DIR, name)
    with open(os.path.join(CTRL, "deploy", "bundles", name), "rb") as s, open(dest, "wb") as o:
        o.write(s.read())
    yield d, cal, name
    os.remove(dest)


def test_the_daemon_runs_it_and_logs_the_frame(staged_frozen, monkeypatch):
    import daemon as daemon_mod
    monkeypatch.setattr(daemon_mod, "POLICY_WORKER_DEADLINE_MS", 200.0)   # the Mac is not the Pi
    d, _cal, name = staged_frozen
    ok, why, info = d.policy_arm(spec(file=name, max_seconds=2.0))
    assert ok, why
    assert info["bundle_version"] == 3
    p = keep_alive_until(d, lambda p: p["phase"] == "done", timeout=30.0)
    assert "max run time" in p["exit_reason"], p["exit_reason"]
    log = d._pol["log"][:int(d._pol["n"])]
    assert len(log) > 50
    _T, KP, KD = declared(Bundle.load(os.path.join(CTRL, "deploy", "bundles", name)))
    kp, kd = log[:, 43:49], log[:, 49:55]          # t | pos vel tau amps temp | grav gyro | target kp kd
    # the start ramps the gains in and the end (soft stop at max time) bleeds them out: in
    # between, every tick sends the declared frame
    full = np.all(np.isclose(kp, KP, rtol=1e-3), axis=1) & np.all(
        np.isclose(kd, KD, rtol=1e-3, atol=1e-4), axis=1)
    assert full.sum() >= 50, "only {} of {} ticks carried the declared kp/kd".format(full.sum(), len(log))
    assert np.all(kp <= KP * (1 + 1e-3)) and np.all(kd <= KD * (1 + 1e-3) + 1e-4), \
        "never more than the declared gains"
