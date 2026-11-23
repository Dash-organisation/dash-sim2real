"""A standing policy run is guarded on the MEASURED pose, and its PD command may lead the
workspace edge (policy_ws.py, 2026-11-17).

standing04_xshape commands nominal_ctrl ~2-3 deg past the measured workspace edge (the tibia/knee
stop minus the workspace margin) while it stands ~2 deg inside it: the command is a torque request,
drive_kp * (target - pos), not a pose. Checked as a pose it refused every arm. These tests pin the
replacement: the start pose must be inside with 1 deg to spare, the command may lead by up to
8 deg, the measured pose ending up within 1 deg of the edge ends the run (in the SET_POS hold,
standing), a run declared hung keeps the strict command rule, and the clearances are recorded.

MockBus only -- no hardware."""
import math
import os
import types

import numpy as np
import pytest

import paths
import daemon as daemon_mod
import policy_ws
from joint_limits import JointLimits
from jointmap import JointMap
from test_blackbox import wait_mode
from test_policy_run import (armed, keep_alive_until, needs_v2, v2_fixture, v2_spec,  # noqa: F401
                             _loaded, _to_policy_pose)
from test_blackbox import robot                                               # noqa: F401

MOTORS = list(paths.MOTOR_NAMES)
# nominal_ctrl - default_motor_pos of standing04_xshape_best (model rad, actuator order): the
# gravity deflection the standing command leads the pose by
XSHAPE_LEAD = np.radians([-1.68, 4.03, 1.56, 1.68, -4.03, -1.57])
NAME = "zz_ws_lead_test.npz"


# ===================================================================== the clearance field
def _disk_limits(centres, radius, res=1.0, abd=(-47.0, 47.0)):
    """A JointLimits whose knee region is a disk per leg: clearance is known in closed form."""
    legs = {}
    for side, (cc, ct) in centres.items():
        c0, t0 = cc - 40.0, ct - 40.0
        n = int(80 / res)
        ci = c0 + (np.arange(n) + 0.5) * res
        tj = t0 + (np.arange(n) + 0.5) * res
        C, T = np.meshgrid(ci, tj, indexing="ij")
        grid = np.hypot(C - cc, T - ct) <= radius
        legs[side] = dict(abd_safe=abd, abd_observed=abd, abd_zero=float("nan"), knee_grid=grid,
                          knee_cam_origin=c0, knee_thigh_origin=t0, knee_grid_deg=res,
                          knee_zero=np.array([np.nan, np.nan]))
    return JointLimits(legs)


def test_the_clearance_of_a_disk_is_its_radius_minus_the_distance():
    lim = _disk_limits({"left": (50.0, 10.0)}, radius=10.0)
    c = policy_ws.WorkspaceClearance(lim)
    # good to about one cell (the pose is read in the cell that holds it): 1.25 at 1 deg/cell
    for d in (0.0, 3.0, 6.0, 9.0):                    # inside
        assert c.knee("left", 50.0 + d, 10.0) == pytest.approx(10.0 - d, abs=1.25)
    for d in (12.0, 15.0, 19.0):                      # outside: negative
        assert c.knee("left", 50.0 + d, 10.0) == pytest.approx(10.0 - d, abs=1.25)
    assert c.knee("left", 50.0 + 30.0, 10.0) == -policy_ws.RADIUS_DEG   # saturated
    assert c.knee("left", 500.0, 10.0) == -policy_ws.RADIUS_DEG         # off the grid


def test_the_sign_agrees_with_joint_limits_everywhere():
    lim = _disk_limits({"left": (50.0, 10.0), "right": (40.0, 0.0)}, radius=7.0)
    c = policy_ws.WorkspaceClearance(lim)
    rng = np.random.default_rng(1)
    for _ in range(3000):
        side = rng.choice(["left", "right"])
        cam, th = rng.uniform(20, 80), rng.uniform(-30, 40)
        inside = lim.check_knee(side, cam, th)[0]
        assert (c.knee(side, cam, th) > 0) == inside


def test_the_pose_clearance_is_the_worst_of_both_legs_knee_and_abduction():
    lim = _disk_limits({"left": (50.0, 10.0), "right": (50.0, 10.0)}, radius=10.0, abd=(-30, 0))
    c = policy_ws.WorkspaceClearance(lim)
    pose = {"left.abd": -15.0, "left.cam": 50.0, "left.thigh": 10.0,
            "right.abd": -15.0, "right.cam": 57.0, "right.thigh": 10.0}
    v, where = c.pose(pose)
    assert where == "right knee" and v == pytest.approx(3.0, abs=1.0)
    pose["left.abd"] = -1.0
    v, where = c.pose(pose)
    assert where == "left abduction" and v == pytest.approx(1.0)


def test_supports_only_gridded_limits():
    assert policy_ws.WorkspaceClearance.supports(_disk_limits({"left": (0.0, 0.0)}, 5.0))
    assert not policy_ws.WorkspaceClearance.supports(types.SimpleNamespace(validate=None))


# ===================================================================== the daemon
@pytest.fixture
def staged_lead(armed):                                                       # noqa: F811
    """A v2 bundle whose nominal_ctrl leads default_motor_pos like standing04_xshape's does."""
    d, cal = armed
    dest = os.path.join(paths.POLICY_DIR, NAME)
    a, _meta = v2_fixture.v2_arrays_and_meta()
    dmp = np.asarray(a["default_motor_pos"], float)
    v2_fixture.write_v2_bundle(dest, nominal=dmp + XSHAPE_LEAD, default_motor_pos=dmp)
    yield d, cal, NAME
    if os.path.exists(dest):
        os.remove(dest)


def _poses(name):
    b = np.load(os.path.join(paths.POLICY_DIR, name), allow_pickle=True)
    jm = JointMap.load(os.path.join(paths.DEPLOY, "deploy_map.json"))
    rest = dict(zip(MOTORS, jm.to_norm_deg(b["default_motor_pos"])))
    nom = dict(zip(MOTORS, jm.to_norm_deg(b["nominal_ctrl"])))
    return rest, nom


def _workspace(d, name, rest_clear, nominal_past):
    """Install a disk workspace per leg on the rest->nominal line: the start pose `rest_clear` deg
    inside, nominal_ctrl `nominal_past` deg outside (needs rest_clear + nominal_past = distance)."""
    rest, nom = _poses(name)
    centres, radius = {}, None
    for s in ("left", "right"):
        r = np.array([rest[s + ".cam"], rest[s + ".thigh"]])
        n = np.array([nom[s + ".cam"], nom[s + ".thigh"]])
        dist = float(np.linalg.norm(n - r))
        u = (n - r) / dist
        # centre behind the rest pose so that: radius - |rest-c| = rest_clear, |nom-c| - radius = past
        k = (nominal_past + rest_clear - dist) / 2.0 + 6.0     # any k works; keep the disk big
        rad = k + rest_clear
        centre = r - u * k
        assert abs((np.linalg.norm(n - centre) - rad) - (dist - rest_clear)) < 1e-6
        centres[s] = tuple(centre)
        radius = rad if radius is None else min(radius, rad)
    lim = _disk_limits(centres, radius)
    d.wstore = types.SimpleNamespace(limits=lim, legs={})
    return rest, nom


def _dist(name):
    rest, nom = _poses(name)
    return min(math.hypot(nom[s + ".cam"] - rest[s + ".cam"], nom[s + ".thigh"] - rest[s + ".thigh"])
               for s in ("left", "right"))


@needs_v2
def test_a_standing_arm_accepts_a_command_that_leads_the_edge(staged_lead):
    """The standing04_xshape case: start pose inside, nominal_ctrl past the edge but within 8 deg.
    The old rule (command must be inside) refused this; the new one arms."""
    d, _cal, name = staged_lead
    D = _dist(name)
    _workspace(d, name, rest_clear=2.5, nominal_past=D - 2.5)
    _to_policy_pose(d, name)
    ok, why, info = d.policy_arm(v2_spec(file=name, hung=False))
    try:
        assert ok, why
        assert info["ws_rest_clearance_deg"] >= daemon_mod.POLICY_WS_REST_MARGIN_DEG
        assert info["ws_stance_clearance_deg"] < 0, "nominal_ctrl really is past the edge"
    finally:
        d.policy_stop(hard=True)
        keep_alive_until(d, lambda p: p["phase"] == "done", timeout=5.0)


@needs_v2
def test_the_same_workspace_still_refuses_a_run_declared_hung(staged_lead):
    """Unloaded, a leg goes where it is commanded: hung keeps the strict command rule."""
    d, _cal, name = staged_lead
    D = _dist(name)
    _workspace(d, name, rest_clear=2.5, nominal_past=D - 2.5)
    ok, why, _ = d.policy_arm(v2_spec(file=name, hung=True))
    assert not ok and "stance" in why and "workspace" in why, why


@needs_v2
def test_a_start_pose_too_close_to_the_edge_is_refused(staged_lead):
    d, _cal, name = staged_lead
    D = _dist(name)
    _workspace(d, name, rest_clear=0.4, nominal_past=D - 0.4)
    _to_policy_pose(d, name)
    ok, why, info = d.policy_arm(v2_spec(file=name, hung=False))
    assert not ok and "default_motor_pos" in why and "edge" in why, why
    assert info["ws_rest_clearance_deg"] < daemon_mod.POLICY_WS_REST_MARGIN_DEG


@needs_v2
def test_a_command_leading_by_more_than_the_limit_is_refused(staged_lead, monkeypatch):
    d, _cal, name = staged_lead
    _workspace(d, name, rest_clear=2.5, nominal_past=_dist(name) - 2.5)
    # same geometry, but pretend the edge is 9 deg further in on the nominal side
    real = policy_ws.WorkspaceClearance.pose
    stance_like = {"n": 0}

    def pose(self, norm):
        c, where = real(self, norm)
        stance_like["n"] += 1
        return (c - 9.0, where) if c < 0 else (c, where)
    monkeypatch.setattr(policy_ws.WorkspaceClearance, "pose", pose)
    _to_policy_pose(d, name)
    ok, why, _ = d.policy_arm(v2_spec(file=name, hung=False))
    assert not ok and "nominal_ctrl" in why and "lead" in why, why


@needs_v2
def test_the_measured_pose_nearing_the_edge_ends_a_standing_run_in_the_hold(staged_lead):
    """Robot lifted, leg sagging toward the stop, a push: the MEASURED pose comes within 1 deg of
    the edge -> the run ends, and a standing robot is held in SET_POS, not dropped. The minimum
    clearance is recorded."""
    d, _cal, name = staged_lead
    D = _dist(name)
    rest, nom = _workspace(d, name, rest_clear=2.5, nominal_past=D - 2.5)
    _to_policy_pose(d, name)
    mp = pytest.MonkeyPatch()
    try:
        _loaded(mp, d)
        # hold the measured pose at default_motor_pos (the robot's weight), whatever is commanded
        b = np.load(os.path.join(paths.POLICY_DIR, name), allow_pickle=True)
        rest_model = np.asarray(b["default_motor_pos"], float)
        shift = {"rad": np.zeros(6)}
        real = d._policy_measure

        def measure(p):
            _pos, a, temp, err = real(p)
            return rest_model + shift["rad"], a, temp, err
        mp.setattr(d, "_policy_measure", measure)
        ok, why, _ = d.policy_arm(v2_spec(file=name, hung=False, max_seconds=10.0))
        assert ok, why
        p = keep_alive_until(d, lambda p: p["phase"] in ("run", "done"), timeout=10.0)
        assert p["phase"] == "run", p.get("exit_reason")
        time.sleep(0.3)
        p = d.get_snapshot()["policy"]
        assert p["phase"] == "run", "standing at default_motor_pos, 2.5 deg inside: keeps running"
        assert p["ws_meas_clearance_min_deg"] == pytest.approx(2.5, abs=1.0)
        # now the leg moves 2 deg toward the edge (the nominal side): 0.5 deg left < 1
        shift["rad"] = XSHAPE_LEAD * (2.0 / D)
        p = keep_alive_until(d, lambda p: p["phase"] == "done", timeout=5.0)
        assert "measured pose" in d._pol["exit_reason"] and "edge" in d._pol["exit_reason"]
        assert wait_mode(d, "MANUAL"), "a standing robot must be held in SET_POS, not dropped"
        assert d._pol["ws_meas_min"] < daemon_mod.POLICY_WS_MEAS_MARGIN_DEG
    finally:
        d.policy_stop(hard=True)
        mp.undo()


@needs_v2
def test_a_standing_command_is_judged_on_its_lead_not_on_being_inside(staged_lead):
    """_policy_ws_command, directly: inside -> ok, 3 deg past -> ok, 9 deg past -> refused."""
    d, _cal, name = staged_lead
    _workspace(d, name, rest_clear=2.5, nominal_past=_dist(name) - 2.5)
    b = np.load(os.path.join(paths.POLICY_DIR, name), allow_pickle=True)
    jm = JointMap.load(os.path.join(paths.DEPLOY, "deploy_map.json"))
    p = {"jm": jm, "ws_clear": policy_ws.WorkspaceClearance(d.wstore.limits),
         "ws_cmd_min": float("inf"), "ws_cmd_where": None}
    rest = np.asarray(b["default_motor_pos"], float)
    D = _dist(name)
    assert d._policy_ws_command(p, rest)[0]
    assert d._policy_ws_command(p, rest + XSHAPE_LEAD * ((2.5 + 3.0) / D))[0]
    ok, why = d._policy_ws_command(p, rest + XSHAPE_LEAD * ((2.5 + 9.0) / D))
    assert not ok and "leads" in why, why
    assert p["ws_cmd_min"] < -daemon_mod.POLICY_WS_CMD_LEAD_DEG


import time                                                                   # noqa: E402
