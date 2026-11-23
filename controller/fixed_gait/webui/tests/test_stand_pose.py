"""🏠 Home drives to the standing pose (daemon.STAND_POSE_DEG): the v36 assembly pose of the rebuilt
robot with the relocated achilles (soles flat, CoM over them), written in the normalized frame
that is zeroed in the init pose (dash01_homing_v8 = same motor angles as v7).
dash-01CAD/homing/dash01_homing_v8/README.md has the derivation.

    python -m pytest controller/fixed_gait/webui/tests/test_stand_pose.py -v
"""
import daemon as daemon_mod
import paths
from test_blackbox import capture_zero, robot, wait_mode      # noqa: F401  (pytest fixtures)


def test_home_targets_the_standing_pose(robot):
    d, cal, _b, _dir = robot
    capture_zero(d, cal)
    ok, why = d.home()
    assert ok, why
    with d.lock:
        assert d._manual_targets == {n: daemon_mod.STAND_POSE_DEG[n] for n in paths.MOTOR_NAMES}
        assert d._home_kind == "stand"
    assert wait_mode(d, "MANUAL")


def test_stand_pose_is_symmetric_and_inside_the_hard_clamps():
    for side in paths.SIDES:
        for role in paths.ROLES:
            assert abs(daemon_mod.STAND_POSE_DEG[f"{side}.{role}"]) < daemon_mod.HARD_CLAMP[role]
    for role in paths.ROLES:      # normalized frame: both legs read the same in a mirrored pose,
        # up to the 0.27 deg by which the v8 export's own two legs differ
        assert abs(daemon_mod.STAND_POSE_DEG[f"left.{role}"]
                   - daemon_mod.STAND_POSE_DEG[f"right.{role}"]) < 0.3


def test_stand_pose_is_the_v8_export_pose():
    """norm = -sign * (v8 export qpos at the init pose), with the twin signs (twinmap.DEFAULT_SIGNS)
    and the export qpos printed by dash01_homing_v8/tools/derive_init.py (README section 3). Hips
    straight under the torso is 25 deg inward of the init stops."""
    import twinmap
    export_qpos_at_init = {"left.abd": 25.00, "left.cam": -55.47, "left.thigh": 17.11,
                           "right.abd": -25.00, "right.cam": 55.25, "right.thigh": -16.84}
    for n, q in export_qpos_at_init.items():
        assert daemon_mod.STAND_POSE_DEG[n] == -twinmap.DEFAULT_SIGNS[n] * q, n
