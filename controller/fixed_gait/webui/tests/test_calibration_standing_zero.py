"""The zero is captured STANDING (2026-11-17): "Set zero" records the standing calibration pose as
calibration.ZERO_POSE_DEG (= STAND_POSE_DEG), not as 0. The normalized frame itself is unchanged
(0 = the kneeling init pose), so everything stored in normalized degrees stays valid."""
import json

import paths                                             # noqa: F401  (sys.path)
import calibration
from stand_pose import STAND_POSE_DEG
from jointmap import JointMap
import os

RAW = {"right.abd": 12.3, "right.cam": -140.0, "right.thigh": 77.7,
       "left.abd": -3.1, "left.cam": 201.5, "left.thigh": -9.9}


def _cal(tmp_path, monkeypatch):
    monkeypatch.setattr(calibration.paths, "CALIB_FILE", str(tmp_path / "calibration.json"))
    c = calibration.Calibration()
    c.save = lambda path=None: calibration.Calibration.save(c, str(tmp_path / "calibration.json"))
    return c


def test_the_zero_pose_is_the_standing_pose():
    assert calibration.ZERO_POSE_DEG == {n: float(v) for n, v in STAND_POSE_DEG.items()}


def test_the_captured_pose_reads_as_the_standing_pose(tmp_path, monkeypatch):
    c = _cal(tmp_path, monkeypatch)
    ok, why = c.set_zero(RAW)
    assert ok, why
    for n in paths.MOTOR_NAMES:
        assert abs(c.norm(n, RAW[n]) - STAND_POSE_DEG[n]) < 1e-9
        assert abs(c.raw(n, STAND_POSE_DEG[n]) - RAW[n]) < 1e-9       # Home goes back to the capture
    assert c.zero_raw == RAW, "the raw-at-rest fingerprint is still the raw reading itself"


def test_the_captured_pose_is_model_zero_for_the_policy(tmp_path, monkeypatch):
    """The whole point: standing in the calibration setup = 0 rad in the policy's model."""
    c = _cal(tmp_path, monkeypatch)
    c.set_zero(RAW)
    jm = JointMap.load(os.path.join(paths.DEPLOY, "deploy_map.json"))
    norm = [c.norm(n, RAW[n]) for n in paths.MOTOR_NAMES]          # MOTOR_NAMES order
    ok, rep = jm.check_stance(norm, [0.0] * 6)
    assert ok, rep


def test_a_sign_flip_keeps_the_capture_at_the_standing_pose(tmp_path, monkeypatch):
    c = _cal(tmp_path, monkeypatch)
    c.set_zero(RAW)
    for n in paths.MOTOR_NAMES:
        c.set_sign(n, -c.signs[n])
        assert abs(c.norm(n, RAW[n]) - STAND_POSE_DEG[n]) < 1e-9, n
        # and moving the joint +1 raw now reads the other way round
        assert abs(c.norm(n, RAW[n] + 1.0) - (STAND_POSE_DEG[n] + c.signs[n])) < 1e-9


def test_the_zero_pose_survives_a_reload(tmp_path, monkeypatch):
    c = _cal(tmp_path, monkeypatch)
    c.set_zero(RAW)
    c2 = calibration.Calibration.load_or_new(str(tmp_path / "calibration.json"))
    assert c2.zero_pose == calibration.ZERO_POSE_DEG and c2.zero_pose_is_standing
    for n in paths.MOTOR_NAMES:
        assert abs(c2.norm(n, RAW[n]) - STAND_POSE_DEG[n]) < 1e-9


def test_an_old_kneeling_calibration_still_means_what_it_meant(tmp_path):
    """A file written before 2026-11-17 has no zero_pose_deg: it was captured kneeling (= 0)."""
    p = tmp_path / "calibration.json"
    p.write_text(json.dumps({"stage": "complete", "zero_epoch": 3, "zero_raw": RAW,
                             "motors": {n: {"offset_deg": RAW[n], "sign": 1, "confirmed": True}
                                        for n in paths.MOTOR_NAMES}}))
    c = calibration.Calibration.load_or_new(str(p))
    assert not c.zero_pose_is_standing
    assert all(v == 0.0 for v in c.zero_pose.values())
    for n in paths.MOTOR_NAMES:
        assert c.norm(n, RAW[n]) == 0.0
    assert c.snapshot()["zero_pose"] == "kneeling (legacy)"
    # and a single-joint standing re-zero cannot be mixed into it
    ok, why = c.set_zero_one("left.cam", 10.0)
    assert not ok and "KNEELING" in why
    # a full re-zero converts it to the standing convention
    c.save = lambda path=None: calibration.Calibration.save(c, str(p))
    c.set_zero(RAW)
    assert c.zero_pose_is_standing and c.snapshot()["zero_pose"] == "standing"
