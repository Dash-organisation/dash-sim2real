"""run_policy --mock gives the simulated IMU a mount calibration of its own (2026-11-17).

It used to take the robot's data/sensehat_mount.json -- the real HAT, on the roof -- and apply it to
the simulator's under-the-base mount, which reads the robot upside down: every web-UI rehearsal
ended on its first tick with "fallen: gravity_z +1.00 ... tilt 176 deg". Without a calibration it
seeded one through MountCal.set_declared(), which no longer exists, and set_level() SAVES -- so a
rehearsal on a Pi without a mount calibration would have written the simulator's rotation as the
robot's."""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "..", "fixed_gait", "webui"))

import mountcal       # noqa: E402
import sensehat       # noqa: E402
import run_policy     # noqa: E402


def test_the_mock_mount_is_the_simulators_rotation():
    m = run_policy._mock_mount(sensehat)
    assert m.calibrated
    assert np.allclose(m.R, sensehat.MOCK_R, atol=1e-9)
    up_body = m.R @ (sensehat.MOCK_R.T @ np.array([0.0, 0.0, 1.0]))
    assert np.allclose(up_body, [0.0, 0.0, 1.0], atol=1e-9), "upright must read upright"


def test_the_mock_mount_never_writes_the_robots_calibration(tmp_path, monkeypatch):
    f = tmp_path / "sensehat_mount.json"
    monkeypatch.setattr(mountcal, "MOUNT_FILE", str(f))
    real_save = mountcal.MountCal.save
    calls = []
    monkeypatch.setattr(mountcal.MountCal, "save",
                        lambda self, *a, **k: (calls.append(1), real_save(self, *a, **k)))
    run_policy._mock_mount(sensehat)
    assert not f.exists() and not calls, "a rehearsal wrote the robot's IMU mount calibration"
