"""The policy and its governor read the body rate in RAD/S; the Sense HAT publishes DEG/S.

2026-11-17, first standing run of standing04_xshape: the governor hard-stopped on "body rate 16.1
rad/s over the 12.0 limit" two policy ticks in, and the robot dropped. The black box had the torso
turning at 13-20 deg/s -- the same number, in the other unit. The test sense double every other
test uses publishes a zero gyro, which is why nothing caught it."""
import os
import sys
import time
import types

import numpy as np

import paths                                                  # noqa: F401  (sys.path)
import daemon as daemon_mod

sys.path.insert(0, paths.DEPLOY)
import run_policy                                             # noqa: E402


class _Sense:
    """sensehat.fast(): (t_wall, up_body, gyro_body in DEG/S)."""

    def __init__(self, gyro_dps):
        self.g = list(gyro_dps)

    def fast(self):
        return (time.time(), [0.0, 0.0, 1.0], list(self.g))


def test_the_daemon_hands_the_policy_rad_per_s():
    d = types.SimpleNamespace(sense=_Sense([16.1, -57.29578, 0.0]))
    grav, gyro, _age = daemon_mod.RobotDaemon._policy_imu(d)
    assert np.allclose(grav, [0.0, 0.0, -1.0])
    assert np.allclose(gyro, [np.radians(16.1), -1.0, 0.0], atol=1e-6)


def test_run_policy_hands_the_policy_rad_per_s():
    grav, gyro, _age = run_policy.Runner.read_imu(None, _Sense([57.29578, 0.0, -16.1]), 0.0)
    assert np.allclose(gyro, [1.0, 0.0, np.radians(-16.1)], atol=1e-6)


def test_a_16_deg_per_s_torso_no_longer_trips_the_12_rad_per_s_governor():
    d = types.SimpleNamespace(sense=_Sense([0.0, 16.1, 0.0]))
    _g, gyro, _a = daemon_mod.RobotDaemon._policy_imu(d)
    assert float(np.abs(gyro).max()) < 12.0
