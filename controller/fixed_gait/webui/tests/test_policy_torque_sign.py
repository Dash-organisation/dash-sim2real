"""The policy's torque observation carries the calibration sign (2026-11-17).

First standing run of standing04_xshape: at the handover, hip_roll_L, cam_L and thigh_R read the
OPPOSITE sign of the simulation's standing torques (drive_kp * (nominal_ctrl - default_motor_pos)),
the other three agreed -- and the three were exactly the drives whose calibration sign is -1.
Positions go raw -> normalized (calibration sign) -> model (deploy_map sign); currents went raw ->
model with the deploy_map sign only. A current must take the same road as the position it moves."""
import types

import numpy as np

import paths
import daemon as daemon_mod
from jointmap import JointMap
import os

SIGNS = {"right.abd": 1.0, "right.cam": 1.0, "right.thigh": -1.0,
         "left.abd": -1.0, "left.cam": -1.0, "left.thigh": 1.0}


def _daemon(cur):
    calib = types.SimpleNamespace(signs=dict(SIGNS), norm=lambda n, raw: SIGNS[n] * raw)
    by_name = {n: types.SimpleNamespace(pos=0.0, cur=cur[n], temp=30.0, err=0)
               for n in paths.MOTOR_NAMES}
    return types.SimpleNamespace(calib=calib, by_name=by_name)


def test_a_current_and_the_position_it_drives_have_the_same_sign_in_the_model():
    jm = JointMap.load(os.path.join(paths.DEPLOY, "deploy_map.json"))
    for n in paths.MOTOR_NAMES:
        # +1 A raw drives the raw angle up; the raw angle going up moves the MODEL angle by
        # sign_cal * sign_map -- so must the torque
        cur = {m: (1.0 if m == n else 0.0) for m in paths.MOTOR_NAMES}
        d = _daemon(cur)
        _pos, amps, _t, _e = daemon_mod.RobotDaemon._policy_measure(d, {"jm": jm})
        tau = jm.torque_to_model(amps)
        k = list(jm.model_from_motor).index(list(paths.MOTOR_NAMES).index(n))
        raw_up = np.zeros(6)
        raw_up[list(paths.MOTOR_NAMES).index(n)] = 1.0
        dmodel = jm.to_model_rad([d.calib.norm(m, raw_up[i]) for i, m in enumerate(paths.MOTOR_NAMES)]) \
            - jm.to_model_rad([0.0] * 6)
        assert np.sign(tau[k]) == np.sign(dmodel[k]) != 0, n


def test_the_first_standing_runs_currents_now_match_the_simulation():
    """The handover of 2026-11-17, replayed: raw currents that gave the model torques
    [+5.86, -17.81, +10.93, +1.27, -19.04, +20.54] now give the simulation's signs
    [-, +, +, +, -, -] (drive_kp * (nominal_ctrl - default_motor_pos) of standing04_xshape)."""
    jm = JointMap.load(os.path.join(paths.DEPLOY, "deploy_map.json"))
    old = np.array([5.86, -17.81, 10.93, 1.27, -19.04, 20.54])            # model, actuator order
    raw = np.zeros(6)
    raw[np.asarray(jm.model_from_motor)] = old / (jm.sign * jm.kt_joint * jm.kt_efficiency)
    cur = {n: float(raw[i]) for i, n in enumerate(paths.MOTOR_NAMES)}
    _pos, amps, _t, _e = daemon_mod.RobotDaemon._policy_measure(_daemon(cur), {"jm": jm})
    tau = jm.torque_to_model(amps)
    assert list(np.sign(tau)) == [-1, 1, 1, 1, -1, -1], tau
