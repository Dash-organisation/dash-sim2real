#!/usr/bin/env python3
"""Closed-loop MuJoCo replay of a v3 balance bundle on a standing plant, in plain numpy (no JAX).

Mac-side only (needs `mujoco`). It mirrors BalanceRL/env.py `_step_one` for the nominal plant with
the noise and the domain randomization turned off:

    100 Hz control: controller/deploy/controller_balance.py (the DEPLOYED law, byte for byte)
    1 kHz physics : tau = clip(kp (q_ref - q) - kd qdot, +-forcerange)       (BalanceRL/drive.py)
    transport     : the command reaches the plant `delay_ms` later (default 12 ms, the sim's mean)
    sensors       : joint q/qd, last applied torque, gravity in the torso frame, imu_gyro

Used by the Kd validation test to (1) take a policy step from a run where the policy stands in
simulation, and (2) check that the same step, FROZEN (constant target/kp/kd), still stands in the
simulation, and what changing kd does there. See docs/sim2real/DASH01_plan_test_Kd_2026-11-24.md.
"""
import os
import sys

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "controller", "deploy"))

import mujoco  # noqa: E402

from bundle import Bundle  # noqa: E402
from controller_balance import PolicyControllerBalance  # noqa: E402

ACT_NAMES = ["hip_roll_L", "cam_L", "thigh_L", "hip_roll_R", "cam_R", "thigh_R"]
DEFAULT_MODEL = os.path.join(ROOT, "BalanceRL", "model", "stand_xshape.xml")


# stand_*.xml reference ../../xshape_closed/meshes/, a folder of the old DASH-01 repo that this
# repo does not carry. The same STL files (byte-identical, checked 2026-11-23) are in dash-01CAD.
MESH_DIR = os.path.join(ROOT, "dash-01CAD", "homing", "dash01_homing_v8", "meshes")


def load_model(model_path):
    with open(model_path) as f:
        xml = f.read()
    xml = xml.replace('file="../../xshape_closed/meshes/', 'file="' + MESH_DIR + "/")
    return mujoco.MjModel.from_xml_string(xml)


class Plant:
    def __init__(self, model_path=DEFAULT_MODEL):
        self.m = load_model(model_path)
        self.d = mujoco.MjData(self.m)
        m, O = self.m, mujoco.mjtObj
        ids = [mujoco.mj_name2id(m, O.mjOBJ_ACTUATOR, n) for n in ACT_NAMES]
        assert ids == list(range(6)), ids
        self.qadr = np.array([m.jnt_qposadr[m.actuator_trnid[a, 0]] for a in range(6)])
        self.dadr = np.array([m.jnt_dofadr[m.actuator_trnid[a, 0]] for a in range(6)])
        self.torso = mujoco.mj_name2id(m, O.mjOBJ_BODY, "torso")
        self.gyro_adr = int(m.sensor_adr[mujoco.mj_name2id(m, O.mjOBJ_SENSOR, "imu_gyro")])
        self.key = mujoco.mj_name2id(m, O.mjOBJ_KEY, "stand")
        self.lim = m.actuator_forcerange[:, 1].copy()
        self.decim = int(round(0.01 / m.opt.timestep))
        # the six base joints (base_x .. base_yaw) of the torso: "held" pins them, like the
        # operator's stand in BalanceRL/env.py and like a tight harness on the real robot
        base = [j for j in range(m.njnt) if m.jnt_bodyid[j] == self.torso]
        self.base_q = np.array([m.jnt_qposadr[j] for j in base])
        self.base_d = np.array([m.jnt_dofadr[j] for j in base])
        self.held = False
        self.reset()

    def reset(self):
        mujoco.mj_resetDataKeyframe(self.m, self.d, self.key)
        self.d.qvel[:] = 0.0
        mujoco.mj_forward(self.m, self.d)
        self.tau = np.zeros(6)
        self.hold_q = self.d.qpos[self.base_q].copy()

    def sense(self):
        d = self.d
        R = d.xmat[self.torso].reshape(3, 3)
        grav = R.T @ np.array([0.0, 0.0, -1.0])
        gyro = d.sensordata[self.gyro_adr:self.gyro_adr + 3].copy()
        return d.qpos[self.qadr].copy(), d.qvel[self.dadr].copy(), self.tau.copy(), grav, gyro

    def pitch_deg(self):
        g = self.sense()[3]
        return float(np.degrees(np.arctan2(g[0], -g[2])))

    def height(self):
        return float(self.d.xpos[self.torso][2])

    def tick(self, cmds, delay_ms):
        """One 10 ms tick. cmds: list of the last commands (newest last) as (target, kp, kd)."""
        for k in range(self.decim):
            # the command issued at tick start is live once its age reaches delay_ms
            age_ticks = 0 if k >= delay_ms else int(np.ceil((delay_ms - k) / 10.0))
            tgt, kp, kd = cmds[max(0, len(cmds) - 1 - age_ticks)]
            q, qd = self.d.qpos[self.qadr], self.d.qvel[self.dadr]
            tau = np.clip(kp * (tgt - q) - kd * qd, -self.lim, self.lim)
            self.d.ctrl[:] = tau
            mujoco.mj_step(self.m, self.d)
            if self.held:
                self.d.qpos[self.base_q] = self.hold_q
                self.d.qvel[self.base_d] = 0.0
            self.tau = tau


def run(bundle, seconds=10.0, delay_ms=12.0, frozen=None, push=None, held=False,
        model_path=DEFAULT_MODEL, frame=None):
    """Run the policy (frozen=None) or a constant (target, kp, kd) after the policy's own target
    filter (frozen = 18-dim action fed through the deployed controller via override_action).
    frame = (target, kp, kd) sent as is every tick, bypassing the controller (kp/kd outside the
    policy's range, e.g. kd 0). push = (t_s, duration_s, force_x_N) on the torso (or, held, a
    joint-space kick is not modelled). held = torso pinned. Returns a dict of per-tick arrays."""
    p = Plant(model_path)
    p.held = held
    c = PolicyControllerBalance(bundle)
    q, qd, tau, g, w = p.sense()
    c.start(q, qd, tau, g, w)
    nominal = (c.nominal.copy(), c.kp0.copy(), c.kd0.copy())
    cmds = [nominal] * 3
    log = {k: [] for k in ("t", "q", "qd", "tau", "grav", "gyro", "action", "target", "kp", "kd",
                           "pitch", "height")}
    n = int(round(seconds / 0.01))
    for i in range(n):
        q, qd, tau, g, w = p.sense()
        o = c.step(q, qd, tau, g, w, override_action=frozen)
        new = ((np.asarray(o.target), np.asarray(o.kp), np.asarray(o.kd)) if frame is None
               else tuple(np.asarray(v, float) for v in frame))
        cmds = cmds[-2:] + [new]
        if push is not None and push[0] <= i * 0.01 < push[0] + push[1]:
            p.d.xfrc_applied[p.torso, 0] = push[2]
        else:
            p.d.xfrc_applied[p.torso, :] = 0.0
        p.tick(cmds, delay_ms)
        for k, v in (("t", i * 0.01), ("q", q), ("qd", qd), ("tau", tau), ("grav", g),
                     ("gyro", w), ("action", np.asarray(o.residual)), ("target", new[0]),
                     ("kp", new[1]), ("kd", new[2]), ("pitch", p.pitch_deg()), ("height", p.height())):
            log[k].append(np.array(v, float))
    return {k: np.array(v) for k, v in log.items()}


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bundle")
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--delay-ms", type=float, default=12.0)
    a = ap.parse_args()
    r = run(Bundle.load(a.bundle), a.seconds, a.delay_ms)
    print("final pitch %.2f deg, height %.3f m, min height %.3f" % (r["pitch"][-1], r["height"][-1],
                                                                    r["height"].min()))
