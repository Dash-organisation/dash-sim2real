#!/usr/bin/env python3
"""Turn one policy step into FROZEN-STEP bundles for the Kd validation test (numpy only).

A frozen-step bundle is the source bundle with its action head replaced by a constant:

    act_w = 0, act_b = a*          ->  the network outputs a* on every tick, whatever it observes

so the deployed controller (controller/deploy/controller_balance.py, unchanged) turns it into the
SAME MIT frame on every tick: target = nominal + q_scale a*[0:6] (after the bundle's 80 ms target
filter), kp = gain_map(a*[6:12]), kd = gain_map(a*[12:18]). The daemon runs it as any v3 balance
bundle: same arm checks, same approach to the stance, same workspace guard, same governor, same
dead-man, same 10 s cap and the same run log (target, kp, kd, pos, vel, tau, amps per tick). No
robot code changes. The estimator and hidden layers are zeroed too (unused once act_w = 0), which
makes each file ~30 kB instead of 1 MB.

    python tools/kd_validation/make_frozen_bundles.py            # step_standing04_xshape_best.json
    python tools/kd_validation/make_frozen_bundles.py --step <json> --out controller/deploy/bundles

The variants (VARIANTS below) keep the step's leg target and change only the gains of the four leg
joints (cam_L, thigh_L, cam_R, thigh_R). The two hip rolls get the bundle's NOMINAL frame
(--abduction nominal, the default: target nominal_ctrl, kp drive_kp 120, kd drive_kd 4, i.e. action
0): in every quiet stance of standing04_xshape one hip roll is nearly limp (kp 20, kd 0.3 -- searched
over 48 pushed sim runs, 2026-11-23), which would let the robot lean sideways in the harness, and
the abduction is not what the test is about. --abduction step keeps the step's hip-roll frame. Each bundle is checked through the deployed
controller before it is written: after 1 s its frame must be the declared one to 1e-4.
"""
import argparse
import copy
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "controller", "deploy"))

from bundle import Bundle  # noqa: E402
from controller_balance import PolicyControllerBalance  # noqa: E402

LEG = [1, 2, 4, 5]          # cam_L, thigh_L, cam_R, thigh_R (model order)
ABD = [0, 3]                # hip_roll_L, hip_roll_R
PREFIX = "kdval"

# name -> (leg kd or None = the step's, leg kp scale), in the order the test runs them
VARIANTS = [
    ("A_step", None, 1.0),      # the policy step's leg frame exactly as the policy sent it
    ("B_kd5", 5.0, 1.0),        # all four leg joints at kd 5 (the driver's and the policy's max)
    ("C_kd2", 2.0, 1.0),
    ("D_kd1", 1.0, 1.0),
    ("E_kd05", 0.5, 1.0),
    ("F_kd5_kphalf", 5.0, 0.5),  # kd 5, leg kp halved: does kp matter too?
]


def gain_to_action(k, k0, k_lo, k_hi):
    """Inverse of controller_balance.gain_map, element-wise."""
    k = np.asarray(k, float)
    k0 = np.broadcast_to(np.asarray(k0, float), k.shape)
    if np.any(k < k_lo - 1e-9) or np.any(k > k_hi + 1e-9):
        raise ValueError("gain {} outside the bundle's range [{}, {}]".format(k, k_lo, k_hi))
    a = np.zeros_like(k)
    up, dn = k > k0, k < k0
    a[up] = np.log(k[up] / k0[up]) / np.log(k_hi / k0[up])
    a[dn] = np.log(k[dn] / k0[dn]) / np.log(k0[dn] / k_lo)
    return a


def frame_to_action(b, target, kp, kd):
    a = b.a
    g = b.meta["gains"]
    qa = (np.asarray(target, float) - a["nominal_ctrl"]) / a["q_scale"]
    if np.any(np.abs(qa) > 1.0) or np.any(target < a["q_lo"]) or np.any(target > a["q_hi"]):
        raise ValueError("target {} is outside what the action can reach".format(target))
    return np.concatenate([qa, gain_to_action(kp, a["drive_kp"], g["kp_lo"], g["kp_hi"]),
                           gain_to_action(kd, a["drive_kd"], g["kd_lo"], g["kd_hi"])])


def frozen_arrays(b, action):
    arr = {k: np.array(v, copy=True) for k, v in b.a.items()}
    for k in list(arr):
        if k.startswith(("est_", "pi_")):
            arr[k] = np.zeros_like(arr[k])
    arr["act_w"] = np.zeros_like(arr["act_w"])
    arr["act_b"] = np.asarray(action, np.float32)
    return arr


def check(path, target, kp, kd, ticks=100):
    """Run the written bundle through the deployed controller with a moving, noisy fake robot and
    return the worst deviation of its frame from the declared one after `ticks` ticks."""
    b = Bundle.load(path)
    c = PolicyControllerBalance(b)
    rng = np.random.default_rng(0)
    q0 = np.asarray(b["default_motor_pos"], float)
    c.start(q0, np.zeros(6), np.zeros(6), np.array([0.0, 0.0, -1.0]), np.zeros(3))
    for i in range(ticks):
        o = c.step(q0 + 0.02 * rng.standard_normal(6), rng.standard_normal(6),
                   10 * rng.standard_normal(6), np.array([0.05 * np.sin(i / 7), 0.0, -1.0]),
                   rng.standard_normal(3))
    return max(float(np.abs(o.target - target).max()), float(np.abs(o.kp - kp).max()),
               float(np.abs(o.kd - kd).max()))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--step", default=os.path.join(HERE, "step_standing04_xshape_best.json"))
    ap.add_argument("--out", default=os.path.join(ROOT, "controller", "deploy", "bundles"))
    ap.add_argument("--abduction", choices=("nominal", "step"), default="nominal")
    a = ap.parse_args()
    step = json.load(open(a.step))
    src = Bundle.load(os.path.join(ROOT, "controller", "deploy", "bundles", step["bundle"]))
    T = np.asarray(step["frame"]["target_rad"], float)
    KP = np.asarray(step["frame"]["kp"], float)
    KD = np.asarray(step["frame"]["kd"], float)
    if a.abduction == "nominal":
        T[ABD] = src["nominal_ctrl"][ABD]
        KP[ABD] = src["drive_kp"][ABD]
        KD[ABD] = src["drive_kd"][ABD]
    # the step's own action must give back the step's frame (no rounding surprise)
    a_step = frame_to_action(src, T, KP, KD)
    rows = []
    for name, kd_leg, kp_scale in VARIANTS:
        kp, kd = KP.copy(), KD.copy()
        if kd_leg is not None:
            kd[LEG] = kd_leg
        kp[LEG] = kp[LEG] * kp_scale
        act = a_step if (kd_leg is None and kp_scale == 1.0) else frame_to_action(src, T, kp, kd)
        if a.abduction == "nominal":
            assert np.allclose(act[[0, 3, 6, 9, 12, 15]], 0.0, atol=1e-6), "abduction != action 0"
        meta = copy.deepcopy(src.meta)
        meta["run"] = "{}_{}".format(PREFIX, name)
        meta["frozen_step"] = {
            "what": "constant action: act_w = 0, act_b = action. NOT a balance controller -- no "
                    "feedback at all. Kd validation test, docs/sim2real/DASH01_plan_test_Kd_2026-11-24.md",
            "source_bundle": step["bundle"], "source_run": step.get("run"),
            "picked_t_s": step["picked_t_s"], "variant": name,
            "leg_kd": kd_leg, "leg_kp_scale": kp_scale, "abduction": a.abduction,
            "target_rad": [round(float(v), 5) for v in T],
            "kp": [round(float(v), 3) for v in kp], "kd": [round(float(v), 4) for v in kd],
        }
        path = os.path.join(a.out, "{}_{}.npz".format(PREFIX, name))
        arrays = frozen_arrays(src, act)
        Bundle(arrays, meta)                                  # validate before writing
        meta = dict(meta, bundle_version=src.version)
        np.savez_compressed(path, meta=np.array(json.dumps(meta, sort_keys=True)),
                            **{k: np.asarray(v) for k, v in arrays.items()})
        err = check(path, T, kp, kd)
        if err > 1e-4:
            raise SystemExit("{}: frame off by {:.2e} after 1 s".format(path, err))
        rows.append((name, kp, kd, os.path.getsize(path), err))
    np.set_printoptions(precision=2, suppress=True, linewidth=140)
    print("target (deg, model order hip_roll_L cam_L thigh_L hip_roll_R cam_R thigh_R):",
          np.degrees(T))
    for name, kp, kd, size, err in rows:
        print("{}_{:14s} kp {}  kd {}  {:6d} B  check {:.1e}".format(PREFIX, name, kp, kd, size, err))


if __name__ == "__main__":
    main()
