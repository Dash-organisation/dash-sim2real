#!/usr/bin/env python3
"""Take ONE policy step from a simulated run where the policy stands, and check in simulation what
freezing it, and changing its kd, does. Mac-side (needs `mujoco`).

    python tools/kd_validation/pick_step.py                       # standing04_xshape_best
    python tools/kd_validation/pick_step.py --bundle ... --seconds 20 --window 10 20

Writes tools/kd_validation/step_<bundle>.json: the 18-dim action of the picked tick and the MIT
frame it makes (target, kp, kd per joint, model order hip_roll_L, cam_L, thigh_L, hip_roll_R,
cam_R, thigh_R), plus the simulation checks below. make_frozen_bundles.py turns it into bundles.

The picked tick is the one closest (in action space) to the median action over the window: a
real step of a run that stands, not an average of steps.

Checks, all on the nominal plant, no noise, 12 ms command delay:
  1. the policy itself, free standing (does it stand, how much does its frame move);
  2. the frozen frame, free standing (no feedback any more: does it still stand?);
  3. the frozen frame with the torso pinned (the harness case), cam/thigh kd 5 / 2 / 1 / 0.5 / 0;
  4. the full policy with its cam/thigh kd output capped at 3 / 2 / 1, under +-30 N torso pushes:
     would a kd cap at deployment break the policy?
"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import sim_closed_loop as SIM  # noqa: E402
import controller_balance as CB  # noqa: E402  (on the path via sim_closed_loop)

LEG = [1, 2, 4, 5]          # cam_L, thigh_L, cam_R, thigh_R


def r3(x):
    return [round(float(v), 4) for v in np.ravel(x)]


def fall_time(x, tilt_deg=20.0):
    i = np.flatnonzero(np.abs(x["pitch"]) > tilt_deg)
    return None if i.size == 0 else round(float(x["t"][i[0]]), 2)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bundle", default=os.path.join(SIM.ROOT, "controller", "deploy", "bundles",
                                                     "standing04_xshape_best.npz"))
    ap.add_argument("--model", default=SIM.DEFAULT_MODEL)
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--window", type=float, nargs=2, default=(10.0, 20.0))
    ap.add_argument("--push", type=float, default=None,
                    help="torso push (N, +x forward) for 0.1 s at t = 5 s before the window: the "
                         "policy settles into another quiet stance after it")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    b = SIM.Bundle.load(a.bundle)
    name = os.path.splitext(os.path.basename(a.bundle))[0]

    # 1. the policy
    push = None if a.push is None else (5.0, 0.1, a.push)
    if push is not None and a.window[0] < 6.0:
        raise SystemExit("--window must start after the push has settled (>= 6 s)")
    r = SIM.run(b, a.seconds, push=push, model_path=a.model)
    w = (r["t"] >= a.window[0]) & (r["t"] < a.window[1])
    idx = np.flatnonzero(w)
    med = np.median(r["action"][idx], 0)
    k = int(idx[np.argmin(((r["action"][idx] - med) ** 2).sum(1))])
    T, KP, KD = r["target"][k], r["kp"][k], r["kd"][k]
    policy = {
        "stands": bool(fall_time(r) is None), "final_pitch_deg": round(float(r["pitch"][-1]), 2),
        "height_m": round(float(r["height"][-1]), 3),
        "window_kp_mean": r3(r["kp"][idx].mean(0)), "window_kp_tick_to_tick_rms":
            r3(np.sqrt((np.diff(r["kp"][idx], axis=0) ** 2).mean(0))),
        "window_kd_mean": r3(r["kd"][idx].mean(0)),
        "window_kd_at_max_fraction": r3((r["kd"][idx] > 0.999 * float(b.meta["gains"]["kd_hi"])).mean(0)),
        "window_q_mean": r3(r["q"][idx].mean(0)), "window_tau_mean": r3(r["tau"][idx].mean(0)),
    }
    print("policy: stands %s, pitch %.2f deg; picked tick %d (t = %.2f s)"
          % (policy["stands"], policy["final_pitch_deg"], k, r["t"][k]))

    # 2. frozen, free standing
    x = SIM.run(b, 5.0, frame=(T, KP, KD), model_path=a.model)
    frozen_free = {"falls_after_s": fall_time(x), "pitch_at_1s_deg": round(float(x["pitch"][100]), 2)}
    print("frozen, free standing: falls after", frozen_free["falls_after_s"], "s")

    # 3. frozen, torso pinned, kd sweep on cam/thigh
    held = []
    for kdl in (None, 5.0, 2.0, 1.0, 0.5, 0.0):
        kd = KD.copy()
        if kdl is not None:
            kd[LEG] = kdl
        x = SIM.run(b, 3.0, held=True, frame=(T, KP, kd), model_path=a.model)
        q = np.degrees(x["q"])
        held.append({"kd_leg": "step" if kdl is None else kdl, "final_q_deg": r3(q[-1]),
                     "final_tau": r3(x["tau"][-1]),
                     "qd_rms_last_1s": float("%.2e" % np.sqrt((x["qd"][-100:] ** 2).mean()))})
        print("held, kd", held[-1]["kd_leg"], "q", held[-1]["final_q_deg"])

    # 4. kd cap on the full policy
    caps = []
    orig = CB.gain_map
    try:
        for cap in (None, 3.0, 2.0, 1.0):
            def gm(av, k0, lo, hi, cap=cap):
                g = orig(av, k0, lo, hi)
                is_kd = hi == float(b.meta["gains"]["kd_hi"])     # the kd map, not the kp one
                return np.minimum(g, cap) if (cap is not None and is_kd) else g
            CB.gain_map = gm
            row = {"kd_cap": cap}
            for push in (None, 30.0, -30.0):
                x = SIM.run(b, 10.0, push=None if push is None else (5.0, 0.1, push),
                            model_path=a.model)
                row["push_%s" % push] = {"falls_after_s": fall_time(x),
                                         "max_abs_pitch_deg": round(float(np.abs(x["pitch"]).max()), 2)}
            caps.append(row)
            print("kd cap", cap, row)
    finally:
        CB.gain_map = orig

    out = {
        "bundle": os.path.basename(a.bundle), "run": b.meta.get("run"),
        "model": os.path.relpath(a.model, SIM.ROOT),
        "sim": "tools/kd_validation/sim_closed_loop.py: nominal plant, no noise, 12 ms delay, 1 kHz",
        "push_before_window": None if push is None else {"t_s": 5.0, "duration_s": 0.1,
                                                          "force_x_n": a.push},
        "picked_tick": k, "picked_t_s": round(float(r["t"][k]), 2), "window_s": list(a.window),
        "joints": SIM.ACT_NAMES,
        "action": r3(r["action"][k]),
        "frame": {"target_rad": r3(T), "kp": r3(KP), "kd": r3(KD)},
        "state_at_tick": {"q_rad": r3(r["q"][k]), "tau_nm": r3(r["tau"][k]),
                          "pitch_deg": round(float(r["pitch"][k]), 2)},
        "checks": {"policy": policy, "frozen_free_standing": frozen_free,
                   "frozen_torso_held_kd_sweep": held, "policy_kd_cap_pushes": caps},
    }
    path = a.out or os.path.join(HERE, "step_{}{}.json".format(
        name, "" if a.push is None else "_push{:+.0f}N".format(a.push)))
    with open(path, "w") as f:
        json.dump(out, f, indent=1)
    print("wrote", path)


if __name__ == "__main__":
    main()
