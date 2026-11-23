"""Build deploy_map.json -- the normalized-degrees -> model-radians map run_policy.py requires.

    python controller/deploy/make_deploy_map.py --out controller/deploy/deploy_map.json \
        --when 2026-11-16 [--verified]

THE OFFSETS COME FROM THE STANDING POSE, NOT FROM THE CAD HOMING
----------------------------------------------------------------
The convention is `model = sign * norm + offset` (jointmap.JointMap). The daemon's standing pose,
STAND_POSE_DEG (controller/deploy/stand_pose.py), is the MJCF v8 xshape qpos0: with every joint
trim at 0 it must map to 0 rad on all six joints, so

    offset_j = -sign_j * STAND_POSE_DEG_j

Until 2026-11-16 this tool transcribed fklut's model_map.json and a CAD homing correspondence
instead; for the xshape that put the offsets 20-25 deg out, and the three standing04_xshape runs
were stopped during the approach with hip roll 12 deg off its target. Deriving the offsets from
the one pose the daemon and the policy already agree on removes that second source of truth.

THE SIGNS
---------
Not derivable from a pose: a sign error and an offset error read the same at a single pose. They
are carried over from the existing map (`--signs-from`, default the --out file), falling back to
DEFAULT_SIGNS below, which is the 2026-09-24 map:

    abduction  left +1 / right -1   (hip_roll_L and hip_roll_R share the axis (+1, 0, 0); the robot
                                     reads + outward on both legs, so the right is inverted)
    cam        left +1 / right -1   (the MJCF right cam/thigh turn about (0, -1, 0))
    thigh      left -1 / right +1

They must be re-checked with jointmap.check_direction, robot hung and limp, on all six joints --
abduction first, it has never been checked cleanly. Only then pass --verified.

RE-RUN THIS AFTER EVERY CHANGE TO STAND_POSE_DEG OR TO THE ZERO POSE. Every drive re-randomises its
raw origin on a power cycle and the webui calibration is re-captured with it; this map describes
the normalized frame on top of it, so a re-zero in a different pose makes it stale -- the same rule
JointMap.invalidate() states.
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import jointmap                                                        # noqa: E402
from stand_pose import STAND_POSE_DEG                                  # noqa: E402

DEFAULT_SIGNS = {"left.abd": +1.0, "left.cam": +1.0, "left.thigh": -1.0,
                 "right.abd": -1.0, "right.cam": -1.0, "right.thigh": +1.0}
NOTE = ("offset = -sign * STAND_POSE_DEG ({:+.2f}): the standing pose with trims at 0 IS the MJCF "
        "v8 xshape qpos0, so it maps to 0 rad in the model. Sign {}.")


def load_signs(path):
    """Signs from an existing deploy_map.json, or DEFAULT_SIGNS if there is none."""
    if path and os.path.exists(path):
        with open(path, "r", encoding="utf-8-sig") as f:
            joints = json.load(f).get("joints", {})
        return {n: float(joints.get(n, {}).get("sign", DEFAULT_SIGNS[n])) for n in DEFAULT_SIGNS}, path
    return dict(DEFAULT_SIGNS), "DEFAULT_SIGNS"


def build(signs, when=None, verified=False, sign_note="carried over -- re-run check_direction",
          kt_joint=None, kt_efficiency=jointmap.KT_EFFICIENCY):
    """signs + STAND_POSE_DEG -> a JointMap whose offsets put the standing pose at model 0 rad."""
    entries = {}
    for n in jointmap.MODEL_TO_MOTOR:
        s = float(signs[n])
        if s not in (-1.0, 1.0):
            raise SystemExit("{}: sign must be +1 or -1, got {}".format(n, s))
        entries[n] = {
            "sign": s,
            "offset_deg": round(-s * float(STAND_POSE_DEG[n]), 4) + 0.0,
            "erpm_per_rad_s": None,
            "verified": bool(verified),
            "verified_when": when,
            "note": NOTE.format(STAND_POSE_DEG[n], sign_note),
        }
    return jointmap.JointMap(entries, kt_joint=kt_joint, kt_efficiency=kt_efficiency)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="controller/deploy/deploy_map.json")
    ap.add_argument("--signs-from", default=None,
                    help="deploy_map.json to take the signs (and kt) from; default: --out")
    ap.add_argument("--when", default=None, help="stamp written into verified_when")
    ap.add_argument("--verified", action="store_true",
                    help="mark all six joints verified: only after check_stance (1 deg) and "
                         "check_direction have passed on the robot")
    a = ap.parse_args()

    src = a.signs_from or a.out
    signs, where = load_signs(src)
    kt, eff = None, jointmap.KT_EFFICIENCY
    if os.path.exists(src):
        old = jointmap.JointMap.load(src)
        kt, eff = old.kt_joint, old.kt_efficiency
    note = ("checked with check_direction" if a.verified
            else "from {} -- re-run check_direction".format(os.path.basename(where)))
    jm = build(signs, when=a.when, verified=a.verified, sign_note=note,
               kt_joint=kt, kt_efficiency=eff)

    print("signs from : {}".format(where))
    print("%-12s %6s %10s %12s" % ("joint", "sign", "stand_deg", "offset_deg"))
    for n in jointmap.MODEL_TO_MOTOR:
        e = jm.e[n]
        print("%-12s %+6.0f %+10.2f %+12.2f" % (n, e["sign"], STAND_POSE_DEG[n], e["offset_deg"]))
    ok, why = jm.check_ready()
    jm.save(a.out)
    print("\nwrote {}".format(a.out))
    if not ok:
        print("!! " + why)
        print("   run check_stance + check_direction on the robot, then re-run with --verified")
        return 1
    print("RE-RUN after any change to STAND_POSE_DEG or to the zero pose.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
