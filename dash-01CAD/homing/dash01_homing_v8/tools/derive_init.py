#!/usr/bin/env python3
"""Rebuild dash01_homing_v8.xml: the v8 (xshape) export re-posed so that qpos = 0 is the init pose
of dash01_homing_v7 (same hip / cam / thigh orientations = same motor angles), loops solved.

    cd dash-01CAD/homing/dash01_homing_v8
    python tools/derive_init.py ../dash01_homing_v7/dash01_homing_v7.xml EXPORT_assembly-v8_closed.xml dash01_homing_v8.xml

Needs mujoco and scipy. Only body pos/euler are rewritten (joints, sites, inertials untouched);
the exporter's "thight" body/geom names are renamed "thigh" so the twin's motor regexes match.
"""
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot

V7_MOTORS = {"R": ["Revolute-2", "Revolute-30", "Revolute-27"], "L": ["Revolute-3", "Revolute-38", "Revolute-33"]}
V8 = {"R": dict(motors=["Revolute-2", "Revolute-30", "Revolute-40"], knee=["Revolute-29", "Revolute-41"],
                ach=["Revolute-42", "Revolute-39"]),
      "L": dict(motors=["Revolute-3", "Revolute-38", "Revolute-43"], knee=["Revolute-37", "Revolute-47"],
                ach=["Revolute-46", "Revolute-45"])}


def jnt(m, n):
    return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)


def rel_rot(m, d, b):
    p = m.body_parentid[b]
    return Rot.from_matrix(d.xmat[p].reshape(3, 3)).inv() * Rot.from_matrix(d.xmat[b].reshape(3, 3))


def loop_res(m, d, name):
    e = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_EQUALITY, name)
    return d.site_xpos[m.eq_obj1id[e]] - d.site_xpos[m.eq_obj2id[e]]


def solve(m, d, joints, eq):
    idx = [m.jnt_qposadr[jnt(m, n)] for n in joints]

    def f(x):
        d.qpos[idx] = x
        mujoco.mj_kinematics(m, d)
        return loop_res(m, d, eq)
    f(least_squares(f, d.qpos[idx].copy(), xtol=1e-14, ftol=1e-14, gtol=1e-14).x)
    return np.linalg.norm(loop_res(m, d, eq))


def main(p7, p8, out):
    m7 = mujoco.MjModel.from_xml_path(p7); d7 = mujoco.MjData(m7); mujoco.mj_kinematics(m7, d7)
    m8 = mujoco.MjModel.from_xml_path(p8); d8 = mujoco.MjData(m8); mujoco.mj_kinematics(m8, d8)
    target = {}
    for s in "LR":
        for n7, n8 in zip(V7_MOTORS[s], V8[s]["motors"]):
            j7, j8 = jnt(m7, n7), jnt(m8, n8)
            ax = m8.jnt_axis[j8] / np.linalg.norm(m8.jnt_axis[j8])
            rv = (rel_rot(m8, d8, m8.jnt_bodyid[j8]).inv() * rel_rot(m7, d7, m7.jnt_bodyid[j7])).as_rotvec()
            assert np.linalg.norm(rv - (rv @ ax) * ax) < 1e-6, f"{n8}: v7 orientation not reachable about the axis"
            target[m8.jnt_qposadr[j8]] = rv @ ax
            print(f"{n8:12s} export -> init {np.degrees(rv @ ax):+9.3f} deg")
    worst = 0.0
    for t in np.linspace(0, 1, 121)[1:]:        # continuation keeps the CAD assembly branch
        for a, q in target.items():
            d8.qpos[a] = t * q
        for s in "LR":
            worst = max(worst, solve(m8, d8, V8[s]["knee"], f"loop_knee_{s}"),
                        solve(m8, d8, V8[s]["ach"], f"loop_achilles_{s}"))
    print(f"max loop residual on the way: {worst * 1e3:.2e} mm")
    tree = ET.parse(p8); root = tree.getroot(); root.set("model", "dash01_homing_v8")
    for b in root.iter("body"):
        i = mujoco.mj_name2id(m8, mujoco.mjtObj.mjOBJ_BODY, b.get("name")); p = m8.body_parentid[i]
        Rp = d8.xmat[p].reshape(3, 3)
        b.set("pos", " ".join(repr(float(v)) for v in Rp.T @ (d8.xpos[i] - d8.xpos[p])))
        R = Rp.T @ d8.xmat[i].reshape(3, 3)
        b.set("euler", " ".join(repr(float(v)) for v in Rot.from_matrix(R).as_euler("xyz")))  # MJCF "XYZ"
    for el in root.iter():
        if el.tag in ("body", "geom") and el.get("name"):
            el.set("name", el.get("name").replace("thight", "thigh"))
    ET.indent(tree, "    ")
    with open(out, "w") as f:
        f.write('<?xml version="1.0" ?>\n' + ET.tostring(root, encoding="unicode") + "\n")
    m = mujoco.MjModel.from_xml_path(out); d = mujoco.MjData(m); mujoco.mj_kinematics(m, d)
    err = max(np.linalg.norm(loop_res(m, d, f"loop_{k}_{s}")) for k in ("knee", "achilles") for s in "LR")
    print(f"wrote {out}; loops at qpos 0: {err * 1e3:.2e} mm; body pose error {np.abs(d.xpos - d8.xpos).max() * 1e3:.1e} mm")


if __name__ == "__main__":
    main(*sys.argv[1:4])
