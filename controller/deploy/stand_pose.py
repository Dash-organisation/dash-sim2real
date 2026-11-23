"""The standing pose, in normalized degrees -- shared by the daemon and make_deploy_map.py.

Normalized degrees, zero = the HOMING POSE the drives are zeroed in
(dash-01CAD/homing/dash01_homing_v8, README.md there: hips 25 deg outward at their stops; same
motor angles as dash01_homing_v7, only the achilles linkage moved).

It is the v8 (xshape) export pose (dash-01CAD/homing/dash01_homing_v8/EXPORT_assembly-v8_closed.xml):
hips straight under the torso, both soles flat and level at z -877 mm (x -90..+189 mm), CoM
x +39 mm, y +0.2 mm, loops closed to 1e-11 m in MuJoCo with the CAD masses (motors not included).
Each value is -sign * (export qpos at the init pose), per side: the export's two legs differ by
<= 0.27 deg and that difference is kept rather than averaged away.

This pose IS the MJCF v8 xshape qpos0. So deploy_map.json's offsets follow from it directly:
with trims at 0, STAND_POSE_DEG must map to 0 rad in the model on all six joints, i.e.

    offset_deg_j = -sign_j * STAND_POSE_DEG_j

(make_deploy_map.py derives them that way). Lives here, not in the daemon, so that the desktop
tool does not have to import the whole CAN stack to read six numbers.
"""

STAND_POSE_DEG = {"left.abd": -25.0, "left.cam": 55.47, "left.thigh": 17.11,
                  "right.abd": -25.0, "right.cam": 55.25, "right.thigh": 16.84}
