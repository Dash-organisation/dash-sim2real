"""Signed clearance to the recorded safe workspace, for a STANDING policy run (2026-11-17).

Why a policy run needs more than JointLimits.validate. A balance policy's joint target is a PD
COMMAND, not a pose: standing, it sits a gravity deflection beyond where the joint actually rests,
because drive_kp * (target - pos) is the torque that carries the robot. For standing04_xshape that
deflection puts nominal_ctrl ~2-3 deg past the measured workspace edge (the tibia/knee stop, minus
the workspace's own margin), while the pose the robot really stands in (default_motor_pos) is
~2-3 deg inside it. Checking the command against the workspace refuses a robot that is nowhere
near the stop; checking nothing lets an unloaded leg run into it.

So a standing run is guarded on two numbers instead of one:
  * the MEASURED pose must keep a clearance of at least POLICY_WS_MEAS_MARGIN_DEG (1 deg) -- the
    leg itself never gets near the stop;
  * the COMMAND may lead past the edge, but by at most POLICY_WS_CMD_LEAD_DEG (8 deg: the
    training's standing deflection, 5.9 deg cam / 5.0 deg thigh, plus margin).
Both are signed distances in normalized degrees: positive inside the safe set, negative outside.

The knee region is a polygon (a boolean occupancy grid), so its clearance is a distance field,
computed once per arm from the grid: every offset in a disk of RADIUS_DEG is tested with one
shifted boolean AND over the grid (~450 numpy ops on a 265x119 grid: a fraction of a second on the
Pi, nothing at all per tick). Distances saturate at RADIUS_DEG -- nothing here needs to know
whether a pose is 12 or 40 deg from the edge. Per tick it is two array lookups per leg.

Resolution: a pose is read in the grid cell that holds it, and distances run between cell centres,
so a clearance is good to about one cell (~1 deg on the recorded grids) -- the same resolution the
workspace itself is checked at (JointLimits.check_knee), whose inside/outside it always agrees with
in sign. The 3 deg margin the workspace is eroded by at build time sits on top of that.

Pure numpy, no daemon import: tests build it from a synthetic JointLimits."""
import math

import numpy as np

RADIUS_DEG = 12.0          # the distance field saturates here (well beyond the 8 deg command lead)


def _neighbour(grid, dx, dy, fill):
    """out[i, j] = grid[i + dx, j + dy], `fill` where that falls off the grid."""
    nx, ny = grid.shape
    out = np.full(grid.shape, fill, dtype=bool)
    i0, i1 = max(0, -dx), min(nx, nx - dx)
    j0, j1 = max(0, -dy), min(ny, ny - dy)
    if i0 < i1 and j0 < j1:
        out[i0:i1, j0:j1] = grid[i0 + dx:i1 + dx, j0 + dy:j1 + dy]
    return out


def knee_clearance_field(grid, res_deg, radius_deg=RADIUS_DEG):
    """Signed clearance (deg) of every cell CENTRE to the edge of the safe set.

    Safe cell: +(d - res/2), d = distance to the nearest unsafe cell centre (off-grid is unsafe).
    Unsafe cell: -(d - res/2), d = distance to the nearest safe cell centre.
    So a safe cell touching an unsafe one reads +res/2 and its unsafe neighbour -res/2: the edge
    sits between them, at 0. Saturates at +-radius_deg."""
    G = np.asarray(grid, bool)
    r = int(math.ceil(radius_deg / res_deg))
    big = float(r + 1)
    d_unsafe = np.full(G.shape, big)          # for safe cells: distance (cells) to unsafe
    d_safe = np.full(G.shape, big)            # for unsafe cells: distance (cells) to safe
    offsets = sorted(((dx, dy) for dx in range(-r, r + 1) for dy in range(-r, r + 1)
                      if (dx or dy) and dx * dx + dy * dy <= r * r),
                     key=lambda o: o[0] * o[0] + o[1] * o[1])
    for dx, dy in offsets:
        dist = math.hypot(dx, dy)
        hit_u = G & _neighbour(~G, dx, dy, fill=True) & (d_unsafe > dist)
        d_unsafe[hit_u] = dist
        hit_s = ~G & _neighbour(G, dx, dy, fill=False) & (d_safe > dist)
        d_safe[hit_s] = dist
    half = 0.5
    field = np.where(G, (d_unsafe - half), -(d_safe - half)) * res_deg
    return np.clip(field, -radius_deg, radius_deg)


class WorkspaceClearance:
    """Signed clearance of a full normalized pose to a JointLimits-shaped workspace."""

    def __init__(self, limits, radius_deg=RADIUS_DEG):
        self.radius_deg = float(radius_deg)
        self.legs = {}
        for side, L in limits.legs.items():
            res = float(L["knee_grid_deg"])
            self.legs[side] = dict(
                field=knee_clearance_field(L["knee_grid"], res, radius_deg),
                c0=float(L["knee_cam_origin"]), t0=float(L["knee_thigh_origin"]), res=res,
                abd=(float(L["abd_safe"][0]), float(L["abd_safe"][1])))

    @staticmethod
    def supports(limits):
        """True for a real JointLimits (per-leg grids); test doubles without grids are not."""
        legs = getattr(limits, "legs", None)
        return isinstance(legs, dict) and bool(legs) and all(
            isinstance(L, dict) and "knee_grid" in L for L in legs.values())

    def knee(self, side, cam, thigh):
        L = self.legs[side]
        i = int(math.floor((cam - L["c0"]) / L["res"]))
        j = int(math.floor((thigh - L["t0"]) / L["res"]))
        f = L["field"]
        if not (0 <= i < f.shape[0] and 0 <= j < f.shape[1]):
            return -self.radius_deg                  # off the recorded grid: outside, far
        return float(f[i, j])

    def abd(self, side, abd):
        lo, hi = self.legs[side]["abd"]
        return float(min(abd - lo, hi - abd))

    def pose(self, norm):
        """(clearance_deg, where) for {motor: normalized deg}: the minimum over both legs' knee
        and abduction. `where` names the limiting one, e.g. "right knee"."""
        best, where = float("inf"), None
        for side in self.legs:
            for part, c in (("knee", self.knee(side, norm[side + ".cam"], norm[side + ".thigh"])),
                            ("abduction", self.abd(side, norm[side + ".abd"]))):
                if c < best:
                    best, where = c, "{} {}".format(side, part)
        return best, where
