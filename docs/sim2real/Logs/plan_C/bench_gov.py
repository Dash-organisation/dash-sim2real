"""Pure compute on one idle core: ctrl.step (in process) and SafetyGovernor.step, real bundle. No CAN."""
import os, sys, time
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
R = os.path.expanduser("~/running_robot/robot")
sys.path[:0] = [os.path.join(R, "fixed_gait", "webui"), os.path.join(R, "deploy")]
import numpy as np
from bundle import Bundle
from controller_balance import PolicyControllerBalance
import safety as SAFE
os.sched_setaffinity(0, {3})
b = Bundle.load(os.path.join(R, "fixed_gait/webui/data/policies/standing04_xshape_best.npz"))
pos = np.asarray(b["default_motor_pos"], float); z = np.zeros(6)
grav = np.array([0.0, 0.0, -1.0]); gyro = np.zeros(3)
ctrl = PolicyControllerBalance(b); ctrl.start(pos, z, z, grav, gyro)
lim = SAFE.Limits.from_bundle(b, persist_ticks=20)
gov = SAFE.SafetyGovernor(lim, 0.01)
rng = np.random.default_rng(0)
pc = time.perf_counter_ns
def bench(fn, n=1500):
    for _ in range(50): fn()
    ts = []
    for _ in range(n):
        t = pc(); fn(); ts.append(pc() - t)
    ts.sort(); return ts[len(ts)//2]/1e6, ts[int(.9*len(ts))]/1e6
out = {}
def c():
    p = pos + rng.normal(0, 0.01, 6)
    out["o"] = ctrl.step(p, z, z, grav, gyro)
print("ctrl.step        p50/p90 %.2f / %.2f ms" % bench(c))
o = out["o"]
tgt = getattr(o, "target", o[0] if isinstance(o, tuple) else o)
kp = getattr(o, "kp", np.full(6, 20.0)); kd = getattr(o, "kd", np.full(6, 0.5))
def g():
    gov.step(np.asarray(tgt, float), np.asarray(kp, float), np.asarray(kd, float),
             pos + rng.normal(0, 0.01, 6), z, grav, gyro, telemetry_age=0.005, deadman_age=0.1,
             drive_temp=np.full(6, 35.0), drive_err=np.zeros(6, int))
print("governor.step    p50/p90 %.2f / %.2f ms   (stop=%s)" % (*bench(g), gov.stop))
print("python", sys.version.split()[0], "numpy", np.__version__)
