#!/usr/bin/env python3
"""The control law in process vs. in the policy worker, in a simulated 100 Hz loop (plan B, step 4).

NO CAN, NO MOTORS, NO DAEMON. It loads a v3 bundle, runs the daemon's tick shape at 100 Hz --
observation, control law, then some Python "rest of the tick" standing in for the governor, the
workspace guards, the send and the log -- while background Python threads compete for the GIL the
way the daemon's IMU filter, black box and Flask threads do. Then the same with ctrl.step in the
worker process (policy_worker.py), waiting in conn.poll() like daemon._tick_policy does.

It answers: with this much competing Python in the interpreter, what does the control law cost the
tick, and does the loop hold its rate -- in process vs. in the worker?

    python3 controller/tools/bench_policy_worker.py                     # 5 s each, 2 load threads,
                                                                        # GIL switch 0.5 ms (server.py)
    python3 controller/tools/bench_policy_worker.py --load-threads 0    # no competition
    python3 controller/tools/bench_policy_worker.py --bundle path/to/standing04_xshape_best.npz

On the robot's Pi (daemon stopped or not -- nothing here touches the bus):
    cd ~/running_robot/robot && taskset -c 0,1 python3 tools/bench_policy_worker.py \\
        --bundle fixed_gait/webui/data/policies/standing04_xshape_best.npz --seconds 10
(`taskset -c 0,1` puts the "daemon" on the cores its systemd drop-in gives it; the worker pins
itself to 2-3.)
"""
import argparse
import os
import sys
import threading
import time

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")               # like server.py, before numpy

HERE = os.path.dirname(os.path.abspath(__file__))
CONTROLLER = os.path.dirname(HERE)
WEBUI = os.path.join(CONTROLLER, "fixed_gait", "webui")
DEPLOY = os.path.join(CONTROLLER, "deploy")
for _p in (WEBUI, DEPLOY):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np                                                 # noqa: E402

import policy_worker as PW                                          # noqa: E402
from bundle import Bundle                                           # noqa: E402
from controller_balance import PolicyControllerBalance             # noqa: E402

TICK_HZ = 100.0


def find_bundle():
    for d in (os.path.join(WEBUI, "data", "policies"), os.path.join(DEPLOY, "bundles")):
        if not os.path.isdir(d):
            continue
        for f in sorted(os.listdir(d)):
            if f.endswith(".npz"):
                path = os.path.join(d, f)
                try:
                    if Bundle.load(path).version == 3:
                        return path
                except Exception:                                   # noqa: BLE001
                    continue
    return None


def busy_us(us):
    """Pure-Python work for `us` microseconds: holds the GIL the whole time, like a filter step."""
    t_end = time.perf_counter() + us * 1e-6
    x = 0.0
    while time.perf_counter() < t_end:
        for i in range(50):
            x += i * 0.5
    return x


class Load:
    """N threads, each doing `work_us` of Python at `hz` -- the IMU filter is one such at 200 Hz."""

    def __init__(self, n, hz, work_us):
        self.stop = threading.Event()
        self.threads = [threading.Thread(target=self.run, args=(hz, work_us), daemon=True)
                        for _ in range(n)]

    def run(self, hz, work_us):
        period = 1.0 / hz
        nxt = time.perf_counter()
        while not self.stop.is_set():
            busy_us(work_us)
            nxt += period
            s = nxt - time.perf_counter()
            if s > 0:
                time.sleep(s)
            else:
                nxt = time.perf_counter()

    def __enter__(self):
        for t in self.threads:
            t.start()
        return self

    def __exit__(self, *a):
        self.stop.set()
        for t in self.threads:
            t.join(1.0)


def observations(b, n, seed=0):
    rng = np.random.default_rng(seed)
    pos0 = np.asarray(b["default_motor_pos"], float)
    for _ in range(n):
        yield (pos0 + rng.normal(0, 0.01, 6), rng.normal(0, 0.1, 6), rng.normal(0, 1.0, 6),
               np.array([0.0, 0.0, -1.0]) + rng.normal(0, 0.01, 3), rng.normal(0, 0.05, 3))


def loop(step, seconds, rest_us):
    """The daemon's loop shape: free-runs on a slip (next_t reset, never caught up)."""
    n_max = int(seconds * TICK_HZ) + 1
    ctrl_ms = np.zeros(n_max)
    tick_ms = np.zeros(n_max)
    late = 0
    slips = 0
    dt = 1.0 / TICK_HZ
    nxt = time.perf_counter()
    t0 = nxt
    n = 0
    while n < n_max:
        t_a = time.perf_counter()
        late += step(n)
        t_b = time.perf_counter()
        busy_us(rest_us)
        t_c = time.perf_counter()
        ctrl_ms[n] = (t_b - t_a) * 1e3
        tick_ms[n] = (t_c - t_a) * 1e3
        n += 1
        nxt += dt
        s = nxt - time.perf_counter()
        if s > 0:
            time.sleep(s)
        else:
            slips += 1
            nxt = time.perf_counter()
    el = time.perf_counter() - t0
    return {"hz": n / el, "ctrl": ctrl_ms[:n], "tick": tick_ms[:n], "late": late, "slips": slips,
            "n": n}


def run_inproc(b, obs, seconds, rest_us, pad_us, pad_steps):
    ctrl = PolicyControllerBalance(b)
    scratch = PolicyControllerBalance(b)
    o0 = obs[0]
    ctrl.start(o0[0], np.zeros(6), o0[2], o0[3], o0[4])
    scratch.start(o0[0], np.zeros(6), o0[2], o0[3], o0[4])

    def step(n):
        o = obs[n % len(obs)]
        ctrl.step(*o)
        if pad_us:
            busy_us(pad_us)
        for _ in range(pad_steps):
            scratch.step(*o)
        return 0
    return loop(step, seconds, rest_us)


def run_worker(path, obs, seconds, rest_us, deadline_ms, pad_us, pad_steps):
    c = PW.PolicyWorkerClient({"pad_us": pad_us, "pad_steps": pad_steps})
    ready = c.start(path, timeout=30.0)
    o0 = obs[0]
    assert c.begin(0, o0[0], np.zeros(6), o0[2], o0[3], o0[4], timeout=1.0) == PW.OK, c.why
    seq = [0]

    def step(n):
        seq[0] += 1
        t0 = time.perf_counter()
        c.post(seq[0], *obs[n % len(obs)])
        # the daemon does the rate count and the measured-pose guard here; nothing to do in a bench
        cmd, st = c.collect(seq[0], t0 + deadline_ms / 1e3)
        if st not in (PW.OK, PW.LATE):
            raise RuntimeError("worker {}: {}".format(st, c.why))
        return 1 if cmd is None else 0
    try:
        out = loop(step, seconds, rest_us)
    finally:
        c.stop(wait=True)
    out["ready"] = ready
    out["stale"] = c.stale
    return out


def pct(x, q):
    return float(np.percentile(x, q)) if len(x) else float("nan")


def report(name, r):
    print("  {:<10} {:6.1f} Hz  ctrl p50 {:5.2f} p90 {:5.2f} max {:6.2f} ms   "
          "tick p50 {:5.2f} p90 {:5.2f} max {:6.2f} ms   slips {:4d}  late {:4d}".format(
              name, r["hz"], pct(r["ctrl"], 50), pct(r["ctrl"], 90), r["ctrl"].max(),
              pct(r["tick"], 50), pct(r["tick"], 90), r["tick"].max(), r["slips"], r["late"]))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--bundle", help="a v3 bundle (default: the first one in data/policies or "
                                     "deploy/bundles)")
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--load-threads", type=int, default=2,
                    help="background Python threads competing for the GIL")
    ap.add_argument("--load-hz", type=float, default=200.0)
    ap.add_argument("--load-us", type=float, default=1500.0,
                    help="Python work per load-thread iteration, microseconds")
    ap.add_argument("--rest-us", type=float, default=3000.0,
                    help="Python work in the tick besides the control law (governor, guards, send, "
                         "log) -- 3 ms is daemon.POLICY_TICK_OVERHEAD_MS")
    ap.add_argument("--deadline-ms", type=float, default=6.0)
    ap.add_argument("--step-pad-us", type=float, default=0.0,
                    help="pure-Python work added to every control-law step, in BOTH modes, to give a "
                         "fast machine the Pi's step time (~3000 on a Mac stands in for the Pi 3B's "
                         "3.5 ms); leave 0 on the Pi")
    ap.add_argument("--step-repeat", type=int, default=0,
                    help="extra steps of a scratch copy of the control law per tick, in BOTH modes: "
                         "the real numpy call pattern, GIL releases included (~35 on an M-series "
                         "Mac stands in for the Pi's 3.5 ms); leave 0 on the Pi")
    ap.add_argument("--only", choices=("inproc", "worker"))
    ap.add_argument("--switch-interval-ms", type=float, default=0.5,
                    help="sys.setswitchinterval for this process: how long a thread that wants the "
                         "GIL back waits before forcing a hand-over. Default 0.5 ms, what server.py "
                         "runs the daemon at; 5 is CPython's own default")
    a = ap.parse_args(argv)
    sys.setswitchinterval(a.switch_interval_ms / 1e3)

    path = a.bundle or find_bundle()
    if path is None:
        sys.exit("no v3 bundle found -- pass --bundle")
    b = Bundle.load(path)
    if b.version != 3:
        sys.exit("{} is a v{} bundle; the worker serves v3".format(path, b.version))
    obs = list(observations(b, 500))

    print("bundle {}  (v{}, {:.0f} Hz)".format(os.path.basename(path), b.version, 1 / b.control_dt))
    print("python {}  numpy {}  cpus {}  OPENBLAS_NUM_THREADS={}".format(
        sys.version.split()[0], np.__version__, os.cpu_count(),
        os.environ.get("OPENBLAS_NUM_THREADS")))
    print("GIL switch interval {:.2f} ms; control-law step padded by {:.0f} us of Python and {} "
          "extra steps".format(sys.getswitchinterval() * 1e3, a.step_pad_us, a.step_repeat))
    print("load: {} thread(s) x {:.0f} us of Python at {:.0f} Hz; rest of tick {:.0f} us; "
          "worker deadline {:.1f} ms; {:.0f} s per mode".format(
              a.load_threads, a.load_us, a.load_hz, a.rest_us, a.deadline_ms, a.seconds))
    res = {}
    with Load(a.load_threads, a.load_hz, a.load_us):
        if a.only in (None, "inproc"):
            res["in-process"] = run_inproc(b, obs, a.seconds, a.rest_us, a.step_pad_us,
                                           a.step_repeat)
        if a.only in (None, "worker"):
            res["worker"] = run_worker(path, obs, a.seconds, a.rest_us, a.deadline_ms,
                                       a.step_pad_us, a.step_repeat)
    print()
    for k, r in res.items():
        report(k, r)
    if "worker" in res:
        rd = res["worker"]["ready"]
        print("\n  worker: pid {}  affinity {}  probe step p50 {:.2f} ms p90 {:.2f} ms  "
              "stale answers discarded {}".format(rd["pid"], rd["affinity"], rd["step_ms"],
                                                  rd["step_p90_ms"], res["worker"]["stale"]))
    print("\n  ctrl = the control law's share of the tick (in the worker: post + wait); tick = ctrl "
          "+ the rest; late = worker answers past the deadline (the daemon re-uses the previous "
          "command; 3 in a row soft-stop).")


if __name__ == "__main__":
    main()
