"""The policy's control law in its OWN process (plan B, 2026-11-18): the worker and its client.

WHY. 2026-11-17, standing04_xshape on the robot's Pi 3B: ctrl.step takes 3.5 ms alone and 9.2 ms
inside the daemon, because the daemon's one interpreter also runs the 200 Hz IMU filter, the black
box, Flask and the stream to uiproc, and the GIL lets one of them run at a time. The loop held 60-76
Hz instead of 100 and the governor stopped every run. Here ctrl.step runs in a second interpreter,
pinned to the cores the daemon does not use; the daemon sends the observation, does what does not
depend on the answer, and waits for the command in conn.poll() -- which releases the GIL.

SYNCHRONOUS ON PURPOSE. Tick N sends observation N and waits for command N, up to a deadline. The
policy was trained with 6-18 ms of randomised actuator delay; a "use last tick's command" pipeline
would add a whole 10 ms tick on top and leave that envelope. A command that misses the deadline is
replaced by the previous one for that tick (daemon.py counts it and soft-stops after a few in a
row), and the late answer is read and DISCARDED on the next collect, by sequence number.

This one file describes the wire format for both ends. It imports nothing from the daemon -- only
controller/deploy/ (the bundle and the control law), and only inside the worker.

THE PROTOCOL. multiprocessing.Pipe(duplex=True), send_bytes/recv_bytes. Every message is a 1-byte
type, then a payload. In the hot loop the payload is raw float64 (ndarray.tobytes()) of fixed
length -- no pickle. `seq` is an int64.

  daemon -> worker
    S  seq + obs      ctrl.start(pos, vel, tau, grav, gyro)          -> A + seq
    T  seq + obs      cmd = ctrl.step(pos, vel, tau, grav, gyro)    -> C + seq + cmd
    M  JSON {"name": one of METHODS, "args": [...]}  rare, off the hot path -> K + JSON {"result"}
    Q  -               clean exit

  worker -> daemon
    R  JSON {"step_ms", "step_p90_ms", "pid", "affinity", "version", ...}   once, after the probe
    A  seq             start() done
    C  seq + float64[CMD_N]  (CMD_LAYOUT)
    K  JSON            a method's return value
    E  UTF-8 text      an exception; the worker exits right after sending it

  obs = float64[OBS_N]: pos 6, vel 6, tau 6, grav 3, gyro 3 -- MODEL frame, actuator order, exactly
  what _tick_policy hands ctrl.step in process.

The worker never sees the CAN bus, the calibration, the disk (beyond the one bundle it loads) or the
network. Everything that keeps the robot safe stays in the daemon, unchanged.
"""
import contextlib
import json
import os
import sys
import threading
import time

# BEFORE numpy: the worker owns one core's worth of BLAS. In the daemon numpy is already imported
# (server.py sets the same variable first), so this only ever takes effect in the worker.
if "numpy" not in sys.modules:
    os.environ["OPENBLAS_NUM_THREADS"] = "1"

import numpy as np                                                    # noqa: E402

OBS_N = 24
METHODS = ("set_run", "set_speed", "zero_heading", "start_brake", "cancel_brake")
SUPPORTED_VERSIONS = (2, 3)          # what the worker can run; daemon.POLICY_WORKER_VERSIONS picks
WORKER_CPUS = (2, 3)                 # the daemon is pinned to 0-1 by its systemd drop-in
PROBE_WARMUP = 10
PROBE_TICKS = 40                     # = daemon.POLICY_PROBE_TICKS

# The command frame. Names are CommandBalance's (controller_balance.py) plus what the daemon reads
# off the controller itself (heading_deg(), braking) and the worker's own step time.
CMD_LAYOUT = (("target", 6), ("kp", 6), ("kd", 6), ("action", 18),
              ("phase", 1), ("freq", 1), ("saturated", 1), ("commit", 1), ("run_flag", 1),
              ("speed_cmd", 1), ("brake_frac", 1), ("vel_est", 3), ("heading_deg", 1),
              ("braking", 1), ("step_ms", 1))
CMD_N = sum(n for _, n in CMD_LAYOUT)
assert CMD_N == 49, CMD_N
_SLICES = {}
_o = 0
for _name, _n in CMD_LAYOUT:
    _SLICES[_name] = (_o, _o + _n)
    _o += _n
del _o, _name, _n

_HDR = 9                             # type byte + int64 seq
OBS_MSG_LEN = _HDR + 8 * OBS_N
CMD_MSG_LEN = _HDR + 8 * CMD_N


def _seq_bytes(seq):
    return np.int64(seq).tobytes()


def _seq_of(msg):
    return int(np.frombuffer(msg, np.int64, 1, 1)[0])


def pack_obs(kind, seq, pos, vel, tau, grav, gyro, out=None):
    """S or T message for one observation. `out` is an optional preallocated float64[OBS_N]."""
    o = np.empty(OBS_N) if out is None else out
    o[0:6] = pos
    o[6:12] = vel
    o[12:18] = tau
    o[18:21] = grav
    o[21:24] = gyro
    return kind + _seq_bytes(seq) + o.tobytes()


def unpack_obs(msg):
    if len(msg) != OBS_MSG_LEN:
        raise ValueError("observation message is {} bytes, expected {}".format(len(msg), OBS_MSG_LEN))
    o = np.frombuffer(msg, np.float64, OBS_N, _HDR).copy()
    return _seq_of(msg), o[0:6], o[6:12], o[12:18], o[18:21], o[21:24]


class WorkerCommand:
    """A command as the daemon reads it: CommandBalance's attribute names, plus the worker's own
    step time and the two things the daemon used to ask the controller for directly."""
    __slots__ = tuple(n for n, _ in CMD_LAYOUT) + ("seq",)

    def __init__(self, seq, v):
        self.seq = seq
        for name, (a, b) in _SLICES.items():
            setattr(self, name, v[a:b] if b - a > 1 else float(v[a]))
        self.saturated = bool(self.saturated)
        self.commit = bool(self.commit)
        self.braking = bool(self.braking)


def pack_cmd(seq, cmd, heading_deg, braking, step_ms, buf):
    """C message from a controller's command object, through a preallocated float64[CMD_N]."""
    def put(name, val, default=0.0):
        a, b = _SLICES[name]
        if val is None:
            buf[a:b] = default
        else:
            buf[a:b] = val
    put("target", cmd.target)
    put("kp", cmd.kp)
    put("kd", cmd.kd)
    act = getattr(cmd, "action", None)
    a, b = _SLICES["action"]
    if act is None:
        buf[a:b] = np.nan
    else:
        act = np.asarray(act, np.float64).ravel()
        buf[a:b] = np.nan
        buf[a:a + min(act.size, b - a)] = act[:b - a]
    put("phase", float(getattr(cmd, "phase", 0.0) or 0.0))
    put("freq", float(getattr(cmd, "freq", 0.0) or 0.0))
    put("saturated", 1.0 if getattr(cmd, "saturated", False) else 0.0)
    put("commit", 1.0 if getattr(cmd, "commit", False) else 0.0)
    put("run_flag", float(getattr(cmd, "run_flag", 0.0) or 0.0))
    put("speed_cmd", float(getattr(cmd, "speed_cmd", 0.0) or 0.0))
    put("brake_frac", float(getattr(cmd, "brake_frac", 0.0) or 0.0))
    ve = getattr(cmd, "vel_est", None)
    a, b = _SLICES["vel_est"]
    buf[a:b] = np.nan
    if ve is not None:
        ve = np.asarray(ve, np.float64).ravel()
        buf[a:a + min(ve.size, b - a)] = ve[:b - a]
    put("heading_deg", heading_deg)
    put("braking", 1.0 if braking else 0.0)
    put("step_ms", step_ms)
    return b"C" + _seq_bytes(seq) + buf.tobytes()


def unpack_cmd(msg):
    if len(msg) != CMD_MSG_LEN:
        raise ValueError("command message is {} bytes, expected {}".format(len(msg), CMD_MSG_LEN))
    v = np.frombuffer(msg, np.float64, CMD_N, _HDR).copy()
    return WorkerCommand(_seq_of(msg), v)


# ===================================================================== the worker process
def _deploy_dir():
    # controller/fixed_gait/webui/policy_worker.py -> controller/deploy (robot/deploy on the Pi)
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(os.path.dirname(os.path.dirname(here)), "deploy")


def _pin():
    """Pin to WORKER_CPUS where the OS can (Linux); anywhere else, carry on unpinned."""
    try:
        have = os.sched_getaffinity(0)
        want = set(WORKER_CPUS) & set(range(os.cpu_count() or 1))
        if want:
            os.sched_setaffinity(0, want)
        return sorted(os.sched_getaffinity(0)) if want else sorted(have)
    except (AttributeError, OSError):
        return None


def _build(bundle_path):
    d = _deploy_dir()
    if d not in sys.path:
        sys.path.insert(0, d)
    from bundle import Bundle
    b = Bundle.load(bundle_path)
    if b.version not in SUPPORTED_VERSIONS:
        raise ValueError("the policy worker runs v{} bundles; this one is v{}".format(
            "/v".join(str(v) for v in SUPPORTED_VERSIONS), b.version))
    if b.version == 3:
        from controller_balance import PolicyControllerBalance
        return b, PolicyControllerBalance(b)
    from controller_v2 import PolicyControllerV2
    return b, PolicyControllerV2(b)


def _probe(ctrl, b):
    """daemon._policy_probe, here: the step time of THIS process, which is the one that will run."""
    nu = int(b.meta.get("nu", 6))
    pos = np.asarray(b["nominal_ctrl"], float)
    zero6, grav, gyro = np.zeros(nu), np.array([0.0, 0.0, -1.0]), np.zeros(3)
    ctrl.start(pos, zero6, zero6, grav, gyro)
    for _ in range(PROBE_WARMUP):
        ctrl.step(pos, zero6, zero6, grav, gyro)
    ts = []
    for _ in range(PROBE_TICKS):
        t0 = time.perf_counter()
        ctrl.step(pos, zero6, zero6, grav, gyro)
        ts.append(time.perf_counter() - t0)
    ts.sort()
    return ts[len(ts) // 2] * 1e3, ts[int(len(ts) * 0.9)] * 1e3


def _busy(seconds):
    t_end = time.perf_counter() + seconds
    x = 0.0
    while time.perf_counter() < t_end:
        for i in range(50):
            x += i * 0.5
    return x


def worker_main(conn, bundle_path, opts=None):
    """The worker. Any exception goes back as E with its text, and the worker exits.

    opts (tests and the bench only): sleep_ms + sleep_from (step index from which every step
    sleeps) or sleep_steps (indices), raise_at (index of the step that raises), fail_load (raise at
    load), pad_us (pure-Python work added to every step) or pad_steps (that many extra steps of a
    scratch copy of the control law -- the real numpy call pattern, GIL releases included): the
    bench (tools/bench_policy_worker.py) uses them to give a fast machine the Pi 3B's step time."""
    opts = dict(opts or {})
    try:
        affinity = _pin()
        if opts.get("fail_load"):
            raise RuntimeError(str(opts["fail_load"]))
        b, ctrl = _build(bundle_path)
        step_ms, step_p90 = _probe(ctrl, b)
        ready = {"step_ms": step_ms, "step_p90_ms": step_p90, "pid": os.getpid(),
                 "affinity": affinity, "version": int(b.version),
                 "has_heading": bool(getattr(ctrl, "has_heading", False)),
                 "openblas_threads": os.environ.get("OPENBLAS_NUM_THREADS")}
        conn.send_bytes(b"R" + json.dumps(ready).encode())
        sleep_s = float(opts.get("sleep_ms", 0.0)) / 1e3
        sleep_from = opts.get("sleep_from")
        sleep_steps = set(opts.get("sleep_steps") or ())
        raise_at = opts.get("raise_at")
        pad_s = float(opts.get("pad_us", 0.0)) * 1e-6
        pad_steps = int(opts.get("pad_steps", 0))
        scratch = _build(bundle_path)[1] if pad_steps else None
        if scratch is not None:
            p0 = np.asarray(b["nominal_ctrl"], float)
            scratch.start(p0, np.zeros(6), np.zeros(6), np.array([0.0, 0.0, -1.0]), np.zeros(3))
        buf = np.empty(CMD_N)
        n_step = 0
        has_heading = bool(getattr(ctrl, "has_heading", False))
        while True:
            try:
                msg = conn.recv_bytes()
            except (EOFError, OSError):
                return                                   # the daemon went away: so do we
            kind = msg[:1]
            if kind == b"T":
                seq, pos, vel, tau, grav, gyro = unpack_obs(msg)
                if raise_at is not None and n_step == int(raise_at):
                    raise RuntimeError("test: the control law raised at step {}".format(n_step))
                t0 = time.perf_counter()
                cmd = ctrl.step(pos, vel, tau, grav, gyro)
                if pad_s:
                    _busy(pad_s)
                for _ in range(pad_steps):
                    scratch.step(pos, vel, tau, grav, gyro)
                heading = float(ctrl.heading_deg()) if has_heading else 0.0
                braking = bool(getattr(ctrl, "braking", False))
                ms = (time.perf_counter() - t0) * 1e3
                if sleep_s and ((sleep_from is not None and n_step >= int(sleep_from))
                                or n_step in sleep_steps):
                    time.sleep(sleep_s)
                n_step += 1
                conn.send_bytes(pack_cmd(seq, cmd, heading, braking, ms, buf))
            elif kind == b"S":
                seq, pos, vel, tau, grav, gyro = unpack_obs(msg)
                ctrl.start(pos, vel, tau, grav, gyro)
                n_step = 0
                conn.send_bytes(b"A" + _seq_bytes(seq))
            elif kind == b"M":
                req = json.loads(msg[1:].decode())
                name = req.get("name")
                if name not in METHODS:
                    raise ValueError("method {!r} is not one the worker forwards".format(name))
                res = getattr(ctrl, name)(*req.get("args", []))
                if isinstance(res, tuple):
                    res = list(res)
                if isinstance(res, (np.floating, np.integer, np.bool_)):
                    res = res.item()
                conn.send_bytes(b"K" + json.dumps({"result": res}).encode())
            elif kind == b"Q":
                return
            else:
                raise ValueError("unknown message type {!r}".format(kind))
    except BaseException as e:                       # noqa: BLE001 -- the text IS the report
        try:
            conn.send_bytes(b"E" + "{}: {}".format(type(e).__name__, e).encode("utf-8", "replace"))
        except Exception:                            # noqa: BLE001 -- nobody left to tell
            pass
    finally:
        try:
            conn.close()
        except Exception:                            # noqa: BLE001
            pass


# ===================================================================== the daemon's client
class PolicyWorkerError(RuntimeError):
    pass


# spawn re-runs the parent's __main__ in the child unless it is hidden (multiprocessing.spawn
# get_preparation_data). For the daemon that is server.py: Flask, the daemon, the Sense HAT driver
# -- seconds of imports on a Pi 3B, in a process that needs none of them. One spawn at a time.
_SPAWN_LOCK = threading.Lock()


@contextlib.contextmanager
def _main_hidden():
    main = sys.modules.get("__main__")
    saved = {}
    with _SPAWN_LOCK:
        if main is not None:
            for k in ("__file__", "__spec__"):
                if k in main.__dict__:
                    saved[k] = main.__dict__[k]
            main.__dict__.pop("__file__", None)
            main.__dict__["__spec__"] = None
        try:
            yield
        finally:
            if main is not None:
                main.__dict__.pop("__spec__", None)
                main.__dict__.update(saved)


# collect() states
OK, LATE, DEAD, ERROR, BAD = "ok", "late", "dead", "error", "bad"

# ON THE GIL, MEASURED (2026-11-18, tools/bench_policy_worker.py). Every syscall wrapper releases the
# GIL, and taking it back while other Python threads are busy costs up to sys.getswitchinterval()
# (5 ms by default) each time; poll + recv_bytes is three such round trips after the answer is in.
# Reading the socket under the Pipe with one recv() (SO_RCVTIMEO) or select() + one recv() was
# tried and measured: no better than this, so the plain Connection stayed. What DID move the
# numbers was the switch interval itself -- see the bench's --switch-interval-ms.


class PolicyWorkerClient:
    """The daemon's end of the pipe. Every call after start() is made from the CAN thread only.

    collect() never blocks past its deadline and never raises: a dead, broken or confused worker
    comes back as a state the daemon turns into a stop. `why` carries the detail."""

    def __init__(self, opts=None):
        self.opts = dict(opts or {})
        self.proc = None
        self.conn = None
        self.ready = None
        self.why = ""
        self.stale = 0                    # late answers read and discarded
        self.last_rtt = 0.0               # post -> collect(ok), seconds
        self._t_post = 0.0
        self._obs = np.empty(OBS_N)
        self._closed = False
        self._reaper = None

    # ---------------------------------------------------------------- lifecycle
    def start(self, bundle_path, timeout=15.0):
        """Spawn, load, probe. Returns the READY dict; raises PolicyWorkerError with the reason."""
        import multiprocessing as mp
        ctx = mp.get_context("spawn")             # never fork a multi-threaded daemon
        parent, child = ctx.Pipe(duplex=True)
        proc = ctx.Process(target=worker_main, args=(child, str(bundle_path), self.opts),
                           name="policy-worker", daemon=True)
        try:
            with _main_hidden():
                proc.start()
        except Exception as e:                    # noqa: BLE001 -- surfaced as the refusal
            parent.close()
            child.close()
            raise PolicyWorkerError("the policy worker could not be started: {}".format(e))
        child.close()
        self.proc, self.conn = proc, parent
        try:
            if not parent.poll(timeout):
                raise PolicyWorkerError("the policy worker did not report READY within {:.0f} s"
                                        .format(timeout))
            msg = parent.recv_bytes()
        except (EOFError, OSError):
            self._reap(wait=True)
            raise PolicyWorkerError("the policy worker died while starting (exit code {})".format(
                self.proc.exitcode))
        except PolicyWorkerError:
            self._reap(wait=True)
            raise
        if msg[:1] == b"E":
            self._reap(wait=True)
            raise PolicyWorkerError("the policy worker failed to start: {}".format(
                msg[1:].decode("utf-8", "replace")))
        if msg[:1] != b"R":
            self._reap(wait=True)
            raise PolicyWorkerError("the policy worker sent {!r} instead of READY".format(msg[:1]))
        self.ready = json.loads(msg[1:].decode())
        return self.ready

    def alive(self):
        return (self.proc is not None and not self._closed and self.proc.is_alive())

    def stop(self, timeout=1.0, wait=True):
        """Q, then join; terminate, then kill, if it will not go. wait=False does the joining on a
        reaper thread, so the CAN thread never waits for a process to exit."""
        if self.proc is None or self._closed:
            if wait and self._reaper is not None:
                self._reaper.join()
            return
        self._closed = True
        try:
            self.conn.send_bytes(b"Q")
        except (OSError, ValueError, EOFError):
            pass
        self._reap(wait=wait, timeout=timeout)

    def _reap(self, wait=True, timeout=1.0):
        self._closed = True
        proc, conn = self.proc, self.conn

        def reap():
            proc.join(timeout)
            if proc.exitcode is None:
                proc.terminate()
                proc.join(0.5)
            if proc.exitcode is None:
                proc.kill()
                proc.join(1.0)
            try:
                conn.close()
            except Exception:                     # noqa: BLE001
                pass
        if wait:
            reap()
        else:
            self._reaper = threading.Thread(target=reap, name="policy-worker-reaper", daemon=True)
            self._reaper.start()

    # ---------------------------------------------------------------- the hot path
    def _send(self, data):
        if self._closed:
            self.why = "the policy worker was already stopped"
            return False
        try:
            self.conn.send_bytes(data)
            return True
        except (OSError, ValueError, EOFError) as e:
            self.why = "the pipe to the policy worker is broken: {}".format(e)
            return False

    def post(self, seq, pos, vel, tau, grav, gyro):
        """Send observation `seq` and return at once. False if the pipe is gone."""
        self._t_post = time.perf_counter()
        return self._send(pack_obs(b"T", seq, pos, vel, tau, grav, gyro, self._obs))

    def _recv(self, deadline):
        """(state, msg). Waits in poll(), which releases the GIL."""
        remaining = max(0.0, deadline - time.perf_counter())
        try:
            if not self.conn.poll(remaining):
                if not self.proc.is_alive():
                    self.why = "the policy worker died (exit code {})".format(self.proc.exitcode)
                    return DEAD, None
                return LATE, None
            msg = self.conn.recv_bytes()
        except (EOFError, OSError, ValueError) as e:
            self.why = "the policy worker is gone: {}".format(type(e).__name__)
            return DEAD, None
        if msg[:1] == b"E":
            self.why = "the policy worker raised: {}".format(msg[1:].decode("utf-8", "replace"))
            return ERROR, None
        return OK, msg

    def collect(self, seq, deadline):
        """(command, state) for observation `seq`, waiting until the absolute perf_counter
        `deadline`. Answers to OLDER observations -- the late ones -- are read and thrown away."""
        if self._closed:
            self.why = self.why or "the policy worker was already stopped"
            return None, DEAD
        while True:
            st, msg = self._recv(deadline)
            if st != OK:
                return None, st
            if msg[:1] != b"C" or len(msg) != CMD_MSG_LEN:
                self.why = "malformed answer from the policy worker ({!r}, {} bytes)".format(
                    msg[:1], len(msg))
                return None, BAD
            s = _seq_of(msg)
            if s < seq:
                self.stale += 1
                continue
            if s > seq:
                self.why = "the policy worker answered observation {} while {} was pending".format(
                    s, seq)
                return None, BAD
            self.last_rtt = time.perf_counter() - self._t_post
            return unpack_cmd(msg), OK

    def _await(self, want, seq, deadline):
        """Wait for a reply of type `want` (A or K), discarding late commands on the way."""
        while True:
            st, msg = self._recv(deadline)
            if st != OK:
                return st, None
            kind = msg[:1]
            if kind == b"C" and len(msg) == CMD_MSG_LEN and (seq is None or _seq_of(msg) < seq):
                self.stale += 1
                continue
            if kind != want:
                self.why = "the policy worker sent {!r} while {!r} was expected".format(kind, want)
                return BAD, None
            if want == b"A" and (len(msg) != _HDR or _seq_of(msg) != seq):
                self.why = "the policy worker acknowledged the wrong start"
                return BAD, None
            return OK, msg

    def begin(self, seq, pos, vel, tau, grav, gyro, timeout=0.5):
        """ctrl.start() in the worker. Returns the state (OK when acknowledged)."""
        if not self._send(pack_obs(b"S", seq, pos, vel, tau, grav, gyro, self._obs)):
            return DEAD
        st, _ = self._await(b"A", seq, time.perf_counter() + timeout)
        return st

    def call(self, name, *args, timeout=0.5, seq=None):
        """A controller method, by name. Returns (state, result). Off the hot path: JSON here."""
        if name not in METHODS:
            raise ValueError("not a forwarded method: {!r}".format(name))
        args = [a.item() if isinstance(a, np.generic) else a for a in args]
        if not self._send(b"M" + json.dumps({"name": name, "args": args}).encode()):
            return DEAD, None
        st, msg = self._await(b"K", seq, time.perf_counter() + timeout)
        if st != OK:
            return st, None
        res = json.loads(msg[1:].decode()).get("result")
        return OK, (tuple(res) if isinstance(res, list) else res)
