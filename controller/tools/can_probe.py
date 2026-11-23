#!/usr/bin/env python3
"""Passive CAN and daemon probe (plan C, step 0). It LISTENS. It never transmits a frame.

What it answers, before and after each step of plan C:

  listen    per interface: frames/s received and sent by this host (the daemon's commands, seen
            through socketcan's local loopback), error frames, the interface's drop counters and
            txqueuelen; per drive: status period (kernel timestamps), and the delay from the last
            command to that drive to its next status -- a drive that REPLIES to commands (2026-11-20:
            right.cam) has a short, tight delay and the loop's period, a periodic one (every other
            drive) emits every 5.00 ms whatever the commands do.
  --bench   what one read of a waiting frame costs, per method: python-can bus.recv(timeout=0)
            (what daemon._drain did up to plan C), a raw AF_CAN socket, and the empty read that
            ends every drain (exception vs poll vs select). Each method gets its own socket and
            drains it every 10 ms, like the daemon's tick.
  --threads per-thread CPU of the running daemon over the window (utime+stime, voluntary and
            involuntary context switches), from /proc/<pid>/task. Linux does not see Python's
            thread names; by creation order they are: main, black box, CAN loop, IMU, environment,
            Flask (journal 2026-11-20, section 4).

On the robot's Pi, daemon running, robot LIMP (it only listens, so either is fine):

    cd ~/running_robot/robot && .venv/bin/python tools/can_probe.py --seconds 10 --bench --threads
    ... --json /tmp/can_probe_before.json            # keep the numbers for the before/after

(`--cpus 2,3`, the default, keeps the probe off the daemon's cores 0-1.)

Plain Python 3 and Linux only for the socket parts; python-can is optional (its bench row is skipped
without it). The analysis functions are pure and tested off the robot
(fixed_gait/webui/tests/test_can_probe.py).
"""
import argparse
import json
import os
import select
import socket
import struct
import subprocess
import sys
import time

FRAME_FMT = "=IB3x8s"                      # struct can_frame: can_id, len, pad/res0/len8_dlc, data
FRAME_SIZE = struct.calcsize(FRAME_FMT)    # 16
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_EFF_MASK = 0x1FFFFFFF
CAN_SFF_MASK = 0x000007FF
SO_TIMESTAMP = getattr(socket, "SO_TIMESTAMP", 29)       # missing from Python on the Pi
SOL_CAN_RAW = getattr(socket, "SOL_CAN_RAW", 101)
CAN_RAW_ERR_FILTER = getattr(socket, "CAN_RAW_ERR_FILTER", 2)
MSG_DONTROUTE = getattr(socket, "MSG_DONTROUTE", 4)      # set on frames this host sent
TIMEVAL = struct.Struct("@ll")

DRIVE_IDS = (104, 105, 106)                # abd, cam, thigh -- same ids on both buses
ROLE = {104: "abd", 105: "cam", 106: "thigh"}
SIDE = {"can0": "right", "can1": "left"}   # paths.SIDE_CHANNEL, inverted
TICK_S = 0.010


# ===================================================================== pure analysis
def pct(xs, q):
    """Percentile (nearest rank on a sorted copy); None for an empty list."""
    if not xs:
        return None
    s = sorted(xs)
    k = min(len(s) - 1, max(0, int(round(q / 100.0 * (len(s) - 1)))))
    return s[k]


def stats(xs, scale=1.0, nd=3):
    if not xs:
        return {"n": 0}
    return {"n": len(xs), "p50": round(pct(xs, 50) * scale, nd), "p90": round(pct(xs, 90) * scale, nd),
            "p99": round(pct(xs, 99) * scale, nd), "max": round(max(xs) * scale, nd),
            "min": round(min(xs) * scale, nd)}


def drive_name(ch, cid):
    return "{}.{}".format(SIDE.get(ch, ch), ROLE.get(cid, cid))


def classify_drive(period_ms, delay_ms):
    """'periodic' / 'replies to commands' / 'silent' / '?' from the status period and the delay
    after a command. A drive replying to commands: its delay p90 is well under a millisecond and its
    period is the LOOP's (~10 ms). A periodic drive: period 5.00 ms, delay spread over the command
    period."""
    if not period_ms or period_ms.get("n", 0) < 5:
        return "silent"
    p50 = period_ms["p50"]
    if delay_ms and delay_ms.get("n", 0) >= 5 and delay_ms["p90"] < 1.0 and p50 > 7.0:
        return "replies to commands"
    if 4.5 <= p50 <= 5.5:
        return "periodic 200 Hz"
    return "periodic {:.0f} Hz".format(1000.0 / p50) if p50 > 0 else "?"


def summarize(frames, seconds):
    """frames: list of (t_s, channel, can_id_with_flags, dlc, local). `local` is True for frames
    this host sent (the daemon's commands). Returns a JSON-able dict."""
    out = {"seconds": round(seconds, 3), "channels": {}, "drives": {}}
    seconds = max(seconds, 1e-9)
    by_ch = {}
    last_cmd = {}                          # (ch, cid) -> t of the last command to that drive
    status_t = {}                          # (ch, cid) -> [t...]
    delays = {}                            # (ch, cid) -> [status t - last command t]
    cmd_n = {}
    for t, ch, cid_raw, dlc, local in frames:
        c = by_ch.setdefault(ch, {"rx": 0, "tx_local": 0, "err": 0, "std": 0, "ext": 0, "rtr": 0,
                                  "ids": {}})
        if cid_raw & CAN_ERR_FLAG:
            c["err"] += 1
            continue
        if cid_raw & CAN_RTR_FLAG:
            c["rtr"] += 1
        ext = bool(cid_raw & CAN_EFF_FLAG)
        aid = cid_raw & (CAN_EFF_MASK if ext else CAN_SFF_MASK)
        c["ext" if ext else "std"] += 1
        key = "{}{:x}".format("x" if ext else "s", aid)
        c["ids"][key] = c["ids"].get(key, 0) + 1
        drive = aid & 0xFF
        if local:
            c["tx_local"] += 1
            if ext:
                last_cmd[(ch, drive)] = t
                cmd_n[(ch, drive)] = cmd_n.get((ch, drive), 0) + 1
            continue
        c["rx"] += 1
        if ext and drive in DRIVE_IDS:
            status_t.setdefault((ch, drive), []).append(t)
            tc = last_cmd.get((ch, drive))
            if tc is not None and 0.0 <= t - tc < 0.050:
                delays.setdefault((ch, drive), []).append(t - tc)
    for ch, c in sorted(by_ch.items()):
        c["rx_per_s"] = round(c["rx"] / seconds, 1)
        c["tx_local_per_s"] = round(c["tx_local"] / seconds, 1)
        c["rx_per_tick"] = round(c["rx"] / seconds * TICK_S, 2)
        c["ids"] = {k: round(v / seconds, 1) for k, v in sorted(c["ids"].items())}   # per second
        out["channels"][ch] = c
    keys = sorted(set(status_t) | set(cmd_n))
    for ch, cid in keys:
        ts = status_t.get((ch, cid), [])
        per = [b - a for a, b in zip(ts, ts[1:])]
        period = stats(per, 1e3)
        delay = stats(delays.get((ch, cid), []), 1e3)
        out["drives"][drive_name(ch, cid)] = {
            "channel": ch, "id": cid, "status_per_s": round(len(ts) / seconds, 1),
            "cmd_per_s": round(cmd_n.get((ch, cid), 0) / seconds, 1),
            "period_ms": period, "delay_after_cmd_ms": delay,
            "verdict": classify_drive(period, delay)}
    rx = sum(c["rx"] for c in out["channels"].values())
    out["status_frames_per_tick"] = round(rx / seconds * TICK_S, 2)
    return out


def parse_task_stat(text):
    """(utime, stime) in clock ticks from one /proc/<pid>/task/<tid>/stat line. The comm field is
    in parentheses and may contain spaces, so split after the LAST ')'."""
    rest = text[text.rindex(")") + 2:].split()
    return int(rest[11]), int(rest[12])            # fields 14 and 15, counted from the state field


def parse_ctxt(text):
    vol = nonvol = 0
    for line in text.splitlines():
        if line.startswith("voluntary_ctxt_switches:"):
            vol = int(line.split()[1])
        elif line.startswith("nonvoluntary_ctxt_switches:"):
            nonvol = int(line.split()[1])
    return vol, nonvol


def thread_delta(a, b, dt, clk_tck):
    """a, b: {tid: (comm, utime, stime, vol, nonvol)} at two instants -> rows sorted by tid."""
    rows = []
    for tid in sorted(set(a) & set(b), key=int):
        ca, ua, sa, va, na = a[tid]
        _, ub, sb, vb, nb = b[tid]
        rows.append({"tid": int(tid), "comm": ca,
                     "cpu_pct": round(100.0 * ((ub + sb) - (ua + sa)) / clk_tck / dt, 1),
                     "vol_per_s": round((vb - va) / dt, 0), "invol_per_s": round((nb - na) / dt, 0)})
    return rows


# ===================================================================== sockets (Linux)
def _need_af_can():
    if not hasattr(socket, "AF_CAN"):
        sys.exit("can_probe: this Python has no AF_CAN (not Linux?) -- run it on the robot's Pi")


def open_listener(ch, timestamps=True, errors=True):
    """A raw CAN socket bound to `ch`, receive-only by construction: nothing in this file calls
    send() on it."""
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    if timestamps:
        s.setsockopt(socket.SOL_SOCKET, SO_TIMESTAMP, 1)
    if errors:
        s.setsockopt(SOL_CAN_RAW, CAN_RAW_ERR_FILTER, CAN_EFF_MASK)   # every error class
    s.bind((ch,))
    s.setblocking(False)
    return s


def iface_stats(ch):
    base = "/sys/class/net/{}".format(ch)
    out = {}
    for k in ("tx_dropped", "rx_dropped", "tx_errors", "rx_errors", "tx_packets", "rx_packets"):
        try:
            with open(os.path.join(base, "statistics", k)) as f:
                out[k] = int(f.read())
        except OSError:
            pass
    try:
        with open(os.path.join(base, "tx_queue_len")) as f:
            out["txqueuelen"] = int(f.read())
    except OSError:
        pass
    return out


def ip_details(ch):
    try:
        return subprocess.run(["ip", "-s", "-d", "link", "show", ch], capture_output=True,
                              text=True, timeout=2).stdout
    except Exception as e:                                   # noqa: BLE001
        return "ip: {}".format(e)


def listen(channels, seconds):
    _need_af_can()
    socks = {open_listener(ch): ch for ch in channels}
    anc = socket.CMSG_SPACE(TIMEVAL.size)
    frames = []
    before = {ch: iface_stats(ch) for ch in channels}
    t0 = time.time()
    t_end = t0 + seconds
    while True:
        now = time.time()
        if now >= t_end:
            break
        r, _, _ = select.select(list(socks), [], [], min(0.1, t_end - now))
        for s in r:
            while True:
                try:
                    data, ancdata, flags, _ = s.recvmsg(FRAME_SIZE, anc)
                except BlockingIOError:
                    break
                if len(data) < FRAME_SIZE:
                    continue
                t = None
                for lvl, typ, d in ancdata:
                    if lvl == socket.SOL_SOCKET and typ == SO_TIMESTAMP and len(d) >= TIMEVAL.size:
                        sec, usec = TIMEVAL.unpack(d[:TIMEVAL.size])
                        t = sec + usec * 1e-6
                if t is None:
                    t = time.time()
                cid, dlc, _ = struct.unpack(FRAME_FMT, data)
                frames.append((t, socks[s], cid, dlc, bool(flags & MSG_DONTROUTE)))
    seconds = time.time() - t0
    for s in socks:
        s.close()
    after = {ch: iface_stats(ch) for ch in channels}
    out = summarize(frames, seconds)
    for ch in channels:
        d = {k: after[ch][k] - before[ch].get(k, 0) for k in after[ch]
             if k != "txqueuelen" and k in before[ch]}
        out["channels"].setdefault(ch, {})["iface"] = {
            "txqueuelen": after[ch].get("txqueuelen"),
            "tx_dropped_total": after[ch].get("tx_dropped"),
            "delta": d, "tx_dropped_per_s": round(d.get("tx_dropped", 0) / seconds, 2)}
    return out


# ---- read-cost bench ---------------------------------------------------------------------------
def _bench_loop(seconds, drain_once):
    """Every TICK_S: drain_once() -> (per-frame ns list, empty-read ns or None). Returns stats."""
    per_frame, empty, per_tick = [], [], []
    t_end = time.time() + seconds
    nxt = time.time()
    while time.time() < t_end:
        nxt += TICK_S
        d = nxt - time.time()
        if d > 0:
            time.sleep(d)
        f, e = drain_once()
        per_frame.extend(f)
        if e is not None:
            empty.append(e)
        per_tick.append(sum(f) + (e or 0))
    return {"frame_us": stats(per_frame, 1e-3, 1), "empty_us": stats(empty, 1e-3, 1),
            "drain_ms": stats(per_tick, 1e-6, 3),
            "frames_per_drain": round(len(per_frame) / max(1, len(per_tick)), 2)}


def bench_raw(ch, seconds, empty_mode):
    """Raw socket; `empty_mode`: 'exception' (recv until BlockingIOError), 'poll' (poll(0) before
    each recv) or 'select' (select([s],[],[],0) before each recv)."""
    s = open_listener(ch, timestamps=False, errors=False)
    unpack = struct.Struct(FRAME_FMT).unpack
    pc = time.perf_counter_ns
    if empty_mode == "poll":
        p = select.poll()
        p.register(s, select.POLLIN)
        ready = lambda: bool(p.poll(0))                                    # noqa: E731
    elif empty_mode == "select":
        ready = lambda: bool(select.select([s], [], [], 0)[0])              # noqa: E731
    else:
        ready = None

    def drain_once():
        f = []
        while True:
            t = pc()
            if ready is not None and not ready():
                return f, pc() - t
            try:
                b = s.recv(FRAME_SIZE)
            except BlockingIOError:
                return f, pc() - t
            cid, dlc, data = unpack(b)
            _ = (cid & CAN_EFF_FLAG, cid & CAN_EFF_MASK, data[:dlc])
            f.append(pc() - t)
    try:
        return _bench_loop(seconds, drain_once)
    finally:
        s.close()


def bench_canio_raw(ch, seconds, empty_check=None):
    """canio.RawCanBus.recv(timeout=0.0), the daemon's own backend since plan C step 1."""
    here = os.path.dirname(os.path.abspath(__file__))
    webui = os.path.join(os.path.dirname(here), "fixed_gait", "webui")
    if webui not in sys.path:
        sys.path.insert(0, webui)
    try:
        import canio
    except Exception as e:                                   # noqa: BLE001
        return {"skipped": "canio import failed: {}".format(e)}
    if not hasattr(canio, "RawCanBus"):
        return {"skipped": "canio has no RawCanBus (before plan C step 1)"}
    try:
        bus = canio.RawCanBus(ch) if empty_check is None else canio.RawCanBus(ch, empty_check=empty_check)
    except ValueError as e:
        return {"skipped": str(e)}
    pc = time.perf_counter_ns

    def drain_once():
        f = []
        while True:
            t = pc()
            m = bus.recv(timeout=0.0)
            if m is None:
                return f, pc() - t
            _ = (m.is_extended_id, m.arbitration_id, m.data)
            f.append(pc() - t)
    try:
        return _bench_loop(seconds, drain_once)
    finally:
        bus.shutdown()


def bench_python_can(ch, seconds):
    try:
        import can
    except ImportError:
        return {"skipped": "python-can not installed"}
    bus = can.Bus(interface="socketcan", channel=ch)       # opening a socketcan bus sends nothing
    pc = time.perf_counter_ns

    def drain_once():
        f = []
        while True:
            t = pc()
            m = bus.recv(timeout=0.0)
            if m is None:
                return f, pc() - t
            _ = (m.is_extended_id, m.arbitration_id, m.data)
            f.append(pc() - t)
    try:
        return _bench_loop(seconds, drain_once)
    finally:
        bus.shutdown()


def bench(ch, seconds):
    _need_af_can()
    out = {}
    for name, fn in (("python-can", lambda: bench_python_can(ch, seconds)),
                     ("raw/exception", lambda: bench_raw(ch, seconds, "exception")),
                     ("raw/poll", lambda: bench_raw(ch, seconds, "poll")),
                     ("raw/select", lambda: bench_raw(ch, seconds, "select")),
                     ("canio.Raw/default", lambda: bench_canio_raw(ch, seconds)),
                     ("canio.Raw/poll", lambda: bench_canio_raw(ch, seconds, "poll")),
                     ("canio.Raw/exc", lambda: bench_canio_raw(ch, seconds, "exception"))):
        out[name] = fn()
    return out


# ---- daemon threads ----------------------------------------------------------------------------
def daemon_pid():
    try:
        r = subprocess.run(["systemctl", "show", "-p", "MainPID", "--value", "runningrobot-webui"],
                           capture_output=True, text=True, timeout=2)
        pid = int(r.stdout.strip() or 0)
        return pid or None
    except Exception:                                        # noqa: BLE001
        return None


def _task_snapshot(pid):
    snap = {}
    base = "/proc/{}/task".format(pid)
    for tid in os.listdir(base):
        try:
            with open(os.path.join(base, tid, "stat")) as f:
                st = f.read()
            with open(os.path.join(base, tid, "status")) as f:
                stt = f.read()
            with open(os.path.join(base, tid, "comm")) as f:
                comm = f.read().strip()
        except OSError:
            continue
        u, s = parse_task_stat(st)
        v, n = parse_ctxt(stt)
        snap[tid] = (comm, u, s, v, n)
    return snap


# ===================================================================== report
def print_report(r):
    if "listen" in r:
        L = r["listen"]
        print("== listen ({:.1f} s, {} status frames per 10 ms tick)".format(
            L["seconds"], L["status_frames_per_tick"]))
        for ch, c in L["channels"].items():
            i = c.get("iface", {})
            print("  {}: rx {} /s, sent by this host {} /s, err {}, std {}, ext {}; txqueuelen {}, "
                  "tx_dropped +{} ({}/s, total {})".format(
                      ch, c.get("rx_per_s"), c.get("tx_local_per_s"), c.get("err"), c.get("std"),
                      c.get("ext"), i.get("txqueuelen"), i.get("delta", {}).get("tx_dropped"),
                      i.get("tx_dropped_per_s"), i.get("tx_dropped_total")))
        print("  {:<12} {:>7} {:>7}   {:<34} {:<26} {}".format(
            "drive", "stat/s", "cmd/s", "period ms p50/p90/max", "delay after cmd p10..p90", "verdict"))
        for n, d in L["drives"].items():
            p, dl = d["period_ms"], d["delay_after_cmd_ms"]
            ps = "{}/{}/{}".format(p.get("p50"), p.get("p90"), p.get("max")) if p["n"] else "-"
            ds = "{}..{} (n={})".format(dl.get("min"), dl.get("p90"), dl["n"]) if dl["n"] else "-"
            print("  {:<12} {:>7} {:>7}   {:<34} {:<26} {}".format(
                n, d["status_per_s"], d["cmd_per_s"], ps, ds, d["verdict"]))
    for ch, b in r.get("bench", {}).items():
        print("== read cost on {} (drained every 10 ms)".format(ch))
        for name, x in b.items():
            if "skipped" in x:
                print("  {:<18} skipped: {}".format(name, x["skipped"]))
                continue
            f, e, d = x["frame_us"], x["empty_us"], x["drain_ms"]
            print("  {:<18} frame p50/p90 {}/{} us, empty read p50 {} us, drain p50/p90 {}/{} ms "
                  "({} frames/drain)".format(name, f.get("p50"), f.get("p90"), e.get("p50"),
                                             d.get("p50"), d.get("p90"), x["frames_per_drain"]))
    if "threads" in r:
        T = r["threads"]
        if "error" in T:
            print("== daemon threads: {}".format(T["error"]))
        else:
            print("== daemon threads (pid {}, {} s) -- order: main, black box, CAN loop, IMU, env, "
                  "Flask, ...".format(T["pid"], T["seconds"]))
            for row in T["threads"]:
                print("  tid {:>6} {:<16} cpu {:>5.1f} %  vol {:>5.0f}/s  invol {:>5.0f}/s".format(
                    row["tid"], row["comm"], row["cpu_pct"], row["vol_per_s"], row["invol_per_s"]))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--channels", nargs="+", default=["can0", "can1"])
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--bench", action="store_true", help="compare read costs (each method --seconds)")
    ap.add_argument("--bench-seconds", type=float, default=None)
    ap.add_argument("--threads", action="store_true", help="daemon per-thread CPU over --seconds")
    ap.add_argument("--pid", type=int, default=None, help="daemon pid (default: from systemd)")
    ap.add_argument("--no-listen", action="store_true")
    ap.add_argument("--cpus", default="2,3", help="pin the probe to these cores ('' = no pinning)")
    ap.add_argument("--json", default=None, help="also write the raw results here")
    ap.add_argument("--ip", action="store_true", help="print `ip -s -d link show` per channel")
    a = ap.parse_args(argv)

    if a.cpus and hasattr(os, "sched_setaffinity"):
        try:
            os.sched_setaffinity(0, {int(c) for c in a.cpus.split(",")})
        except OSError as e:
            print("(could not pin to cores {}: {})".format(a.cpus, e))
    r = {"when": time.strftime("%Y-%m-%d %H:%M:%S"), "host": socket.gethostname(),
         "python": sys.version.split()[0]}
    if a.threads:
        pid = a.pid or daemon_pid()
        # sampled around the listen window so both describe the same 10 s
        snap_a = _task_snapshot(pid) if pid else None
        t_a = time.time()
    if not a.no_listen:
        r["listen"] = listen(a.channels, a.seconds)
    if a.threads:
        if not pid:
            r["threads"] = {"error": "no daemon pid (pass --pid)"}
        else:
            if a.no_listen:
                time.sleep(a.seconds)
            dt = time.time() - t_a
            r["threads"] = {"pid": pid, "seconds": round(dt, 2),
                            "threads": thread_delta(snap_a, _task_snapshot(pid), dt,
                                                    os.sysconf("SC_CLK_TCK"))}
    if a.bench:
        bs = a.bench_seconds or min(a.seconds, 5.0)
        r["bench"] = {ch: bench(ch, bs) for ch in a.channels}
    if a.ip:
        for ch in a.channels:
            print(ip_details(ch))
    print_report(r)
    if a.json:
        with open(a.json, "w") as f:
            json.dump(r, f, indent=1)
        print("written: {}".format(a.json))
    return r


if __name__ == "__main__":
    main()
