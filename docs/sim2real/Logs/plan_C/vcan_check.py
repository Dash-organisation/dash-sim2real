"""Plan C on the Pi, on a VIRTUAL CAN interface only (no motor is on it).

1. byte identity: every canio command sent by the python-can backend and by RawCanBus, captured
   by an independent raw listener -> identical bytes; decode both ways -> same parse_status.
2. tick cost: a generator process emits 12 status frames per 10 ms (like can0+can1); the measuring
   process does the daemon's tick shape per backend: drain everything, then 6 sends.
"""
import multiprocessing as mp
import os
import socket
import struct
import sys
import time

IFACE = sys.argv[1] if len(sys.argv) > 1 else "vcanplanc"
assert IFACE.startswith("vcan"), "virtual interfaces only"
sys.path.insert(0, os.path.expanduser("~/plan_c_scratch/controller/fixed_gait/webui"))
import canio  # noqa: E402
import can  # noqa: E402


def listener():
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((IFACE,))
    s.settimeout(1.0)
    return s


def identity():
    cases = [(canio.set_current, (105, 0.0)), (canio.set_current, (104, -3.217)),
             (canio.set_pos, (106, -123.4567)), (canio.set_pos, (105, 1e9)),
             (canio.force_control, (104, bytes(range(1, 9))))]
    pc = canio.open_buses("socketcan", [IFACE], backend="python-can")[IFACE]
    raw = canio.RawCanBus(IFACE)
    lis = listener()
    ok = 0
    for fn, args in cases:
        assert fn(pc, *args) is True
        a = lis.recv(16)
        assert fn(raw, *args) is True
        b = lis.recv(16)
        same = a == b
        ok += same
        print("  {:<14} {:<22} python-can {}  raw {}  {}".format(
            fn.__name__, str(args), a.hex(), b.hex(), "IDENTICAL" if same else "DIFFERENT"))
    # decode: a status frame sent by an independent socket, read by both backends
    st = struct.pack(">hhhbB", 1234, -56, 789, 41, 0)
    tx = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    tx.bind((IFACE,))
    time.sleep(0.05)
    while pc.recv(timeout=0.0) is not None:      # the other backend's commands, via loopback
        pass
    while raw.recv(timeout=0.0) is not None:
        pass
    tx.send(struct.pack("=IB3x8s", canio.CAN_EFF_FLAG | 0x2969, 8, st))
    m1 = pc.recv(timeout=1.0)
    m2 = raw.recv(timeout=1.0)
    dec = (canio.parse_status(m1.data) is not None
           and canio.parse_status(m1.data) == canio.parse_status(m2.data)
           and m1.arbitration_id == m2.arbitration_id and m1.is_extended_id == m2.is_extended_id)
    print("  decode status: python-can {} / raw {} -> {}".format(
        canio.parse_status(m1.data), canio.parse_status(m2.data), "SAME" if dec else "DIFFERENT"))
    pc.shutdown(); raw.shutdown(); lis.close(); tx.close()
    return ok == len(cases) and dec


def generator(stop):
    os.sched_setaffinity(0, {3})
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((IFACE,))
    frames = [struct.pack("=IB3x8s", canio.CAN_EFF_FLAG | 0x2900 | cid, 8, bytes(8))
              for cid in (104, 105, 106)]
    nxt = time.time()
    while not stop.is_set():
        nxt += 0.0025                     # 3 frames every 2.5 ms = 12 per 10 ms
        d = nxt - time.time()
        if d > 0:
            time.sleep(d)
        for f in frames:
            s.send(f)


def pct(xs, q):
    s = sorted(xs)
    return s[min(len(s) - 1, int(round(q / 100 * (len(s) - 1))))]


def tick_bench(name, bus, seconds=5.0):
    pcn = time.perf_counter_ns
    drain, send, total, nf = [], [], [], 0
    nxt = time.time()
    t_end = nxt + seconds
    while time.time() < t_end:
        nxt += 0.010
        d = nxt - time.time()
        if d > 0:
            time.sleep(d)
        t0 = pcn()
        while True:
            m = bus.recv(timeout=0.0)
            if m is None:
                break
            if m.is_extended_id:
                canio.parse_status(m.data)
                nf += 1
        t1 = pcn()
        for cid in (104, 105, 106, 104, 105, 106):
            canio.set_current(bus, cid | 0x70, 0.0)      # ids 0xD8.. : nobody, and it is a vcan
        t2 = pcn()
        drain.append(t1 - t0); send.append(t2 - t1); total.append(t2 - t0)
    n = len(total)
    print("  {:<22} drain p50/p90 {:.3f}/{:.3f} ms  6 sends p50/p90 {:.3f}/{:.3f} ms  "
          "tick p50/p90 {:.3f}/{:.3f} ms  ({:.1f} frames/tick)".format(
              name, pct(drain, 50) / 1e6, pct(drain, 90) / 1e6, pct(send, 50) / 1e6,
              pct(send, 90) / 1e6, pct(total, 50) / 1e6, pct(total, 90) / 1e6, nf / n))


def main():
    os.sched_setaffinity(0, {2})
    print("== byte identity on", IFACE)
    good = identity()
    print("  ->", "ALL IDENTICAL" if good else "MISMATCH")
    if "--identity" in sys.argv:
        return
    print("== tick cost on", IFACE, "(12 status frames per 10 ms from another process)")
    stop = mp.Event()
    g = mp.Process(target=generator, args=(stop,), daemon=True)
    g.start()
    time.sleep(0.5)
    try:
        for name, mk in (("python-can", lambda: canio.open_buses("socketcan", [IFACE],
                                                                 backend="python-can")[IFACE]),
                         ("RawCanBus/poll", lambda: canio.RawCanBus(IFACE, empty_check="poll")),
                         ("RawCanBus/exception", lambda: canio.RawCanBus(IFACE, empty_check="exception")),
                         ("RawCanBus/select", lambda: canio.RawCanBus(IFACE, empty_check="select"))):
            b = mk()
            tick_bench(name, b)
            b.shutdown()
    finally:
        stop.set()
        g.join(2)
    print("send errors:", canio.send_errors())


if __name__ == "__main__":
    main()
