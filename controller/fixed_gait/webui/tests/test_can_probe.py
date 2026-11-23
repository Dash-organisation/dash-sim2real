"""tools/can_probe.py: the pure analysis half (the sockets need a Pi). Plan C, step 0."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(HERE))), "tools")
if TOOLS not in sys.path:
    sys.path.insert(0, TOOLS)

import can_probe as P                                                   # noqa: E402

EXT = P.CAN_EFF_FLAG
STATUS = 0x2900                     # status frames: extended, drive id in the low byte


def _traffic(seconds=2.0):
    """can0: abd periodic every 5.00 ms; cam answers each 10 ms command 0.4 ms later. The daemon's
    commands come back flagged `local`."""
    frames = []
    n_cmd = int(seconds / 0.010)
    for k in range(n_cmd):
        t = 0.001 + k * 0.010
        for cid in (104, 105, 106):
            frames.append((t, "can0", EXT | (1 << 8) | cid, 4, True))       # SET_CURRENT
        frames.append((t + 0.0004, "can0", EXT | STATUS | 105, 8, False))  # cam replies
    for k in range(int(seconds / 0.005)):
        frames.append((k * 0.005, "can0", EXT | STATUS | 104, 8, False))
    frames.append((0.5, "can0", P.CAN_ERR_FLAG | 0x4, 8, False))
    frames.append((0.6, "can0", 0x123, 8, False))                           # a standard id
    frames.sort()
    return frames


def test_periodic_and_replying_drives_are_told_apart():
    r = P.summarize(_traffic(), 2.0)
    abd, cam = r["drives"]["right.abd"], r["drives"]["right.cam"]
    assert abd["verdict"] == "periodic 200 Hz"
    assert abs(abd["period_ms"]["p50"] - 5.0) < 1e-6
    assert cam["verdict"] == "replies to commands"
    assert abs(cam["period_ms"]["p50"] - 10.0) < 1e-6
    assert abs(cam["delay_after_cmd_ms"]["p90"] - 0.4) < 1e-6
    # a periodic drive's delay after the last command is spread over the command period
    assert abd["delay_after_cmd_ms"]["p90"] > 4.0
    # thigh is commanded but never speaks
    assert r["drives"]["right.thigh"]["verdict"] == "silent"
    assert r["drives"]["right.thigh"]["cmd_per_s"] == 100.0


def test_channel_counts():
    c = P.summarize(_traffic(), 2.0)["channels"]["can0"]
    assert c["err"] == 1 and c["std"] == 1
    assert c["tx_local_per_s"] == 300.0
    assert c["rx_per_s"] == 100.0 + 200.0 + 0.5         # cam + abd + the standard frame
    assert c["rx_per_tick"] == 3.0


def test_parse_task_stat_survives_spaces_in_comm():
    line = ("1234 (python3 thr x) S 1 2 3 4 5 6 7 8 9 10 111 222 0 0 20 0 7 0 100 0 0")
    assert P.parse_task_stat(line) == (111, 222)


def test_thread_delta():
    a = {"10": ("python3", 100, 50, 1000, 10)}
    b = {"10": ("python3", 300, 150, 2000, 110), "11": ("python3", 1, 1, 1, 1)}
    rows = P.thread_delta(a, b, 10.0, 100)
    assert rows == [{"tid": 10, "comm": "python3", "cpu_pct": 30.0, "vol_per_s": 100.0,
                     "invol_per_s": 10.0}]


def test_the_probe_never_sends():
    """Listening only, by construction: no send()/sendto()/sendmsg() call anywhere in the file."""
    with open(P.__file__) as f:
        src = f.read()
    for call in (".send(", ".sendto(", ".sendmsg(", ".sendall("):
        assert call not in src, call
