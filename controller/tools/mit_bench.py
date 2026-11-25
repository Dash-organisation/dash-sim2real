#!/usr/bin/env python3
"""Free-cam bench: is the AKE90-8 force-control ("MIT") oscillation in the drive, or in the leg?

Plan: docs/sim2real/DASH01_plan_banc_came_2026-11-25.md. Operator procedure: tools/mit_bench/README.md
(repo root). Offline analysis (Mac, with the phone audio): tools/mit_bench/analyze.py.

The left cam (can1, id 105) is screwed on its motor but NOT linked to the leg: a small inertia
that spins freely and that the operator can press on by hand. Phase A runs, on that one drive,
SET_CURRENT 0, SET_POS, MIT at a ladder of kp/kd, MIT as a pure damper, MIT in slow rotation and
SET_RPM at the same speed, and records every status frame of the drive.

    python3 tools/mit_bench.py --channel can1 --id 105 --phase A --dry-run     # print the plan
    python3 tools/mit_bench.py --channel can1 --id 105 --phase A               # run it
    python3 tools/mit_bench.py --channel can1 --id 105 --phase A --steps A2,A3 # replay some steps
    python3 tools/mit_bench.py --channel can1 --id 105 --phase A --mock --yes  # offline

ONE PROGRAM ON THE BUS. Stop the web UI daemon first (LIMP in the UI, then
`sudo systemctl stop runningrobot-ui runningrobot-webui`). The preflight refuses to start if it
sees another program's commands on the bus, and a step stops everything if one shows up later.

WIRE FORMAT. Nothing is re-implemented here: MIT frames come from controller/deploy/mit.py
(`pack`, `limp_payload`), servo frames and the status decoder from webui/canio.py. The one
command canio does not have, SET_RPM (servo packet 3, int32 big-endian ERPM, manual section
2.1.4), is defined below on top of canio's sender.

SAFETY (every guard is exercised by fixed_gait/webui/tests/test_mit_bench.py on a fake bus)
  1. kp <= 200, kd <= 5: a step asking more is refused, and so is one mit.pack had to clamp.
  2. |speed| > step limit (3 rad/s at the output by default; for a velocity step |v_des| + 2).
  3. |Iq| > 15 A, temperature >= 60 C, error code != 0, no status frame for 50 ms.
  4. Runaway: std of Iq about its moving mean (90 ms window) > 3 A for > 0.3 s, or the speed
     changing sign on > 30 % of the frames over 0.5 s -> the step stops, marked `runaway`.
  5. Release, always (end of step, guard, Ctrl+C, exception): 1 s of mit.limp_payload() streamed,
     THEN SET_CURRENT 0 -- a SET_CURRENT 0 alone does not release a force frame
     (daemon._stream_limp). Ctrl+C once ends the current step; twice stops everything.
  The MIT position target is ALWAYS the position measured at the start of the step, and a step is
  refused if that position is outside the force frame's +-720 deg (daemon._send_hold).
  A speed / current / runaway stop ends the step and the bench moves on (that may be the very
  phenomenon under test); temperature, error code, silence, a foreign command, an exception or a
  second Ctrl+C stop the whole session.

Output: one .npz per session (every status frame, the step table, the timestamped journal) and a
summary table printed at the end.
"""
import argparse
import datetime
import json
import math
import os
import signal
import struct
import sys
import time
from collections import deque

HERE = os.path.dirname(os.path.abspath(__file__))
CONTROLLER = os.path.dirname(HERE)                  # robot/ on the Pi
WEBUI = os.path.join(CONTROLLER, "fixed_gait", "webui")
DEPLOY = os.path.join(CONTROLLER, "deploy")
for _p in (WEBUI, DEPLOY):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np                                   # noqa: E402

import canio                                         # noqa: E402
import mit                                           # noqa: E402

# ---------------------------------------------------------------- constants
RATE_HZ = 100.0                  # command rate, like the daemon
RELEASE_S = 1.0                  # limp_payload streamed this long before SET_CURRENT 0
RELEASE_CURRENT_S = 0.2          # then SET_CURRENT 0 streamed this long
PAUSE_S = 3.0                    # beep + pause between steps (an audio and a current marker)

KP_LIMIT = 200.0
KD_LIMIT = 5.0
I_LIMIT_A = 15.0
TEMP_LIMIT_C = 60.0
SILENCE_S = 0.050
SPEED_LIMIT_RADS = 3.0           # output shaft
SPEED_MARGIN_RADS = 2.0          # for velocity steps: |v_des| + this
RUNAWAY_WIN_S = 0.090            # Iq moving-mean window (as in tools/kd_validation/analyze_runs.py)
RUNAWAY_STD_A = 3.0
RUNAWAY_HOLD_S = 0.3
FLIP_WIN_S = 0.5
FLIP_FRAC = 0.30
FLIP_DEADBAND_RADS = 0.05        # a sign change counts only if both speeds are above this: at rest
#                                  the reported speed dithers by +-1 LSB (10 ERPM, 0.006 rad/s) and
#                                  would otherwise "flip" on every frame

# ERPM <-> output rad/s: 21 pole pairs x 8:1 gear (datasheet). jointmap.py notes this factor has
# never been measured on DASH-01, so the tool also shows the speed derived from the position.
POLE_PAIRS = 21
GEAR = 8.0
ERPM_PER_RADS = POLE_PAIRS * GEAR * 60.0 / (2.0 * math.pi)       # ~1604

# MIT velocity span of the AKE90-8: manual value, not yet verified on DASH-01 (CubeMars "V3 Drive
# Software User Manual" V1.0.1, p. 31). Only passed to mit.pack by steps with v_des != 0.
AKE90_V_RANGE = (-20.0, 20.0)

CAN_PACKET_SET_RPM = 3
# Extended-id command fields that mean "somebody is COMMANDING a drive" (VESC servo packets 0-8 minus
# 0, which MockBus uses for its status frames; real status frames are 0x29).
FOREIGN_CMDS = frozenset((1, 2, 3, 4, 5, 6, 7, 8))

# frame record columns
COLUMNS = ("t", "pos_deg", "spd_erpm", "cur_A", "temp_C", "err", "step", "sub",
           "cmd_kind", "cmd_kp", "cmd_kd", "cmd_val", "cmd_v_des")
KINDS = ("none", "current", "servo_pos", "mit", "servo_rpm", "limp_mit")
KIND_CODE = {k: i for i, k in enumerate(KINDS)}
SUB_PAUSE = -1                   # `sub` column: -1 pause, -2 release, -3 preflight
SUB_RELEASE = -2
SUB_PREFLIGHT = -3


def set_rpm(bus, cid, erpm):
    """Servo speed command (packet 3): ERPM as a big-endian int32 (manual section 2.1.4). Lives
    here and not in canio on purpose: nothing but this bench uses it."""
    val = int(round(erpm))
    val = max(-2_147_483_648, min(2_147_483_647, val))
    return canio._send(bus, cid | (CAN_PACKET_SET_RPM << 8), struct.pack(">i", val))


def rads_to_erpm(w):
    return w * ERPM_PER_RADS


def erpm_to_rads(e):
    return e / ERPM_PER_RADS


# ---------------------------------------------------------------- the step plan
STILL = ("still", 5.0, "hands off")
PRESS = ("press", 10.0, "press on the cam by hand, progressively, then release")
TURN = ("turn", 10.0, "turn the cam slowly by hand, both ways")
PRESS_RELEASE_AT = 7.0           # within a press sub-phase: press until 7 s, then release


def _step(sid, kind, what, subs, kp=0.0, kd=0.0, v_des=0.0, rpm=0.0, runaway=True,
          v_limit=None):
    return {"id": sid, "kind": kind, "kp": float(kp), "kd": float(kd), "v_des": float(v_des),
            "rpm": float(rpm), "subs": [list(s) for s in subs], "what": what, "runaway": runaway,
            "v_limit": v_limit}


SYNC_STEP = _step("SYNC", "current", "alignment: one sharp tap on the cam (audio + Iq peak)",
                  [("tap", 5.0, "TAP the cam ONCE, sharply, when told")], runaway=False,
                  v_limit=30.0)     # 0 A: a sharp tap may spin the free cam, and that is harmless

PHASE_A = [
    _step("A0", "current", "measurement noise floor", [STILL]),
    _step("A1", "servo_pos", "servo reference (should be quiet)", [STILL, PRESS]),
    _step("A2a", "mit", "MIT engaged, null law: oscillation with no current? (H1)", [STILL]),
    _step("A2b", "mit", "reproduces 2026-11-24", [STILL], kp=0.1, kd=0.1),
    _step("A2c", "mit", "", [STILL, PRESS], kp=5.0, kd=0.5),
    _step("A2d", "mit", "the 2026-11-24 press-test setting", [STILL, PRESS], kp=50.0, kd=1.0),
    _step("A2e", "mit", "effect of kd", [STILL, PRESS], kp=50.0, kd=0.1),
    _step("A2f", "mit", "effect of kd", [STILL, PRESS], kp=50.0, kd=5.0),
    _step("A2g", "mit", "the policy's max effort", [STILL, PRESS], kp=200.0, kd=1.0),
    _step("A3a", "mit", "kd term alone, moving", [TURN], kp=0.0, kd=1.0),
    _step("A3b", "mit", "kd term alone, moving", [TURN], kp=0.0, kd=5.0),
    _step("A4a", "mit", "MIT in slow rotation", [STILL], kp=0.0, kd=1.0, v_des=1.0),
    _step("A4b", "servo_rpm", "same speed in servo (reference)", [STILL],
          rpm=round(rads_to_erpm(1.0))),
]


def select_steps(plan, spec):
    """`spec` "A2,A3b": each token is a step id or a prefix of ids (A2 -> A2a..A2g). Plan order is
    kept. An unknown token is an error, not a silent no-op."""
    if not spec:
        return list(plan)
    toks = [t.strip() for t in spec.split(",") if t.strip()]
    keep = set()
    for t in toks:
        hit = [s["id"] for s in plan if s["id"] == t or s["id"].startswith(t)]
        if not hit:
            raise ValueError("unknown step {!r} (known: {})".format(
                t, ", ".join(s["id"] for s in plan)))
        keep.update(hit)
    return [s for s in plan if s["id"] in keep]


def step_seconds(step):
    return sum(s[1] for s in step["subs"])


def speed_limit(step):
    if step.get("v_limit"):
        return float(step["v_limit"])
    if step["kind"] == "mit" and step["v_des"]:
        return abs(step["v_des"]) + SPEED_MARGIN_RADS
    if step["kind"] == "servo_rpm":
        return abs(erpm_to_rads(step["rpm"])) + SPEED_MARGIN_RADS
    return SPEED_LIMIT_RADS


def describe(step):
    k = step["kind"]
    if k == "current":
        cmd = "SET_CURRENT 0"
    elif k == "servo_pos":
        cmd = "SET_POS at the measured position"
    elif k == "servo_rpm":
        cmd = "SET_RPM {:+.0f} ERPM (~{:+.2f} rad/s output)".format(step["rpm"],
                                                                    erpm_to_rads(step["rpm"]))
    else:
        cmd = "MIT kp {:g} kd {:g}".format(step["kp"], step["kd"])
        if step["v_des"]:
            cmd += " v_des {:+g} rad/s".format(step["v_des"])
        cmd += ", target = measured position"
    return cmd


def print_plan(steps, out=print):
    total = 0.0
    out("{:<5} {:>5}  {:<48} {}".format("step", "s", "command", "operator"))
    for s in steps:
        d = step_seconds(s)
        total += d + PAUSE_S + RELEASE_S + RELEASE_CURRENT_S
        ops = " + ".join("{:g} s {}".format(sub[1], sub[0]) for sub in s["subs"])
        out("{:<5} {:>5.0f}  {:<48} {}".format(s["id"], d, describe(s), ops))
        if s["what"]:
            out("{:<5} {:>5}  -> {}".format("", "", s["what"]))
        out("{:<5} {:>5}  speed limit {:.1f} rad/s ({:.0f} ERPM){}".format(
            "", "", speed_limit(s), rads_to_erpm(speed_limit(s)),
            "" if s["runaway"] else ", runaway detector off"))
    out("total ~{:.0f} s ({:.1f} min) including pauses and releases".format(total, total / 60.0))


# ---------------------------------------------------------------- metrics (shared with the summary)
def movmean_centered(x, n):
    x = np.asarray(x, float)
    if n < 2 or x.size == 0:
        return x.copy()
    k = np.ones(n) / n
    pad = np.pad(x, (n // 2, n - 1 - n // 2), mode="edge")
    return np.convolve(pad, k, mode="valid")


def step_metrics(t, cur, spd, temp, pos=None):
    """The summary numbers of one step. Same definitions as tools/kd_validation/analyze_runs.py:
    I_ripple_A = std of (Iq - its 90 ms centred moving mean); v_flip = fraction of consecutive
    frames whose reported speed changes sign (no deadband, so comparable with the Kd runs)."""
    t = np.asarray(t, float)
    n = t.size
    if n < 2:
        return {"frames": int(n), "rx_hz": None, "I_mean": None, "I_ripple_A": None,
                "v_flip": None, "I_max": None, "temp_max": None, "v_erpm_rads": None,
                "v_pos_rads": None}
    dur = t[-1] - t[0]
    rx_hz = (n - 1) / dur if dur > 0 else None
    w = max(2, int(round(RUNAWAY_WIN_S * (rx_hz or RATE_HZ))))
    cur = np.asarray(cur, float)
    spd = np.asarray(spd, float)
    ripple = cur - movmean_centered(cur, w)
    out = {"frames": int(n), "rx_hz": rx_hz, "I_mean": float(cur.mean()),
           "I_ripple_A": float(ripple.std()),
           "v_flip": float(np.mean(np.sign(spd[1:]) * np.sign(spd[:-1]) < 0)),
           "I_max": float(np.abs(cur).max()), "temp_max": float(np.max(temp)),
           "v_erpm_rads": float(erpm_to_rads(spd.mean())), "v_pos_rads": None}
    if pos is not None and dur > 0:
        out["v_pos_rads"] = float(math.radians((pos[-1] - pos[0]) / dur))
    return out


# ---------------------------------------------------------------- guards
class Guards:
    """Per-frame checks for one step. `check(frame)` -> None or an outcome string."""

    def __init__(self, step):
        self.v_lim = speed_limit(step)
        self.runaway = step["runaway"]
        self.mit = step["kind"] == "mit"
        self.iq = deque()               # (t, cur) within RUNAWAY_WIN_S
        self.std_since = None           # t at which the Iq std first went over the threshold
        self.sp = deque()               # (t, rad/s) within FLIP_WIN_S
        self.t_first = None
        self.detail = {}

    def check(self, t, st):
        if self.t_first is None:
            self.t_first = t
        v = erpm_to_rads(st["spd"])
        cur = st["cur"]
        if st["err"]:
            return "abort:error"
        if st["temp"] >= TEMP_LIMIT_C:
            return "abort:temp"
        if abs(cur) > I_LIMIT_A:
            return "abort:current"
        if abs(v) > self.v_lim:
            return "abort:speed"
        if self.mit and abs(math.radians(st["pos"])) >= mit.P_MAX:
            return "abort:mit_range"
        if not self.runaway:
            return None
        # Iq std about its moving mean
        iq = self.iq
        iq.append((t, cur))
        while iq and t - iq[0][0] > RUNAWAY_WIN_S:
            iq.popleft()
        if len(iq) >= 3:
            vals = [c for _, c in iq]
            m = sum(vals) / len(vals)
            sd = math.sqrt(sum((c - m) ** 2 for c in vals) / len(vals))
            if sd > RUNAWAY_STD_A:
                if self.std_since is None:
                    self.std_since = t
                elif t - self.std_since > RUNAWAY_HOLD_S:
                    self.detail = {"trigger": "iq_std", "iq_std": sd, "iq_mean": m, "t": t}
                    return "runaway"
            else:
                self.std_since = None
        # speed sign flips
        sp = self.sp
        sp.append((t, v))
        while sp and t - sp[0][0] > FLIP_WIN_S:
            sp.popleft()
        if len(sp) >= 10 and t - self.t_first >= FLIP_WIN_S * 0.9:
            flips = 0
            prev = None
            for _, x in sp:
                if prev is not None and abs(x) > FLIP_DEADBAND_RADS and \
                        abs(prev) > FLIP_DEADBAND_RADS and (x > 0) != (prev > 0):
                    flips += 1
                prev = x
            frac = flips / (len(sp) - 1)
            if frac > FLIP_FRAC:
                vals = [c for _, c in iq] or [cur]
                self.detail = {"trigger": "v_flip", "v_flip": frac,
                               "iq_mean": sum(vals) / len(vals), "t": t}
                return "runaway"
        return None


# ---------------------------------------------------------------- operator interrupts
class Interrupts:
    """Ctrl+C once = end the current step, twice = stop everything. Never raises, so a Ctrl+C can
    not land in the middle of a release and cut it short."""

    def __init__(self):
        self.count = 0

    def install(self):
        try:
            signal.signal(signal.SIGINT, self._handler)
        except ValueError:              # not the main thread (tests)
            pass

    def _handler(self, *_):
        self.count += 1
        print("\n!! Ctrl+C ({}): {}".format(
            self.count, "ending this step" if self.count == 1 else "STOPPING EVERYTHING"),
            flush=True)

    def new_step(self):
        if self.count < 2:
            self.count = 0

    @property
    def end_step(self):
        return self.count >= 1

    @property
    def stop_all(self):
        return self.count >= 2


# ---------------------------------------------------------------- the bench
class StopAll(Exception):
    pass


class Bench:
    def __init__(self, bus, cid, steps, rate_hz=RATE_HZ, release_s=RELEASE_S,
                 release_current_s=RELEASE_CURRENT_S, pause_s=PAUSE_S, clock=time.monotonic,
                 out=print, beep=True, interrupts=None, live_every_s=1.0):
        self.bus, self.cid, self.steps = bus, int(cid), steps
        self.dt = 1.0 / rate_hz
        self.release_s, self.release_current_s, self.pause_s = release_s, release_current_s, pause_s
        self.clock, self.out, self.beep = clock, out, beep
        self.intr = interrupts or Interrupts()
        self.live_every_s = live_every_s
        self.t0 = clock()
        self.t0_wall = datetime.datetime.now().isoformat(timespec="milliseconds")
        self.rows = []
        self.journal = []
        self.table = []
        self.last_rx = None
        self.last_st = None
        self.foreign = 0
        self.tx = {"current": 0, "servo_pos": 0, "mit": 0, "servo_rpm": 0, "limp_mit": 0}
        self.cmd = ("none", 0.0, 0.0, 0.0, 0.0)          # kind, kp, kd, val, v_des
        self.step_idx, self.sub_idx = -1, SUB_PREFLIGHT

    # -- journal
    def log(self, event, **kw):
        e = {"t": self.clock() - self.t0,
             "wall": datetime.datetime.now().isoformat(timespec="milliseconds"),
             "event": event}
        e.update(kw)
        self.journal.append(e)
        self.out("[{:8.3f} s | {}] {}{}".format(
            e["t"], e["wall"][11:], event,
            "".join(" {}={}".format(k, v) for k, v in kw.items())))

    # -- commands
    def _send_current0(self):
        self.cmd = ("current", 0.0, 0.0, 0.0, 0.0)
        self.tx["current"] += 1
        canio.set_current(self.bus, self.cid, 0.0)

    def _send_limp(self):
        self.cmd = ("limp_mit", 0.0, 0.0, 0.0, 0.0)
        self.tx["limp_mit"] += 1
        canio.force_control(self.bus, self.cid, mit.limp_payload())

    # -- reception
    def _rx(self, timeout):
        """Read at most one frame (waiting up to `timeout`), record it, return its status or None.
        Our drive's status frames are recorded; another program's commands are counted."""
        msg = self.bus.recv(timeout=timeout)
        if msg is None or not getattr(msg, "is_extended_id", True):
            return None
        arb = msg.arbitration_id
        if ((arb >> 8) & 0xFF) in FOREIGN_CMDS:
            self.foreign += 1
            return None
        if arb & 0xFF != self.cid:
            return None
        st = canio.parse_status(msg.data)
        if st is None:
            return None
        t = self.clock()
        self.last_rx, self.last_st = t, st
        k, kp, kd, val, vd = self.cmd
        self.rows.append((t - self.t0, st["pos"], st["spd"], st["cur"], st["temp"], st["err"],
                          self.step_idx, self.sub_idx, KIND_CODE[k], kp, kd, val, vd))
        return t, st

    def _pump(self, seconds, send, guards=None, on_tick=None, stoppable=False):
        """Send `send()` every tick for `seconds`, reading every frame in between.
        Returns None, or the outcome that ended it early (only if guards / stoppable)."""
        start = self.clock()
        end = start + seconds
        nxt = start
        while True:
            now = self.clock()
            if now >= end:
                return None
            if now >= nxt:
                send()
                nxt += self.dt
                if nxt < now:                         # fell behind: do not burst to catch up
                    nxt = now + self.dt
                if on_tick is not None:
                    on_tick(now - start)
            r = self._rx(max(0.0, min(nxt, end) - self.clock()))
            if guards is not None:
                if r is not None:
                    why = guards.check(*r)
                    if why:
                        return why
                if self.last_rx is None or self.clock() - self.last_rx > SILENCE_S:
                    return "abort:silent"
                if self.foreign:
                    return "abort:foreign_cmd"
            if stoppable and self.intr.end_step:
                return "abort:operator"

    def release(self, why=""):
        """1 s of limp_payload, THEN SET_CURRENT 0 (daemon._stream_limp). Never cut short."""
        self.log("release", why=why)
        self.sub_idx = SUB_RELEASE
        self._pump(self.release_s, self._send_limp)
        self._pump(self.release_current_s, self._send_current0)

    def _wait_fresh(self, seconds=0.5):
        """Stream SET_CURRENT 0 until a status frame newer than SILENCE_S is in hand."""
        end = self.clock() + seconds
        while self.clock() < end:
            self._send_current0()
            self._rx(self.dt)
            if self.last_rx is not None and self.clock() - self.last_rx < SILENCE_S:
                return True
        return False

    # -- phases of the session
    def preflight(self):
        self.log("preflight")
        self.sub_idx = SUB_PREFLIGHT
        # listen without commanding first: a running daemon shows up as foreign commands
        end = self.clock() + 0.5
        n = 0
        while self.clock() < end:
            if self._rx(0.01) is not None:
                n += 1
        t_listen = 0.5
        if self.foreign:
            raise StopAll("another program is commanding the bus ({} frames in 0.5 s): stop the "
                          "daemon (sudo systemctl stop runningrobot-ui runningrobot-webui)"
                          .format(self.foreign))
        if n == 0:
            raise StopAll("no status frame from id {} in 0.5 s: wrong channel/id, drive off, or "
                          "the drive only answers commands".format(self.cid))
        st = self.last_st
        self.log("preflight_ok", rx_hz=round(n / t_listen, 1), pos_deg=st["pos"],
                 temp=st["temp"], err=st["err"])
        if st["err"]:
            raise StopAll("drive reports error code {}".format(st["err"]))
        if st["temp"] >= TEMP_LIMIT_C:
            raise StopAll("drive at {} C".format(st["temp"]))
        # whatever the drive was holding (a daemon MIT hold included), release it first
        self.release("session start")

    def pause(self, step):
        self.sub_idx = SUB_PAUSE
        self.out("")
        self.out("=" * 78)
        self.out("NEXT: {}  ({:g} s)  {}".format(step["id"], step_seconds(step), describe(step)))
        if step["what"]:
            self.out("      {}".format(step["what"]))
        for name, d, txt in step["subs"]:
            self.out("      {:>5}: {:g} s  {}".format(name, d, txt))
        self.out("      say the step id aloud for the recording")
        self.out("=" * 78)
        if self.beep:
            sys.stdout.write("\a")
            sys.stdout.flush()
        shown = [None]

        def countdown(el):
            left = int(math.ceil(self.pause_s - el))
            if left != shown[0] and left > 0:
                shown[0] = left
                self.out("   starts in {} ...".format(left))
        self._pump(self.pause_s, self._send_current0, on_tick=countdown)

    def run_step(self, k, step):
        """One step. Returns its table row."""
        self.intr.new_step()
        self.step_idx = k
        row = {"id": step["id"], "kind": step["kind"], "kp": step["kp"], "kd": step["kd"],
               "v_des": step["v_des"], "rpm": step["rpm"], "outcome": None, "detail": {}}
        self.pause(step)
        if self.intr.stop_all:
            row["outcome"] = "abort:operator"
            return row
        if not self._wait_fresh():
            row["outcome"] = "abort:silent"
            return row
        pos0 = self.last_st["pos"]
        row["pos0_deg"] = pos0
        send, why = self._command_for(step, pos0)
        if send is None:
            row["outcome"] = "refused:" + why
            self.log("refused", step=step["id"], why=why)
            return row

        guards = Guards(step)
        t_start = self.clock()
        row["t_start"] = t_start - self.t0
        self.log("step_start", step=step["id"], cmd=describe(step), pos0_deg=pos0)
        outcome = None
        live = {"next": 0.0}
        try:
            for j, sub in enumerate(step["subs"]):
                self.sub_idx = j
                name, dur, _txt = sub
                self.log("sub_start", step=step["id"], sub=name)
                cue = self._cue(step, name, dur, live, t_start)
                outcome = self._pump(dur, send, guards=guards, on_tick=cue, stoppable=True)
                if outcome:
                    break
        except Exception as e:                       # noqa: BLE001 -- release, then re-raise
            row["outcome"] = "abort:exception"
            row["detail"] = {"error": repr(e)}
            row["t_end"] = self.clock() - self.t0
            self.log("step_end", step=step["id"], outcome=row["outcome"])
            self.table.append(row)
            self.release("exception")
            self._released_on_exception = True
            raise
        row["t_end"] = self.clock() - self.t0
        row["outcome"] = outcome or "ok"
        row["detail"] = guards.detail
        if self.last_st is not None:
            row["detail"] = dict(row["detail"], last_iq=self.last_st["cur"],
                                 last_spd_erpm=self.last_st["spd"])
        self.log("step_end", step=step["id"], outcome=row["outcome"],
                 **({"trigger": guards.detail.get("trigger"),
                     "iq_mean": round(guards.detail.get("iq_mean", float("nan")), 3)}
                    if guards.detail else {}))
        self.release(row["outcome"])
        return row

    def _cue(self, step, name, dur, live, t_start):
        """Countdown / live line, once per second."""
        shown = [None]

        def cue(el):
            sec = int(el)
            if sec == shown[0]:
                return
            shown[0] = sec
            left = int(math.ceil(dur - el))
            if name == "press":
                word = "APPUIE PROGRESSIVEMENT" if el < PRESS_RELEASE_AT else "RELÂCHE"
            elif name == "tap":
                word = "TAPE MAINTENANT (une fois)" if 1.5 <= el < 3.0 else "attends"
            elif name == "turn":
                word = "fais tourner lentement"
            else:
                word = "ne touche pas"
            st = self.last_st or {"pos": float("nan"), "spd": float("nan"),
                                  "cur": float("nan"), "temp": float("nan")}
            vpos = self._v_from_pos(0.5)
            self.out("  {:>4} {:>2}s  {:<26} pos {:+8.1f}  v {:+6.2f} rad/s (pos {:+6.2f})  "
                     "Iq {:+6.2f} A  {:>3.0f} C".format(
                         step["id"], left, word, st["pos"], erpm_to_rads(st["spd"]),
                         vpos, st["cur"], st["temp"]))
        return cue

    def _v_from_pos(self, window):
        """Output speed from the recorded positions over the last `window` s (rad/s)."""
        if len(self.rows) < 2:
            return float("nan")
        t_end = self.rows[-1][0]
        i = len(self.rows) - 1
        while i > 0 and t_end - self.rows[i - 1][0] <= window:
            i -= 1
        dt = t_end - self.rows[i][0]
        if dt <= 0:
            return float("nan")
        return math.radians((self.rows[-1][1] - self.rows[i][1]) / dt)

    def _command_for(self, step, pos0):
        """(send function, None) or (None, why refused). Every frame is built here, once."""
        k = step["kind"]
        if k == "current":
            return self._send_current0, None
        if k == "servo_pos":
            def send_pos():
                self.cmd = ("servo_pos", 0.0, 0.0, pos0, 0.0)
                self.tx["servo_pos"] += 1
                canio.set_pos(self.bus, self.cid, pos0)
            return send_pos, None
        if k == "servo_rpm":
            erpm = step["rpm"]

            def send_rpm():
                self.cmd = ("servo_rpm", 0.0, 0.0, erpm, 0.0)
                self.tx["servo_rpm"] += 1
                set_rpm(self.bus, self.cid, erpm)
            return send_rpm, None
        if k != "mit":
            return None, "unknown kind " + k
        kp, kd, vd = step["kp"], step["kd"], step["v_des"]
        if kp > KP_LIMIT or kd > KD_LIMIT or kp < 0 or kd < 0:
            return None, "gains kp {:g} kd {:g} outside 0..{:g} / 0..{:g}".format(
                kp, kd, KP_LIMIT, KD_LIMIT)
        p_des = math.radians(pos0)
        if abs(p_des) >= mit.P_MAX:
            return None, "position {:+.1f} deg outside the MIT frame's +-720 deg".format(pos0)
        if vd:
            # time at v_des must keep the shaft inside the MIT frame (the bench does not unwind)
            travel = abs(vd) * step_seconds(step)
            if abs(p_des) + travel >= mit.P_MAX:
                return None, ("{:+.0f} deg + {:.0f} deg of rotation leaves the MIT frame: turn the "
                              "cam back toward 0 by hand".format(pos0, math.degrees(travel)))
            payload, clamped = mit.pack(p_des, kp, kd, v_des=vd, v_range=AKE90_V_RANGE)
        else:
            payload, clamped = mit.pack(p_des, kp, kd)      # no tau_ff in phase A
        if clamped:
            return None, "mit.pack clamped " + ",".join(clamped)
        if len(payload) != mit.DLC:
            return None, "payload of {} bytes".format(len(payload))

        def send_mit():
            self.cmd = ("mit", kp, kd, pos0, vd)
            self.tx["mit"] += 1
            canio.force_control(self.bus, self.cid, payload)
        return send_mit, None

    # -- session
    def run(self):
        self.intr.install()
        stop_reason = None
        try:
            self.preflight()
            for k, step in enumerate([SYNC_STEP] + list(self.steps)):
                row = self.run_step(k, step)
                self.table.append(row)
                o = row["outcome"]
                if self.intr.stop_all:
                    stop_reason = "operator (Ctrl+C twice)"
                elif o in ("abort:temp", "abort:error", "abort:silent", "abort:foreign_cmd"):
                    stop_reason = o
                elif step is SYNC_STEP and o != "ok":
                    self.log("warning", why="SYNC ended " + o + ": align the audio by hand")
                if stop_reason:
                    self.log("stop_all", why=stop_reason)
                    break
        except StopAll as e:
            stop_reason = str(e)
            self.log("stop_all", why=stop_reason)
            self.release("stop_all")
        except BaseException as e:                   # noqa: BLE001 -- release, then re-raise
            self.log("exception", error=repr(e))
            if not getattr(self, "_released_on_exception", False):
                self.release("exception")
            raise
        finally:
            self.sub_idx = SUB_RELEASE
        self.stop_reason = stop_reason
        self._fill_metrics()
        return self.table

    def frames(self):
        if not self.rows:
            return np.zeros((0, len(COLUMNS)))
        return np.asarray(self.rows, dtype=float)

    def _fill_metrics(self):
        F = self.frames()
        for k, row in enumerate(self.table):
            if F.size:
                m = (F[:, 6] == k) & (F[:, 7] >= 0)
                row.update(step_metrics(F[m, 0], F[m, 3], F[m, 2], F[m, 4], F[m, 1]))
            else:
                row.update(step_metrics([], [], [], []))

    def save(self, path):
        meta = {"tool": "mit_bench", "cid": self.cid, "t0_wall": self.t0_wall,
                "columns": list(COLUMNS), "kinds": list(KINDS),
                "sub_codes": {"pause": SUB_PAUSE, "release": SUB_RELEASE,
                              "preflight": SUB_PREFLIGHT},
                "steps": [SYNC_STEP] + list(self.steps), "rate_hz": 1.0 / self.dt,
                "erpm_per_rads": ERPM_PER_RADS, "v_range_mit": AKE90_V_RANGE,
                "limits": {"kp": KP_LIMIT, "kd": KD_LIMIT, "I_A": I_LIMIT_A,
                           "temp_C": TEMP_LIMIT_C, "silence_s": SILENCE_S,
                           "speed_rads": SPEED_LIMIT_RADS, "runaway_std_A": RUNAWAY_STD_A,
                           "runaway_hold_s": RUNAWAY_HOLD_S, "flip_frac": FLIP_FRAC,
                           "flip_deadband_rads": FLIP_DEADBAND_RADS},
                "tx": self.tx, "foreign_cmd_frames": self.foreign,
                "stop_reason": getattr(self, "stop_reason", None),
                "send_stats": canio.send_stats()}
        np.savez_compressed(path, frames=self.frames(), meta_json=json.dumps(meta),
                            steps_json=json.dumps(self.table), journal_json=json.dumps(self.journal))
        return path


def fmt(v, nd=2):
    if v is None:
        return "-"
    if isinstance(v, float):
        if math.isnan(v):
            return "-"
        return ("{:.%df}" % nd).format(v)
    return str(v)


def summary_lines(table):
    keys = ("rx_hz", "I_mean", "I_ripple_A", "v_flip", "I_max", "temp_max", "v_erpm_rads",
            "v_pos_rads")
    hdr = "{:<5} {:>7} {:>7} {:>10} {:>6} {:>6} {:>8} {:>8} {:>8}  {}".format(
        "step", "rx_Hz", "I_mean", "I_ripple_A", "v_flip", "I_max", "temp_max", "v_rad/s",
        "v_pos", "outcome")
    lines = [hdr, "-" * len(hdr)]
    for r in table:
        vals = [fmt(r.get(k), 1 if k in ("rx_hz", "temp_max") else 2) for k in keys]
        lines.append("{:<5} {:>7} {:>7} {:>10} {:>6} {:>6} {:>8} {:>8} {:>8}  {}".format(
            r["id"], *vals, r["outcome"]))
    return lines


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--channel", default="can1")
    ap.add_argument("--id", type=int, default=105)
    ap.add_argument("--interface", default="socketcan")
    ap.add_argument("--phase", default="A", choices=["A", "B"])
    ap.add_argument("--steps", default=None, help="comma list of step ids or prefixes (A2,A3b)")
    ap.add_argument("--dry-run", action="store_true", help="print the plan and exit, bus untouched")
    ap.add_argument("--mock", action="store_true", help="canio MockBus (offline)")
    ap.add_argument("--yes", action="store_true", help="do not ask for the go")
    ap.add_argument("--out", default=None, help="session .npz (default mit_bench_<id>_<ts>.npz)")
    ap.add_argument("--no-beep", action="store_true")
    a = ap.parse_args(argv)

    if a.phase == "B":
        print("phase B (blocked output, MIT vs SET_CURRENT) is not written yet -- see the plan §4.3")
        return 2
    try:
        steps = select_steps(PHASE_A, a.steps)
    except ValueError as e:
        print("!! " + str(e))
        return 2
    print("mit_bench phase {}  channel {} id {}{}".format(
        a.phase, a.channel, a.id, "  [MOCK]" if a.mock else ""))
    print("SYNC (always first): {}".format(SYNC_STEP["what"]))
    print_plan(steps)
    if a.dry_run:
        print("dry run: bus not opened")
        return 0
    if not a.yes:
        print("\nCheck: cam NOT linked to the leg, daemon stopped, battery 50-51 V, charger off, "
              "E-STOP in reach, phone RECORDING 20 cm from the cam.")
        try:
            if input("Type 'go' to start: ").strip().lower() != "go":
                print("not started")
                return 1
        except EOFError:
            return 1
    out = a.out or "mit_bench_{}_{}.npz".format(a.id, time.strftime("%Y%m%d_%H%M%S"))
    bus = canio.open_buses(a.interface, [a.channel], mock=a.mock)[a.channel]
    bench = Bench(bus, a.id, steps, beep=not a.no_beep)
    try:
        bench.run()
    finally:
        try:
            bench._fill_metrics()
            bench.save(out)
            print("\nsaved {}  ({} frames)".format(out, len(bench.rows)))
        finally:
            bus.shutdown()
    print("")
    for line in summary_lines(bench.table):
        print(line)
    if bench.stop_reason:
        print("\nSTOPPED: {}".format(bench.stop_reason))
    print("tx frames: {}   foreign command frames: {}".format(bench.tx, bench.foreign))
    return 0 if not bench.stop_reason else 1


if __name__ == "__main__":
    sys.exit(main())
