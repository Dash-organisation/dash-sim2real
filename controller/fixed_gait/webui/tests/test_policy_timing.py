"""Where a policy control tick's time goes (plan B, step 0 -- 2026-11-18).

The governor stopped every standing run of 2026-11-17 for the loop not holding 90% of 100 Hz, and
the only numbers were the arm-time probe and the black box's loop `dt`. The run now times each
station of the control tick (POLICY_TIMING_COLS) into a preallocated buffer and reports p50 / p90 /
max in policy.done, so the before/after of moving ctrl.step out of the daemon is measured on the
robot rather than argued about.

MockBus only -- no hardware, no CAN, no robot.
"""
import numpy as np

import daemon as daemon_mod
from test_blackbox import capture_zero, robot, wait_mode            # noqa: F401 (pytest fixtures)
from test_policy_run import (armed, keep_alive_until, needs_bundle, spec,  # noqa: F401
                             staged)


def capture_events(d):
    """Every black-box event the daemon logs, as (kind, fields), in order."""
    seen = []
    orig = d._bb_event

    def rec(kind, /, **fields):
        seen.append((kind, fields))
        orig(kind, **fields)
    d._bb_event = rec
    return seen


def done_event(seen):
    got = [f for k, f in seen if k == "policy.done"]
    assert len(got) == 1, [k for k, _ in seen]
    return got[0]


def test_the_stats_are_none_before_the_first_run_tick():
    assert daemon_mod.policy_timing_stats(np.zeros((10, len(daemon_mod.POLICY_TIMING_COLS))), 0) \
        is None


def test_the_stats_are_per_station_percentiles():
    tm = np.zeros((100, len(daemon_mod.POLICY_TIMING_COLS)))
    tm[:, 1] = np.arange(100)                    # ctrl: 0..99 ms
    st = daemon_mod.policy_timing_stats(tm, 100)
    assert set(st) == set(daemon_mod.POLICY_TIMING_COLS)
    assert st["ctrl"]["max"] == 99.0
    assert abs(st["ctrl"]["p50"] - 49.5) < 1e-9
    assert abs(st["ctrl"]["p90"] - 89.1) < 1e-9
    # only the rows that were written count
    st = daemon_mod.policy_timing_stats(tm, 10)
    assert st["ctrl"]["max"] == 9.0


@needs_bundle
def test_policy_done_says_where_the_tick_went(staged):
    d, _cal, fname = staged
    seen = capture_events(d)
    ok, why, _ = d.policy_arm(spec(file=fname, max_seconds=1.0))
    assert ok, why
    assert wait_mode(d, "POLICY")
    p = keep_alive_until(d, lambda p: p["phase"] == "done", timeout=30.0)
    assert p["reached_run"], p["exit_reason"]
    ev = done_event(seen)
    tm = ev["timing_ms"]
    assert set(tm) == set(daemon_mod.POLICY_TIMING_COLS)
    assert ev["timing_ticks"] > 50
    assert ev["tick_budget_ms"] == 10.0
    assert ev["realised_hz"] > 0
    for name, st in tm.items():
        assert 0.0 <= st["p50"] <= st["p90"] <= st["max"], (name, st)
    # the whole tick contains every station of it
    for k in ("p50", "p90", "max"):
        assert tm["total"][k] >= tm["ctrl"][k] - 1e-6
    assert tm["ctrl"]["p50"] > 0.0, "ctrl.step cannot take no time"
    # each row is internally consistent, not just the summaries
    pol = d._pol
    rows = pol["tm"][:pol["tm_n"]]
    parts = rows[:, :6].sum(axis=1)                # measure..log, without rtt
    assert np.all(rows[:, 6] + 1e-6 >= parts), "a station took longer than the whole tick"
    got = d.get_policy()
    assert got["timing_ms"]["total"]["p90"] == tm["total"]["p90"]


@needs_bundle
def test_the_live_status_carries_the_timing(staged):
    d, _cal, fname = staged
    ok, why, _ = d.policy_arm(spec(file=fname, max_seconds=30.0))
    assert ok, why
    p = keep_alive_until(d, lambda p: p["phase"] == "run" and p.get("timing_ms"), timeout=30.0)
    assert p["phase"] == "run", p.get("exit_reason")
    assert p["tick_budget_ms"] == 10.0
    assert set(p["timing_ms"]["total"]) == {"p50", "p90"}
    d.policy_stop(hard=True)
