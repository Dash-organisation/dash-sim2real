"""tools/mit_bench/analyze.py on a synthetic session: audio alignment on the SYNC tap, dominant
lines above A0, and the same step metrics as the bench itself.

    python -m pytest tools/mit_bench/test_analyze.py
"""
import json
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
import analyze as A                                                    # noqa: E402

COLUMNS = ("t", "pos_deg", "spd_erpm", "cur_A", "temp_C", "err", "step", "sub",
           "cmd_kind", "cmd_kp", "cmd_kd", "cmd_val", "cmd_v_des")
STEPS = [("SYNC", [["tap", 5.0, ""]]), ("A0", [["still", 5.0, ""]]),
         ("A2d", [["still", 5.0, ""], ["press", 10.0, ""]])]
TAP_T = 3.0             # session time of the tap (SYNC runs 1..6 s)
OFFSET = 7.3            # audio starts 7.3 s before the session
FS = 48000


def make_session(path, rng):
    rows, table, t, meta_steps = [], [], 0.0, []
    for k, (sid, subs) in enumerate(STEPS):
        for _ in range(int(1.0 * 200)):                    # 1 s of pause before each step
            t += 0.005
            rows.append([t, 10, 0, 0, 35, 0, k, -1, 1, 0, 0, 0, 0])
        t_start = t
        for j, (name, dur, _) in enumerate(subs):
            for _ in range(int(dur * 200)):
                t += 0.005
                cur = rng.normal(0, 0.02)
                spd = 0.0
                if sid == "SYNC" and abs(t - TAP_T) < 0.006:
                    cur, spd = 1.5, 800.0
                if sid == "A2d" and name == "press":
                    cur = 4.0 + rng.normal(0, 0.5)
                rows.append([t, 10, spd, cur, 36, 0, k, j, 3, 50, 1, 10, 0])
        table.append({"id": sid, "kind": "mit", "kp": 50.0 if sid == "A2d" else 0.0,
                      "kd": 1.0 if sid == "A2d" else 0.0, "outcome": "ok", "detail": {},
                      "t_start": t_start, "t_end": t})
        meta_steps.append({"id": sid, "subs": subs})
    meta = {"columns": list(COLUMNS), "steps": meta_steps, "cid": 105, "rate_hz": 100.0,
            "erpm_per_rads": A.ERPM_PER_RADS_DEFAULT}
    np.savez_compressed(path, frames=np.asarray(rows, float), meta_json=json.dumps(meta),
                        steps_json=json.dumps(table), journal_json=json.dumps([]))
    return t


def make_audio(path, t_end, rng):
    from scipy.io import wavfile
    n = int((t_end + OFFSET + 2.0) * FS)
    x = rng.normal(0, 0.003, n)
    i = int((TAP_T + OFFSET) * FS)
    x[i:i + 200] += rng.normal(0, 0.5, 200) * np.exp(-np.arange(200) / 40.0)     # the tap
    # the A2d step (pauses 1 s each, steps 5/5/15 s): session t in [13, 28] -> a 734 Hz line
    a0, a1 = int((13.0 + OFFSET) * FS), int((28.0 + OFFSET) * FS)
    tt = np.arange(a1 - a0) / FS
    x[a0:a1] += 0.05 * np.sin(2 * np.pi * 734.0 * tt)
    wavfile.write(path, FS, (np.clip(x, -1, 1) * 32767).astype(np.int16))


@pytest.fixture
def session(tmp_path):
    rng = np.random.default_rng(1)
    npz = str(tmp_path / "s.npz")
    wav = str(tmp_path / "a.wav")
    t_end = make_session(npz, rng)
    make_audio(wav, t_end, rng)
    return npz, wav, str(tmp_path / "out")


def test_alignment_and_lines(session):
    npz, wav, out = session
    res, out = A.analyse(npz, audio=wav, out=out)
    a = res["audio"]
    assert a["data_tap_t"] == pytest.approx(TAP_T, abs=0.01)
    assert a["offset_s"] == pytest.approx(OFFSET, abs=0.01)
    lines = a["segments"]["A2d"]["lines"]
    assert lines and abs(lines[0]["f_Hz"] - 734.0) < 6.0
    assert lines[0]["dB_above_A0"] > 20.0
    assert a["reference"] == "A0"
    for f in ("summary.md", "summary.json", "spectrogram.png", "step_A2d.png"):
        assert os.path.exists(os.path.join(out, f)), f
    md = open(os.path.join(out, "summary.md")).read()
    assert "A2d.press" in md and "| A0 |" in md


def test_metrics_match_the_bench(session):
    sys.path.insert(0, os.path.join(ROOT, "controller", "tools"))
    import mit_bench
    npz, _, _ = session
    S = A.load_session(npz)
    F, c = S["F"], S["c"]
    m = (F[:, c["step"]] == 2) & (F[:, c["sub"]] >= 0)
    f = F[m]
    ours = A.segment_metrics(S, 2, None)
    theirs = mit_bench.step_metrics(f[:, c["t"]], f[:, c["cur_A"]], f[:, c["spd_erpm"]],
                                    f[:, c["temp_C"]], f[:, c["pos_deg"]])
    for k in ("rx_hz", "I_mean", "I_ripple_A", "v_flip", "I_max", "temp_max", "v_erpm_rads"):
        assert ours[k] == pytest.approx(theirs[k]), k


def test_no_audio(session):
    npz, _, out = session
    res, out = A.analyse(npz, out=out)
    assert res["audio"] is None
    assert [r["segment"] for r in res["segments"]] == ["SYNC", "A0", "A2d", "A2d.still",
                                                       "A2d.press"]
    press = next(r for r in res["segments"] if r["segment"] == "A2d.press")
    assert press["I_mean"] == pytest.approx(4.0, abs=0.1)
