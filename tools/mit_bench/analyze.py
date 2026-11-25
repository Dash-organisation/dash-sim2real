#!/usr/bin/env python3
"""Analyse a free-cam bench session (controller/tools/mit_bench.py), optionally with the phone audio.

    python tools/mit_bench/analyze.py mit_bench_105_<ts>.npz [--audio B.m4a] [--out DIR]
           [--sync-audio-t S] [--sync-data-t S] [--sync-search S]

Input: the session .npz written on the Pi (every status frame of the drive, the step table, the
journal). Plan: docs/sim2real/DASH01_plan_banc_came_2026-11-25.md.

Without audio, per step (and per sub-phase: still / press / turn):
  rx_hz         feedback rate actually received
  I_mean        mean Iq (A)
  I_ripple_A    std of (Iq - its 90 ms centred moving mean)     [same as kd_validation/analyze_runs]
  v_flip        fraction of consecutive frames whose speed changes sign (no deadband)
  I_max         max |Iq|;  temp_max;  v_rad/s (ERPM / 1604) and v_pos (from the position)
  outcome       as the bench recorded it (ok / runaway / abort:<why> / refused:<why>)
plus one figure per step (Iq, speed, temperature) and a session overview.

With --audio (wav, or m4a/mp3/... converted with ffmpeg, or afconvert on macOS):
  * alignment on the SYNC step: the tap is the first sharp Iq/speed excursion of the SYNC frames,
    and in the audio the strongest broadband onset in the first --sync-search seconds (default
    180 s). Both times can be forced (--sync-data-t: session time, s; --sync-audio-t: audio time, s)
    when the automatic pick is wrong -- the candidates are listed in summary.md.
  * per segment, the mean spectrum (Welch, ~2.9 Hz bins) and its dominant lines between 200 Hz and
    10 kHz: frequency, level in dB above the A0 spectrum (the pauses if A0 was not run), dBFS.
  * a global spectrogram with the steps marked.

Writes <out>/summary.md, <out>/summary.json and the figures (default out: next to the .npz,
<name>_analysis/).
"""
import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np

ERPM_PER_RADS_DEFAULT = 21 * 8.0 * 60.0 / (2.0 * math.pi)
MA_WIN_S = 0.090
F_LO, F_HI = 200.0, 10000.0
N_LINES = 8
LINE_PROMINENCE_DB = 6.0
MIN_ABOVE_REF_DB = 6.0       # a line must stand this far above the reference spectrum
SUB_PAUSE = -1


# ------------------------------------------------------------------ session
def load_session(path):
    z = np.load(path, allow_pickle=False)
    meta = json.loads(str(z["meta_json"]))
    F = z["frames"].astype(float)
    cols = {c: i for i, c in enumerate(meta["columns"])}
    return {"F": F, "c": cols, "meta": meta, "steps": json.loads(str(z["steps_json"])),
            "journal": json.loads(str(z["journal_json"])), "path": path}


def movmean_centered(x, n):
    """Same as mit_bench.movmean_centered."""
    x = np.asarray(x, float)
    if n < 2 or x.size == 0:
        return x.copy()
    k = np.ones(n) / n
    pad = np.pad(x, (n // 2, n - 1 - n // 2), mode="edge")
    return np.convolve(pad, k, mode="valid")


def metrics(t, cur, spd, temp, pos, erpm_per_rads):
    """Same definitions as mit_bench.step_metrics (the test checks they agree)."""
    n = len(t)
    if n < 2:
        return {"frames": n, "rx_hz": None, "I_mean": None, "I_ripple_A": None, "v_flip": None,
                "I_max": None, "temp_max": None, "v_erpm_rads": None, "v_pos_rads": None}
    dur = t[-1] - t[0]
    rx = (n - 1) / dur if dur > 0 else None
    w = max(2, int(round(MA_WIN_S * (rx or 100.0))))
    ripple = cur - movmean_centered(cur, w)
    return {"frames": int(n), "rx_hz": rx, "I_mean": float(cur.mean()),
            "I_ripple_A": float(ripple.std()),
            "v_flip": float(np.mean(np.sign(spd[1:]) * np.sign(spd[:-1]) < 0)),
            "I_max": float(np.abs(cur).max()), "temp_max": float(np.max(temp)),
            "v_erpm_rads": float(spd.mean() / erpm_per_rads),
            "v_pos_rads": float(math.radians((pos[-1] - pos[0]) / dur)) if dur > 0 else None}


def segments(S):
    """[(label, step_index, sub_index or None, t0, t1)] in session time: one per step, plus one per
    sub-phase when a step has several."""
    F, c = S["F"], S["c"]
    steps_meta = S["meta"]["steps"]
    out = []
    for k, row in enumerate(S["steps"]):
        m = (F[:, c["step"]] == k) & (F[:, c["sub"]] >= 0)
        if not m.any():
            continue
        t = F[m, c["t"]]
        out.append((row["id"], k, None, float(t[0]), float(t[-1])))
        subs = steps_meta[k]["subs"] if k < len(steps_meta) else []
        if len(subs) > 1:
            for j, sub in enumerate(subs):
                mj = m & (F[:, c["sub"]] == j)
                if mj.sum() > 1:
                    tj = F[mj, c["t"]]
                    out.append(("{}.{}".format(row["id"], sub[0]), k, j, float(tj[0]),
                                float(tj[-1])))
    return out


def segment_metrics(S, k, j):
    F, c = S["F"], S["c"]
    m = (F[:, c["step"]] == k) & (F[:, c["sub"]] >= 0)
    if j is not None:
        m &= F[:, c["sub"]] == j
    f = F[m]
    return metrics(f[:, c["t"]], f[:, c["cur_A"]], f[:, c["spd_erpm"]], f[:, c["temp_C"]],
                   f[:, c["pos_deg"]], S["meta"].get("erpm_per_rads", ERPM_PER_RADS_DEFAULT))


def data_tap_time(S):
    """Session time of the SYNC tap: first frame of the SYNC step where |Iq| or |speed| (each about
    its median, each over its own max) reaches half of its peak. None if both stay flat."""
    F, c = S["F"], S["c"]
    k = next((i for i, r in enumerate(S["steps"]) if r["id"] == "SYNC"), None)
    if k is None:
        return None, "no SYNC step in the session"
    m = (F[:, c["step"]] == k) & (F[:, c["sub"]] >= 0)
    if m.sum() < 5:
        return None, "SYNC step has no frames"
    t = F[m, c["t"]]
    score = np.zeros(m.sum())
    used = []
    for name in ("cur_A", "spd_erpm"):
        x = np.abs(F[m, c[name]] - np.median(F[m, c[name]]))
        if x.max() > 0:
            score = np.maximum(score, x / x.max())
            used.append(name)
    if not used:
        return None, "Iq and speed flat during SYNC (no tap seen)"
    i = int(np.argmax(score >= 0.5))
    return float(t[i]), "first SYNC frame at half peak of " + "/".join(used)


# ------------------------------------------------------------------ audio
def load_audio(path):
    """(samples float mono in [-1, 1], fs). wav is read directly; anything else is converted."""
    from scipy.io import wavfile
    tmp = None
    if not path.lower().endswith(".wav"):
        tmp = tempfile.mkdtemp(prefix="mit_bench_audio_")
        wav = os.path.join(tmp, "audio.wav")
        if shutil.which("ffmpeg"):
            cmd = ["ffmpeg", "-loglevel", "error", "-y", "-i", path, "-ac", "1", wav]
        elif shutil.which("afconvert"):
            cmd = ["afconvert", "-f", "WAVE", "-d", "LEI16", "-c", "1", path, wav]
        else:
            raise RuntimeError("cannot convert {}: install ffmpeg (brew install ffmpeg) or pass "
                               "a .wav".format(path))
        subprocess.run(cmd, check=True)
        path = wav
    try:
        fs, x = wavfile.read(path)
    finally:
        if tmp:
            shutil.rmtree(tmp, ignore_errors=True)
    if x.ndim > 1:
        x = x.mean(axis=1)
    if np.issubdtype(x.dtype, np.integer):
        x = x.astype(float) / float(np.iinfo(x.dtype).max)
    return x.astype(float), int(fs)


def audio_onsets(x, fs, search_s, hop_s=0.002, base_s=0.2):
    """Broadband onsets (energy above 2 kHz in 2 ms hops, over the median of the 200 ms before).
    Returns [(t, strength)] strongest first, at most one per 0.5 s."""
    from scipy.signal import butter, sosfilt
    n = min(len(x), int(search_s * fs))
    sos = butter(4, 2000.0, btype="high", fs=fs, output="sos")
    y = sosfilt(sos, x[:n])
    hop = max(1, int(hop_s * fs))
    m = len(y) // hop
    if m < 10:
        return []
    e = np.sqrt(np.mean(y[:m * hop].reshape(m, hop) ** 2, axis=1)) + 1e-9
    nb = max(1, int(base_s / hop_s))
    strength = np.zeros(m)
    for i in range(nb, m):
        strength[i] = e[i] / (np.median(e[i - nb:i]) + 1e-9)
    order = np.argsort(strength)[::-1]
    picked = []
    for i in order:
        t = i * hop_s
        if strength[i] < 3.0 or len(picked) >= 5:
            break
        if all(abs(t - p[0]) > 0.5 for p in picked):
            picked.append((float(t), float(strength[i])))
    return picked


def spectrum(x, fs):
    from scipy.signal import welch
    nper = min(len(x), 16384 if fs >= 32000 else 8192)
    if nper < 256:
        return None, None
    f, P = welch(x, fs=fs, nperseg=nper, noverlap=nper // 2, window="hann")
    return f, P


def dominant_lines(f, P, Pref, n=N_LINES):
    """Peaks of the dB spectrum in [F_LO, F_HI] with >= 6 dB prominence and >= 6 dB above the
    reference spectrum (A0), ranked by that level above the reference."""
    from scipy.signal import find_peaks
    band = (f >= F_LO) & (f <= F_HI)
    db = 10.0 * np.log10(P + 1e-20)
    idx, _ = find_peaks(db[band], prominence=LINE_PROMINENCE_DB)
    ib = np.flatnonzero(band)[idx]
    if Pref is not None:
        above = db[ib] - 10.0 * np.log10(Pref[ib] + 1e-20)
    else:
        above = np.full(len(ib), np.nan)
    order = np.argsort(-np.nan_to_num(above, nan=-1e9) if Pref is not None else -db[ib])
    out = []
    for o in order:
        if Pref is not None and not above[o] >= MIN_ABOVE_REF_DB:
            continue
        out.append({"f_Hz": float(f[ib[o]]), "dB_above_A0": float(above[o]),
                    "dBFS": float(db[ib[o]])})
        if len(out) >= n:
            break
    return out


# ------------------------------------------------------------------ figures
def _plt():
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        return plt
    except ImportError:
        return None


def step_figures(S, out):
    plt = _plt()
    if plt is None:
        return []
    F, c = S["F"], S["c"]
    epr = S["meta"].get("erpm_per_rads", ERPM_PER_RADS_DEFAULT)
    made = []
    for k, row in enumerate(S["steps"]):
        m = F[:, c["step"]] == k            # the whole step, pause and release included
        if m.sum() < 2:
            continue
        f = F[m]
        t = f[:, c["t"]]
        act = f[:, c["sub"]] >= 0
        fig, ax = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
        ax[0].plot(t, f[:, c["cur_A"]], lw=0.7)
        ax[0].set_ylabel("Iq (A)")
        ax[1].plot(t, f[:, c["spd_erpm"]] / epr, lw=0.7, label="ERPM / {:.0f}".format(epr))
        if len(t) > 2:
            vp = np.radians(np.gradient(f[:, c["pos_deg"]], t))
            ax[1].plot(t, movmean_centered(vp, 9), lw=0.7, alpha=0.7, label="d(pos)/dt")
        ax[1].set_ylabel("speed (rad/s)")
        ax[1].legend(fontsize=8)
        ax[2].plot(t, f[:, c["temp_C"]], lw=0.8)
        ax[2].set_ylabel("temp (C)")
        ax[2].set_xlabel("session time (s)")
        if act.any():
            for a in ax:
                a.axvspan(t[act][0], t[act][-1], color="0.9", zorder=0)
        ax[0].set_title("{}  kp {:g} kd {:g}  {}".format(row["id"], row.get("kp", 0),
                                                         row.get("kd", 0), row["outcome"]))
        fig.tight_layout()
        p = os.path.join(out, "step_{}.png".format(row["id"]))
        fig.savefig(p, dpi=110)
        plt.close(fig)
        made.append(p)
    # overview
    fig, ax = plt.subplots(2, 1, figsize=(14, 6), sharex=True)
    t = F[:, c["t"]]
    ax[0].plot(t, F[:, c["cur_A"]], lw=0.5)
    ax[1].plot(t, F[:, c["spd_erpm"]] / epr, lw=0.5)
    ax[0].set_ylabel("Iq (A)")
    ax[1].set_ylabel("speed (rad/s)")
    ax[1].set_xlabel("session time (s)")
    for lab, k, j, t0, t1 in segments(S):
        if j is None:
            for a in ax:
                a.axvspan(t0, t1, color="C1", alpha=0.08)
            ax[0].text((t0 + t1) / 2, 1.0, lab, transform=ax[0].get_xaxis_transform(),
                       ha="center", va="bottom", fontsize=7)
    fig.tight_layout()
    p = os.path.join(out, "session_overview.png")
    fig.savefig(p, dpi=110)
    plt.close(fig)
    return made + [p]


def spectrogram_figure(x, fs, segs, offset, out):
    plt = _plt()
    if plt is None:
        return None
    fig, ax = plt.subplots(figsize=(16, 6))
    nfft = 4096 if fs >= 32000 else 2048
    ax.specgram(x, NFFT=nfft, Fs=fs, noverlap=nfft // 2, cmap="magma", vmin=-140)
    ax.set_ylim(F_LO, F_HI)
    ax.set_xlabel("audio time (s)")
    ax.set_ylabel("Hz")
    for lab, k, j, t0, t1 in segs:
        if j is None:
            a0, a1 = t0 + offset, t1 + offset
            ax.axvline(a0, color="w", lw=0.6)
            ax.axvline(a1, color="w", lw=0.6, ls=":")
            ax.text((a0 + a1) / 2, F_HI * 0.97, lab, color="w", ha="center", va="top",
                    fontsize=7)
    fig.tight_layout()
    p = os.path.join(out, "spectrogram.png")
    fig.savefig(p, dpi=110)
    plt.close(fig)
    return p


def spectra_figure(spectra, out):
    plt = _plt()
    if plt is None or not spectra:
        return None
    fig, ax = plt.subplots(figsize=(12, 6))
    for lab, (f, P) in spectra.items():
        band = (f >= F_LO) & (f <= F_HI)
        ax.semilogx(f[band], 10 * np.log10(P[band] + 1e-20), lw=0.7, label=lab)
    ax.set_xlabel("Hz")
    ax.set_ylabel("dB")
    ax.legend(fontsize=7, ncol=3)
    fig.tight_layout()
    p = os.path.join(out, "spectra.png")
    fig.savefig(p, dpi=110)
    plt.close(fig)
    return p


# ------------------------------------------------------------------ report
def fmt(v, nd=2):
    if v is None:
        return "—"
    if isinstance(v, float):
        return "—" if math.isnan(v) else ("{:.%df}" % nd).format(v)
    return str(v)


def analyse(npz, audio=None, out=None, sync_audio_t=None, sync_data_t=None, sync_search=180.0):
    S = load_session(npz)
    if out is None:
        out = os.path.splitext(npz)[0] + "_analysis"
    os.makedirs(out, exist_ok=True)
    segs = segments(S)
    outcome = {r["id"]: r["outcome"] for r in S["steps"]}
    rows = []
    for lab, k, j, t0, t1 in segs:
        r = {"segment": lab, "step": S["steps"][k]["id"], "t0": t0, "t1": t1,
             "outcome": outcome[S["steps"][k]["id"]] if j is None else ""}
        r.update(segment_metrics(S, k, j))
        rows.append(r)
    res = {"session": os.path.basename(npz), "meta": {k: S["meta"].get(k) for k in
                                                     ("cid", "t0_wall", "rate_hz", "tx",
                                                      "foreign_cmd_frames", "stop_reason")},
           "segments": rows, "steps": S["steps"], "audio": None}
    figs = step_figures(S, out)

    if audio:
        x, fs = load_audio(audio)
        a = {"file": os.path.basename(audio), "fs": fs, "seconds": len(x) / fs}
        if sync_data_t is None:
            sync_data_t, why = data_tap_time(S)
            a["data_tap_how"] = why
        cands = audio_onsets(x, fs, sync_search)
        a["audio_onset_candidates"] = cands
        if sync_audio_t is None:
            sync_audio_t = cands[0][0] if cands else None
            a["audio_tap_how"] = "strongest onset in the first {:.0f} s".format(sync_search)
        a["data_tap_t"], a["audio_tap_t"] = sync_data_t, sync_audio_t
        if sync_data_t is None or sync_audio_t is None:
            a["error"] = "alignment impossible: pass --sync-data-t and/or --sync-audio-t"
        else:
            off = sync_audio_t - sync_data_t
            a["offset_s"] = off
            # the reference: A0, or the pauses
            ref = None
            spectra = {}
            seg_ref = next((s for s in segs if s[0] == "A0"), None)
            if seg_ref is not None:
                ref_pieces = [(seg_ref[3], seg_ref[4])]
                a["reference"] = "A0"
            else:
                F, c = S["F"], S["c"]
                ref_pieces = []
                for k in range(len(S["steps"])):
                    m = (F[:, c["step"]] == k) & (F[:, c["sub"]] == SUB_PAUSE)
                    if m.sum() > 10:
                        ref_pieces.append((F[m, c["t"]][0], F[m, c["t"]][-1]))
                a["reference"] = "pauses (A0 not run)"
            chunks = [x[max(0, int((p0 + off) * fs)):max(0, int((p1 + off) * fs))]
                      for p0, p1 in ref_pieces]
            chunks = [ch for ch in chunks if len(ch) > 2048]
            if chunks:
                ff, ref = spectrum(np.concatenate(chunks), fs)
            lines = {}
            for lab, k, j, t0, t1 in segs:
                i0, i1 = int((t0 + off) * fs), int((t1 + off) * fs)
                if i0 < 0 or i1 > len(x) or i1 - i0 < 2048:
                    lines[lab] = {"error": "outside the recording"}
                    continue
                f, P = spectrum(x[i0:i1], fs)
                spectra[lab] = (f, P)
                lines[lab] = {"rms_dBFS": float(10 * np.log10(np.mean(x[i0:i1] ** 2) + 1e-20)),
                              "lines": dominant_lines(f, P, ref)}
            a["segments"] = lines
            p = spectrogram_figure(x, fs, segs, off, out)
            if p:
                figs.append(p)
            p = spectra_figure({k: v for k, v in spectra.items() if "." not in k}, out)
            if p:
                figs.append(p)
        res["audio"] = a
    res["figures"] = [os.path.basename(p) for p in figs]
    with open(os.path.join(out, "summary.json"), "w") as fh:
        json.dump(res, fh, indent=1)
    with open(os.path.join(out, "summary.md"), "w") as fh:
        fh.write(markdown(res))
    return res, out


def markdown(res):
    L = ["# Free-cam bench — {}".format(res["session"]), ""]
    m = res["meta"]
    L.append("Drive id {}, session start (Pi clock) {}, command rate {} Hz. Stop: {}. "
             "Frames sent: {}. Foreign command frames: {}.".format(
                 m.get("cid"), m.get("t0_wall"), fmt(m.get("rate_hz"), 0),
                 m.get("stop_reason") or "none", m.get("tx"), m.get("foreign_cmd_frames")))
    L += ["", "## Steps", "",
          "| segment | rx Hz | I_mean | I_ripple_A | v_flip | I_max | temp_max | v rad/s | "
          "v_pos | outcome |",
          "|---|---|---|---|---|---|---|---|---|---|"]
    for r in res["segments"]:
        L.append("| {} | {} | {} | {} | {} | {} | {} | {} | {} | {} |".format(
            r["segment"], fmt(r["rx_hz"], 1), fmt(r["I_mean"]), fmt(r["I_ripple_A"]),
            fmt(r["v_flip"]), fmt(r["I_max"]), fmt(r["temp_max"], 0), fmt(r["v_erpm_rads"]),
            fmt(r["v_pos_rads"]), r["outcome"]))
    det = [(s["id"], s.get("detail")) for s in res["steps"]
           if s["outcome"] == "runaway" or s["outcome"].startswith(("abort", "refused"))]
    if det:
        L += ["", "Stops:", ""]
        for sid, d in det:
            L.append("- {}: {}".format(sid, json.dumps(d)))
    a = res["audio"]
    if a:
        L += ["", "## Audio", "",
              "File {} ({:.0f} s, {} Hz).".format(a["file"], a["seconds"], a["fs"]),
              "Tap in the data: {} s ({}). Tap in the audio: {} s ({}).".format(
                  fmt(a.get("data_tap_t"), 3), a.get("data_tap_how", "forced"),
                  fmt(a.get("audio_tap_t"), 3), a.get("audio_tap_how", "forced")),
              "Audio onset candidates (t s, strength): {}. If the pick is wrong, rerun with "
              "`--sync-audio-t <t>`.".format(
                  ", ".join("{:.3f} ({:.0f}x)".format(t, s)
                            for t, s in a.get("audio_onset_candidates", [])) or "none")]
        if a.get("error"):
            L += ["", "**{}**".format(a["error"])]
        else:
            L += ["Offset audio − session: {:.3f} s. Reference spectrum: {}.".format(
                a["offset_s"], a["reference"]), "",
                "Dominant lines, {:.0f} Hz – {:.0f} kHz (frequency Hz / dB above the reference):"
                .format(F_LO, F_HI / 1000), "",
                "| segment | rms dBFS | lines |", "|---|---|---|"]
            for lab, v in a["segments"].items():
                if "error" in v:
                    L.append("| {} | — | {} |".format(lab, v["error"]))
                    continue
                txt = ", ".join("{:.0f} / {:+.1f}".format(q["f_Hz"], q["dB_above_A0"])
                                for q in v["lines"]) or "none"
                L.append("| {} | {:.1f} | {} |".format(lab, v["rms_dBFS"], txt))
    L += ["", "## Figures", ""] + ["- `{}`".format(f) for f in res["figures"]]
    L += ["", "Reading the results: plan §6 (docs/sim2real/DASH01_plan_banc_came_2026-11-25.md)."]
    return "\n".join(L) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session")
    ap.add_argument("--audio", default=None)
    ap.add_argument("--out", default=None)
    ap.add_argument("--sync-audio-t", type=float, default=None)
    ap.add_argument("--sync-data-t", type=float, default=None)
    ap.add_argument("--sync-search", type=float, default=180.0)
    a = ap.parse_args(argv)
    res, out = analyse(a.session, a.audio, a.out, a.sync_audio_t, a.sync_data_t, a.sync_search)
    print(open(os.path.join(out, "summary.md")).read())
    print("written to {}".format(out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
