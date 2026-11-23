#!/usr/bin/env python3
"""Analyse the run logs of the Kd validation test (frozen-step bundles, kdval_*.npz).

    python tools/kd_validation/analyze_runs.py <policyrun_*.npz | .csv> ... [--out DIR] [--labels ...]

Input: the run logs saved by `policy_cli.py run` (data/policyruns/policyrun_*.npz on the Pi, 65
columns, every six-vector in MODEL order hip_roll_L, cam_L, thigh_L, hip_roll_R, cam_R, thigh_R).
A CSV with the same header (the panel's download) works too. Any policy run log works, so the old
runs (run 5, runs 6-9) can be put next to the new ones as a reference.

Window: the ticks that carry the run's full frame -- after the start ramp, before the end -- i.e.
the ticks whose kp and kd equal their median over the run (frozen bundles), or every tick from
0.5 s on for a real policy run (its gains move on every tick).

Per joint, in the window (the 100 Hz log only sees up to 50 Hz; the whistle is above, so these
numbers are its folded trace, read them next to the audio notes):

  kp_cmd, kd_cmd      what was asked
  err_deg             mean of target - pos (the static deflection)
  I_ripple_A          std of (current - its 90 ms moving mean)     [ANALYSE_bruit_moteurs.md §2]
  dI_rms_A            rms of the tick-to-tick current change          [run 5 metric]
  v_noise_dps         std of (velocity - its 90 ms moving mean), deg/s
  v_flip              fraction of ticks where the reported velocity changes sign
  f_peak_Hz, hf_frac  peak of the current spectrum (Welch) and share of its power above 20 Hz
  kp_eff, kd_eff, R2  least squares  tau = kp_eff (target - pos) - kd_eff vel + c  (needs motion:
                      push the robot gently through the harness during the run, see the plan)
  law_R2              how much of the torque the COMMANDED law kp e - kd v explains
  kp_static           mean torque / mean error (only if |error| > 0.3 deg; biased by any zero
                      offset of the joint, so a cross-check of kp_eff, not a measurement)
  fight               cam/thigh of the same leg: share of ticks with opposite-sign power (90 ms)

Writes <out>/summary.md (one table per joint group, runs side by side) and <out>/summary.json, and,
if matplotlib is there, <out>/<run>_leg.png (current and velocity of the four leg joints) and
<out>/ripple_vs_kd.png.
"""
import argparse
import json
import os
import sys

import numpy as np

J = ["hip_roll_L", "cam_L", "thigh_L", "hip_roll_R", "cam_R", "thigh_R"]
LEG = [1, 2, 4, 5]
DT = 0.01
MA = 9                      # 90 ms moving mean, as in ANALYSE_bruit_moteurs.md
COLS = {"t": 0, "pos": 1, "vel": 7, "tau": 13, "amps": 19, "temp": 25, "grav": 31, "gyro": 34,
        "target": 37, "kp": 43, "kd": 49}


def load(path):
    if path.endswith(".npz"):
        z = np.load(path, allow_pickle=False)
        data = z["data"].astype(float)
        meta = json.loads(str(z["meta_json"])) if "meta_json" in z.files else {}
    else:
        data = np.genfromtxt(path, delimiter=",", skip_header=1)
        meta = {}
    if data.ndim != 2 or data.shape[1] < 55:
        raise ValueError("{}: not a 65-column policy run log".format(path))
    out = {k: (data[:, i:i + 6] if k not in ("t",) else data[:, 0]) for k, i in COLS.items()}
    out["grav"], out["gyro"] = data[:, 31:34], data[:, 34:37]
    out["meta"] = meta
    return out


def window(r):
    kp, kd = r["kp"], r["kd"]
    kpm, kdm = np.median(kp, 0), np.median(kd, 0)
    frozen = np.isclose(kp, kpm, rtol=1e-3).all(1) & np.isclose(kd, kdm, rtol=1e-3, atol=1e-4).all(1)
    if frozen.mean() > 0.5:
        return frozen, True
    return r["t"] - r["t"][0] >= 0.5, False


def movmean(x, n=MA):
    k = np.ones(n) / n
    pad = np.pad(x, ((n // 2, n - 1 - n // 2), (0, 0)), mode="edge")
    return np.stack([np.convolve(pad[:, j], k, mode="valid") for j in range(x.shape[1])], 1)


def welch(x, nper=64):
    """Mean periodogram over 50 %-overlapping Hann segments; x 1-D, fs 100 Hz."""
    x = np.asarray(x, float)
    if x.size < nper:
        nper = max(8, x.size)
    w = np.hanning(nper)
    segs = [x[i:i + nper] for i in range(0, x.size - nper + 1, nper // 2)]
    P = np.mean([np.abs(np.fft.rfft((s - s.mean()) * w)) ** 2 for s in segs], 0)
    return np.fft.rfftfreq(nper, DT), P


def fit_law(e, v, tau):
    A = np.column_stack([e, -v, np.ones_like(e)])
    if np.std(e) < 1e-4 or np.std(v) < 1e-3:
        return None, None, None
    coef, *_ = np.linalg.lstsq(A, tau, rcond=None)
    res = tau - A @ coef
    r2 = 1.0 - res.var() / max(tau.var(), 1e-12)
    return float(coef[0]), float(coef[1]), float(r2)


def analyse(path, label=None):
    r = load(path)
    w, frozen = window(r)
    if w.sum() < 2 * MA:
        raise ValueError("{}: only {} usable ticks".format(path, int(w.sum())))
    pos, vel, tau, amps = (r[k][w] for k in ("pos", "vel", "tau", "amps"))
    tgt, kp, kd = r["target"][w], r["kp"][w], r["kd"][w]
    e = tgt - pos
    ripple = amps - movmean(amps)
    vnoise = vel - movmean(vel)
    joints = {}
    for j, n in enumerate(J):
        f, P = welch(ripple[:, j])
        hf = float(P[f > 20].sum() / max(P[f > 0].sum(), 1e-12))
        kp_e, kd_e, r2 = fit_law(e[:, j], vel[:, j], tau[:, j])
        law = kp[:, j] * e[:, j] - kd[:, j] * vel[:, j]
        law_r2 = 1.0 - np.var(tau[:, j] - law) / max(np.var(tau[:, j]), 1e-12)
        joints[n] = {
            "kp_cmd": float(np.median(kp[:, j])), "kd_cmd": float(np.median(kd[:, j])),
            "err_deg": float(np.degrees(e[:, j].mean())), "tau_mean": float(tau[:, j].mean()),
            "kp_static": (float(tau[:, j].mean() / e[:, j].mean())
                          if abs(e[:, j].mean()) > np.radians(0.3) else None),
            "I_ripple_A": float(ripple[:, j].std()),
            "dI_rms_A": float(np.sqrt(np.mean(np.diff(amps[:, j]) ** 2))),
            "v_noise_dps": float(np.degrees(vnoise[:, j].std())),
            "v_flip": float(np.mean(np.sign(vel[1:, j]) * np.sign(vel[:-1, j]) < 0)),
            "f_peak_Hz": float(f[1:][np.argmax(P[1:])]), "hf_frac": hf,
            "kp_eff": kp_e, "kd_eff": kd_e, "R2": r2, "law_R2": float(law_r2),
            "temp_max": float(r["temp"][w][:, j].max()),
        }
    P = movmean(tau * vel)
    fight = {s: float(np.mean(np.sign(P[:, c]) * np.sign(P[:, t]) < 0))
             for s, c, t in (("L", 1, 2), ("R", 4, 5))}
    g = r["grav"][w]
    pitch = np.degrees(np.arctan2(g[:, 0], -g[:, 2]))
    m = r["meta"]
    name = label or m.get("file") or os.path.basename(path)
    return {"label": name, "path": os.path.basename(path), "frozen": frozen,
            "ticks": int(w.sum()), "seconds": float(w.sum() * DT), "exit": m.get("exit_reason"),
            "realised_hz": m.get("realised_hz"), "late_ticks": m.get("late_ticks"),
            "pitch_mean": float(pitch.mean()), "pitch_span": float(pitch.max() - pitch.min()),
            "fight": fight, "joints": joints,
            "_series": {"t": r["t"][w] - r["t"][w][0], "amps": amps, "vel": vel, "ripple": ripple}}


def fmt(v, nd=2):
    if v is None:
        return "—"
    if isinstance(v, float):
        return ("{:.%df}" % nd).format(v)
    return str(v)


def table(runs, keys, joints):
    lines = []
    for n in joints:
        lines.append("\n**{}**\n".format(n))
        lines.append("| run | " + " | ".join(keys) + " |")
        lines.append("|---|" + "---|" * len(keys))
        for r in runs:
            q = r["joints"][n]
            lines.append("| {} | ".format(r["label"]) + " | ".join(fmt(q[k]) for k in keys) + " |")
    return "\n".join(lines)


def plots(runs, out):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return []
    made = []
    for r in runs:
        s = r["_series"]
        fig, ax = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
        for j in LEG:
            ax[0].plot(s["t"], s["amps"][:, j], lw=0.8, label=J[j])
            ax[1].plot(s["t"], np.degrees(s["vel"][:, j]), lw=0.8, label=J[j])
        ax[0].set_ylabel("current (A)")
        ax[1].set_ylabel("velocity (deg/s)")
        ax[1].set_xlabel("t in window (s)")
        ax[0].legend(ncol=4, fontsize=8)
        ax[0].set_title(r["label"])
        p = os.path.join(out, "{}_leg.png".format(os.path.splitext(r["path"])[0]))
        fig.tight_layout()
        fig.savefig(p, dpi=110)
        plt.close(fig)
        made.append(p)
    fig, ax = plt.subplots(1, 2, figsize=(10, 4))
    for j in LEG:
        kd = [r["joints"][J[j]]["kd_cmd"] for r in runs]
        ax[0].plot(kd, [r["joints"][J[j]]["I_ripple_A"] for r in runs], "o", label=J[j])
        ax[1].plot(kd, [r["joints"][J[j]]["v_noise_dps"] for r in runs], "o", label=J[j])
    ax[0].set_xlabel("kd commanded")
    ax[0].set_ylabel("current ripple (A, std)")
    ax[1].set_xlabel("kd commanded")
    ax[1].set_ylabel("velocity noise (deg/s, std)")
    ax[0].legend(fontsize=8)
    fig.tight_layout()
    p = os.path.join(out, "ripple_vs_kd.png")
    fig.savefig(p, dpi=110)
    plt.close(fig)
    return made + [p]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--labels", nargs="*", default=None)
    ap.add_argument("--out", default=".")
    a = ap.parse_args()
    labels = a.labels or [None] * len(a.logs)
    if len(labels) != len(a.logs):
        sys.exit("--labels needs one label per log")
    runs = [analyse(p, l) for p, l in zip(a.logs, labels)]
    os.makedirs(a.out, exist_ok=True)
    md = ["# Kd validation — run analysis", "",
          "Window: ticks with the full frozen frame (or from 0.5 s for a policy run). 100 Hz log: "
          "only the folded trace of the whistle is visible here.", "",
          "| run | frozen | s | Hz | late | pitch mean / span (deg) | fight L / R | exit |",
          "|---|---|---|---|---|---|---|---|"]
    for r in runs:
        md.append("| {} | {} | {:.1f} | {} | {} | {:+.1f} / {:.1f} | {:.0%} / {:.0%} | {} |".format(
            r["label"], "yes" if r["frozen"] else "no", r["seconds"], fmt(r["realised_hz"], 0),
            fmt(r["late_ticks"]), r["pitch_mean"], r["pitch_span"], r["fight"]["L"],
            r["fight"]["R"], r["exit"] or ""))
    md.append("\n## Noise (leg joints)")
    md.append(table(runs, ["kp_cmd", "kd_cmd", "I_ripple_A", "dI_rms_A", "v_noise_dps", "v_flip",
                           "f_peak_Hz", "hf_frac", "temp_max"], [J[j] for j in LEG]))
    md.append("\n## Stiffness and damping actually obtained (leg joints)")
    md.append(table(runs, ["kp_cmd", "kp_eff", "kp_static", "kd_cmd", "kd_eff", "R2", "law_R2",
                           "err_deg", "tau_mean"], [J[j] for j in LEG]))
    md.append("\n## Hip roll")
    md.append(table(runs, ["kp_cmd", "kd_cmd", "I_ripple_A", "v_noise_dps", "err_deg"],
                    [J[0], J[3]]))
    figs = plots(runs, a.out)
    if figs:
        md.append("\n## Figures\n")
        md += ["![]({})".format(os.path.basename(p)) for p in figs]
    with open(os.path.join(a.out, "summary.md"), "w") as f:
        f.write("\n".join(md) + "\n")
    with open(os.path.join(a.out, "summary.json"), "w") as f:
        json.dump([{k: v for k, v in r.items() if k != "_series"} for r in runs], f, indent=1)
    print("\n".join(md))


if __name__ == "__main__":
    main()
