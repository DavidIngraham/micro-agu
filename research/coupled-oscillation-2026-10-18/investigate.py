"""Diagnose gyro/attitude/command consistency before attributing a coupled mode."""

import os, json, hashlib, sys
from pathlib import Path
import numpy as np
from scipy.signal import welch, csd, resample_poly, butter, sosfiltfilt
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "research/trout-lake-fidelity-2026-10-18"))
import run as prior

DATA = prior.historical.DATA.parent / "2026-02-15 15-38-50"


def save(name, value):
    (OUT / name).write_text(json.dumps(value, indent=2) + "\n")


def read(kind):
    rows = json.loads((DATA / f"{kind}.json").read_text())
    if kind == "IMU":
        rows = [r for r in rows if r["I"] == 0]
    if kind == "VIBE":
        rows = [r for r in rows if r["IMU"] == 0]
    t = np.array([r["TimeUS"] / 1e6 for r in rows])
    return t, {
        k: np.array([r[k] for r in rows]) for k in rows[0] if k not in ["mavpackettype"]
    }


def interp(t, x, tq):
    return np.column_stack([np.interp(tq, t, x[:, k]) for k in range(x.shape[1])])


def psd(x, fs):
    return welch(x, fs=fs, nperseg=round(fs * 10.24), noverlap=round(fs * 5.12), axis=0)


def band(f, p, lo=1.2, hi=1.9):
    return np.sqrt(np.sum(p[(f >= lo) & (f <= hi)], axis=0) * (f[1] - f[0]))


def main():
    ti, imu = read("IMU")
    ta, att = read("ATT")
    tm, mode = read("MODE")
    tr, rc = read("RCOU")
    tv, vibe = read("VIBE")
    gyro = np.column_stack([imu[k] for k in ["GyrX", "GyrY", "GyrZ"]])
    angles = np.deg2rad(np.column_stack([att[k] for k in ["Roll", "Pitch", "Yaw"]]))
    rot = Rotation.from_euler("xyz", angles)
    omega = (rot[:-1].inv() * rot[1:]).as_rotvec() / np.diff(ta)[:, None]
    tw = (ta[:-1] + ta[1:]) / 2
    rcvalues = np.column_stack([rc["C1"], rc["C3"], rc["C4"]])
    a, _, _, _ = prior.load("trout_lake", 1440)
    datasets = {}
    summary = []
    for start in [1440, 2100, 2760]:
        t50 = np.arange(start, start + 90, 0.02)
        t25 = np.arange(start, start + 90, 0.04)
        g50 = np.rad2deg(interp(ti, gyro, t50))
        g25 = resample_poly(g50, 1, 2, axis=0)
        derived = np.rad2deg(interp(tw, omega, t25))
        causal = (
            np.column_stack([np.interp(t25, a["t"], a["Y"][:, k]) for k in [3, 4, 5]])
            * 180
            / np.pi
        )
        spectra = {}
        for label, x, fs in [
            ("raw_50hz", g50, 50),
            ("antialias_25hz", g25, 25),
            ("attitude_derived", derived, 25),
            ("causal_25hz", causal, 25),
        ]:
            f, p = psd(x, fs)
            spectra[label] = {"frequency": f.tolist(), "psd": p.tolist()}
        f, p = psd(g25, 25)
        _, pa = psd(derived, 25)
        cross = np.column_stack(
            [
                csd(g25[:, k], derived[:, k], fs=25, nperseg=256, noverlap=128)[1]
                for k in range(3)
            ]
        )
        coh = abs(cross) ** 2 / np.maximum(p * pa, 1e-12)
        i = np.argmin(abs(f - 1.5625))
        row = {
            "start": start,
            "band_rms_deg_s": {
                label: band(np.array(v["frequency"]), np.array(v["psd"])).tolist()
                for label, v in spectra.items()
            },
            "gyro_attitude_coherence_at_1_56": coh[i].tolist(),
            "gyro_attitude_phase_deg_at_1_56": np.rad2deg(np.angle(cross[i])).tolist(),
        }
        _, p50 = psd(g50, 50)
        f50, _ = psd(g50, 50)
        row["raw50_yaw_peak_hz"] = float(f50[(f50 > 0.2)][np.argmax(p50[f50 > 0.2, 2])])
        summary.append(row)
        datasets[str(start)] = spectra
        if start == 1440:
            np.savez_compressed(
                OUT / "example.npz",
                time=t25 - start,
                gyro=g25,
                attitude_derived=derived,
                causal=causal,
            )
    save("sampling-comparison.json", summary)
    save("spectra.json", datasets)
    # Survey separated 30 s windows across the airborne portion. Do not fit any model.
    survey = []
    for start in range(300, 3301, 30):
        t = np.arange(start, start + 30, 0.04)
        if t[-1] > a["t"][-1]:
            continue
        ix = np.searchsorted(a["t"], t)
        ix = np.clip(ix, 0, len(a["t"]) - 1)
        if not a["valid"][ix].all():
            continue
        g = np.rad2deg(interp(ti, gyro, t))
        at = np.rad2deg(interp(tw, omega, t))
        u = interp(tr, rcvalues, t)
        modes = mode["ModeNum"][np.maximum(0, np.searchsorted(tm, t, side="right") - 1)]
        f, pg = psd(g, 25)
        _, pat = psd(at, 25)
        diff = (2400 - u[:, 2] - (u[:, 1] - 800)) * np.pi * 55 / 360 * 0.1
        _, pd = psd(diff, 25)
        _, c = csd(diff, g[:, 2], fs=25, nperseg=256, noverlap=128)
        i = (
            np.argmax(pg[(f >= 1.2) & (f <= 1.9), 2])
            + np.flatnonzero((f >= 1.2) & (f <= 1.9))[0]
        )
        survey.append(
            dict(
                start=start,
                mode_counts={int(k): int(sum(modes == k)) for k in np.unique(modes)},
                throttle=float(np.mean((u[:, 0] - 1000) / 1000)),
                gyro_band_rms=band(f, pg).tolist(),
                attitude_band_rms=band(f, pat).tolist(),
                peak_hz=float(f[i]),
                diff_yaw_coherence=float(abs(c[i]) ** 2 / max(pd[i] * pg[i, 2], 1e-12)),
                diff_yaw_phase_deg=float(np.angle(c[i]) * 180 / np.pi),
            )
        )
    save("survey.json", survey)
    save(
        "source-audit.json",
        {
            "inputs": {
                k: hashlib.sha256((DATA / f"{k}.json").read_bytes()).hexdigest()
                for k in ["IMU", "ATT", "RCOU", "MODE", "VIBE"]
            },
            "median_dt_s": {
                "IMU": float(np.median(np.diff(ti))),
                "ATT": float(np.median(np.diff(ta))),
                "RCOU": float(np.median(np.diff(tr))),
            },
            "rate_source": "IMU instance 0 GyrX/Y/Z; original causal replay uses latest-sample hold; attitude rate independently derived from successive ATT orientations, shares underlying sensors and EKF.",
        },
    )
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
