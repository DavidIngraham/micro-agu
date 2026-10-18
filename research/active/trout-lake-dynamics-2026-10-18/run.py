"""Frozen-candidate local dynamics validation; no fitting or trajectory-drift veto."""

import os

os.environ["OPENBLAS_NUM_THREADS"] = "1"
import sys, json, hashlib
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from scipy.signal import savgol_filter, welch, csd
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
PRIOR = ROOT / "research/active/trout-lake-fidelity-2026-10-18"
sys.path.insert(0, str(PRIOR))
import run as prior
from microagu_sim.compat.profiles import make_simulator

NAMES = [
    "roll_rate",
    "pitch_rate",
    "yaw_rate",
    "roll_acceleration",
    "pitch_acceleration",
    "yaw_acceleration",
    "forward_velocity",
    "climb",
]
STARTS = [t + d for t in [1440, 2100, 2760] for d in range(0, 90, 10)]


def save(name, value):
    (OUT / name).write_text(json.dumps(value, indent=2) + "\n")


def evaluate(job):
    start, family, theta, alpha = job
    a, b, i, _ = prior.load("trout_lake", start)
    sl = slice(i, i + 151)
    sim = make_simulator("trout_lake", profile="as_built", substeps=32)
    sim.parameters["left_offset"] += theta[0] / 1000
    sim.parameters["right_offset"] += theta[1] / 1000
    sim.parameters["brake_cp_shift"] = theta[2]
    sim.parameters["Cndelta"] = theta[3]
    for key, val in zip(
        ["joint_k_roll", "joint_c_pitch", "joint_c_yaw", "CLbrake", "CDbrake"],
        theta[4:],
    ):
        sim.parameters[key] *= val
    y0 = prior.historical.init(sim, a, b, i, alpha)
    u = np.column_stack([a["U"][sl, 0], a["brake_mm"][sl]])
    y, _ = sim.replay(
        y0, u, voltage=float(b["voltage"][i]), rpm=float(b["rpm"][i]), wind=a["wind"][i]
    )
    assert np.isfinite(y).all()
    rp = Rotation.from_quat(y[:, 3:7])
    rates = rp.inv().apply(y[:, 10:13])
    nav = y[:, 7:10] + np.cross(y[:, 10:13], rp.apply(sim.geometry["imu"]))
    obs = a["Y"][sl]
    ro = Rotation.from_euler("xyz", obs[:, :3])

    def channels(r, v, rp):
        r = np.rad2deg(r)
        acc = savgol_filter(r, 11, 3, deriv=1, delta=0.04, axis=0)
        return np.column_stack([r, acc, rp.inv().apply(v)[:, 0], -v[:, 2]])

    actual = channels(obs[:, 3:6], obs[:, 6:9], ro)
    model = channels(rates, nav, rp)
    # 2 s to settle hidden initialization, 0.24 s edge for derivative support.
    actual = actual[50:-6]
    model = model[50:-6]
    err = model - actual
    bias = err.mean(axis=0)
    centered = np.sqrt(np.mean((err - bias) ** 2, axis=0))
    raw = np.sqrt(np.mean(err**2, axis=0))
    return dict(
        start=start,
        family=family,
        alpha=alpha,
        centered_rmse=centered.tolist(),
        raw_rmse=raw.tolist(),
        bias=bias.tolist(),
    )


def spectral():
    # Same recorded input in flight and both replays. This is a CLOSED-LOOP descriptive comparison.
    starts = [t + d for t in [1440, 2100, 2760] for d in [0, 30, 60]]
    spectra = {}
    transfer = {}
    quiet = []
    for family in ["flight", "baseline", "physics"]:
        ps = []
        cross = []
        inputs = []
        for start in starts:
            d = np.load(
                PRIOR
                / f"{'baseline' if family == 'flight' else family}-trout_lake-{start}-0.35.npz"
            )
            rates = np.rad2deg(
                d["observed"][:, 3:6] if family == "flight" else d["model_rates"]
            )
            velocity = (
                d["observed"][:, 6:9] if family == "flight" else d["model_velocity"]
            )
            output = np.column_stack([rates, -velocity[:, 2]])[50:-50]
            u = d["commands"]
            u = np.column_stack([u[:, 2] - u[:, 1], u[:, 1:].mean(axis=1), u[:, 0]])[
                50:-50
            ]
            f, px = welch(output, fs=25, nperseg=256, noverlap=128, axis=0)
            _, pu = welch(u, fs=25, nperseg=256, noverlap=128, axis=0)
            xy = np.empty((len(f), 3, 4), complex)
            for j in range(3):
                for k in range(4):
                    _, xy[:, j, k] = csd(
                        u[:, j], output[:, k], fs=25, nperseg=256, noverlap=128
                    )
            ps.append(px)
            inputs.append(pu)
            cross.append(xy)
            if family == "flight":
                for i in range(0, len(u) - 75, 25):
                    q = u[i : i + 75]
                    if (
                        np.ptp(q[:, 0]) < 2
                        and np.ptp(q[:, 1]) < 2
                        and np.ptp(q[:, 2]) < 0.02
                    ):
                        quiet.append(dict(start=start, offset_s=2 + i * 0.04))
        spectra[family] = np.mean(ps, axis=0)
        pxu = np.mean(cross, axis=0)
        pu = np.mean(inputs, axis=0)
        transfer[family] = (
            pxu / np.maximum(pu[:, :, None], 1e-12),
            np.abs(pxu) ** 2
            / np.maximum(pu[:, :, None] * spectra[family][:, None, :], 1e-12),
        )
    band = (f >= 0.2) & (f <= 3.0)
    result = {}
    hobs, coh = transfer["flight"]
    valid = (f[:, None, None] >= 0.2) & (f[:, None, None] <= 2.0) & (coh >= 0.5)
    for family in ["baseline", "physics"]:
        db = 10 * np.log10(
            np.maximum(spectra[family], 1e-12) / np.maximum(spectra["flight"], 1e-12)
        )
        h, _ = transfer[family]
        gain = 20 * np.log10(np.maximum(abs(h), 1e-12) / np.maximum(abs(hobs), 1e-12))
        phase = np.rad2deg(np.angle(h * np.conj(hobs)))
        result[family] = dict(
            rate_psd_log_rmse_db=np.sqrt(np.mean(db[band, :3] ** 2, axis=0)).tolist(),
            coherent_gain_rmse_db=float(np.sqrt(np.mean(gain[valid] ** 2)))
            if valid.any()
            else None,
            coherent_phase_rmse_deg=float(np.sqrt(np.mean(phase[valid] ** 2)))
            if valid.any()
            else None,
            peak_rate_frequency_hz=[
                float(f[band][np.argmax(spectra[family][band, k])]) for k in range(3)
            ],
        )
    result["observed_peak_rate_frequency_hz"] = [
        float(f[band][np.argmax(spectra["flight"][band, k])]) for k in range(3)
    ]
    result["coherent_bins"] = int(valid.sum())
    result["quiet_3s_intervals"] = quiet
    np.savez_compressed(OUT / "spectra.npz", frequency=f, **spectra)
    save("spectral.json", result)
    return result


def main():
    save(
        "protocol.json",
        dict(
            target="Trout Lake 1.758 kg, historical 132 mm geometry",
            fit="None; previous candidate frozen",
            validation_starts=STARTS,
            duration_s=6,
            discard_initial_s=2,
            discard_final_s=0.24,
            initial_angles=[0.25, 0.35, 0.45],
            channels=NAMES,
            primary="Demeaned per-window error: rates, smoothed angular accelerations, body-forward ground velocity and climb. Raw errors and bias reported separately.",
            gates=dict(
                mean_channel_ratio=0.95,
                worst_channel_ratio=1.05,
                mean_rate_spectral_error_ratio=1.05,
                coherent_gain_error_ratio=1.05,
                coherent_phase_error_increase_deg=5,
                minimum_coherent_bins=12,
            ),
            spectral="Pooled 30 s reserved-window PSD 0.2–3 Hz; input/output descriptive gain and phase 0.2–2 Hz where observed coherence >=0.5. Not an open-loop system identification.",
            limitations="Same previously examined flight. No fitting on validation. Quiet input does not establish unforced decay; damping identification may remain unsupported. No course or horizontal earth-frame velocity loss. No automatic deployment.",
        ),
    )
    candidate = json.loads((PRIOR / "selection.json").read_text())["primary"]["theta"]
    save("candidate.json", candidate)
    with ProcessPoolExecutor(max_workers=4) as pool:
        rows = list(
            pool.map(
                evaluate,
                [
                    (s, f, t, a)
                    for f, t in [("baseline", prior.DEFAULT), ("physics", candidate)]
                    for a in [0.25, 0.35, 0.45]
                    for s in STARTS
                ],
            )
        )
    save("local.json", rows)
    summary = {}
    for key in ["centered_rmse", "raw_rmse"]:
        base = np.mean([r[key] for r in rows if r["family"] == "baseline"], axis=0)
        new = np.mean([r[key] for r in rows if r["family"] == "physics"], axis=0)
        summary[key] = dict(
            baseline=base.tolist(), candidate=new.tolist(), ratios=(new / base).tolist()
        )
    summary["mean_absolute_window_bias"] = {
        f: np.mean(
            np.abs([r["bias"] for r in rows if r["family"] == f]), axis=0
        ).tolist()
        for f in ["baseline", "physics"]
    }
    spec = spectral()
    r = summary["centered_rmse"]["ratios"]
    psd_ratio = float(
        np.mean(spec["physics"]["rate_psd_log_rmse_db"])
        / np.mean(spec["baseline"]["rate_psd_log_rmse_db"])
    )
    checks = dict(
        local_mean=np.mean(r) <= 0.95,
        local_worst=max(r) <= 1.05,
        spectral=psd_ratio <= 1.05,
        coherence=spec["coherent_bins"] >= 12,
        gain=spec["physics"]["coherent_gain_rmse_db"]
        <= 1.05 * spec["baseline"]["coherent_gain_rmse_db"]
        if spec["coherent_bins"]
        else False,
        phase=spec["physics"]["coherent_phase_rmse_deg"]
        <= spec["baseline"]["coherent_phase_rmse_deg"] + 5
        if spec["coherent_bins"]
        else False,
    )
    summary["checks"] = {k: bool(v) for k, v in checks.items()}
    summary["passed"] = all(checks.values())
    summary["spectral_error_ratio"] = psd_ratio
    save("summary.json", summary)
    print(json.dumps(summary, indent=2), flush=True)
    save(
        "source-hashes.json",
        {
            str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in [
                OUT / "run.py",
                PRIOR / "selection.json",
                ROOT / "simulation/crates/sim-core/src/dynamics.rs",
            ]
        },
    )


if __name__ == "__main__":
    main()
