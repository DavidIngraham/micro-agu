"""Manual-flight coupling fit with amplitude/phase and local-error validation."""

import os

os.environ["OPENBLAS_NUM_THREADS"] = "1"
import sys, json, hashlib
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from scipy.signal import welch, csd, savgol_filter
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
import importlib.util

spec = importlib.util.spec_from_file_location(
    "trout_fidelity_prior", ROOT / "research/active/trout-lake-fidelity-2026-10-18/run.py"
)
prior = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prior)
from microagu_sim.compat.profiles import make_simulator
from microagu_sim.compat.dynamics import causal_replay

TRAIN = [280, 310, 535, 555, 1988]
MANUAL_CHECK = [380, 400, 440]
AUTO_CHECK = [1440, 1470, 1500, 2130, 2160, 2190, 2760, 2790, 2820]
DEFAULT = [1.0, 1.0, 1.0, 1.0, 1.0, 0.0]
BOUNDS = [(0.5, 1.5), (0.5, 1.5), (0.2, 1.5), (0.2, 1.5), (0.85, 1.15), (0.0, 0.12)]
NAMES = [
    "rotor_inertia_scale",
    "yaw_stiffness_scale",
    "yaw_damping_scale",
    "pitch_damping_scale",
    "effective_pull_scale",
    "actuator_lag_s",
]
CACHE = {}


def save(name, v):
    (OUT / name).write_text(json.dumps(v, indent=2) + "\n")


def rates(y):
    return Rotation.from_quat(y[:, 3:7]).inv().apply(y[:, 10:13]) * 180 / np.pi


def metrics(obs, pred):
    actual = obs[50:-10]
    model = pred[50:-10]
    e = model - actual
    e -= e.mean(axis=0)
    rmse = np.sqrt(np.mean(e**2, axis=0))
    scale = np.maximum(np.std(actual, axis=0), 1.0)
    da = savgol_filter(actual, 11, 3, deriv=1, delta=0.04, axis=0)[6:-6]
    dm = savgol_filter(model, 11, 3, deriv=1, delta=0.04, axis=0)[6:-6]
    ae = dm - da
    ae -= ae.mean(axis=0)
    acc = np.sqrt(np.mean(ae**2, axis=0))
    ascale = np.maximum(np.std(da, axis=0), 5.0)
    f, po = welch(actual, fs=25, nperseg=256, noverlap=128, axis=0)
    _, pm = welch(model, fs=25, nperseg=256, noverlap=128, axis=0)
    mask = (f >= 1.2) & (f <= 1.9)
    df = f[1] - f[0]
    ao = np.sqrt(po[mask].sum(axis=0) * df)
    am = np.sqrt(pm[mask].sum(axis=0) * df)
    peak = np.flatnonzero(mask)[np.argmax(po[mask, 2])]
    _, co = csd(actual[:, 2], actual[:, 1], fs=25, nperseg=256, noverlap=128)
    _, cm = csd(model[:, 2], model[:, 1], fs=25, nperseg=256, noverlap=128)
    phase_error = float(np.angle(cm[peak] * np.conj(co[peak])))
    coherence = float(abs(co[peak]) ** 2 / max(po[peak, 1] * po[peak, 2], 1e-12))
    local = float(np.mean(np.r_[(rmse / scale) ** 2, (acc / ascale) ** 2]))
    amplitude = float(np.mean(np.log(np.maximum(am, 1e-3) / np.maximum(ao, 1e-3)) ** 2))
    phase = float(1 - np.cos(phase_error)) if coherence >= 0.5 else 0.0
    return dict(
        rate_rmse=rmse.tolist(),
        acceleration_rmse=acc.tolist(),
        mean_rate_bias=(model - actual).mean(axis=0).tolist(),
        observed_band_rms=ao.tolist(),
        model_band_rms=am.tolist(),
        peak_hz=float(f[peak]),
        observed_qr_phase_deg=float(np.angle(co[peak]) * 180 / np.pi),
        model_qr_phase_deg=float(np.angle(cm[peak]) * 180 / np.pi),
        phase_error_deg=phase_error * 180 / np.pi,
        observed_qr_coherence=coherence,
        local=local,
        amplitude=amplitude,
        phase=phase,
        score=0.4 * local + 0.4 * amplitude + 0.2 * phase,
    )


def evaluate(job, substeps=32):
    start, theta, alpha, label = job
    a, b, i, _ = prior.load("trout_lake", start)
    sl = slice(i, i + 501)
    sim = make_simulator("trout_lake", profile="as_built", substeps=substeps)
    for k, v in zip(
        ["rotor_spin_inertia", "joint_k_yaw", "joint_c_yaw", "joint_c_pitch"], theta[:4]
    ):
        sim.parameters[k] *= v
    y0 = prior.historical.init(sim, a, b, i, alpha)
    commands = np.column_stack([a["U"][sl, 0], a["brake_mm"][sl]])
    u = np.array([sim.controls(*r) for r in commands])
    u[:, 1:] *= theta[4]
    if theta[5] > 1e-6:
        decay = np.exp(-0.04 / theta[5])
        applied = u[0, 1:].copy()
        for j in range(1, len(u)):
            applied = decay * applied + (1 - decay) * u[j, 1:]
            u[j, 1:] = applied
    row = dict(start=start, theta=list(theta), alpha=alpha)
    try:
        y, rpm = causal_replay(
            y0,
            u,
            sim.parameters,
            sim.geometry["mass"],
            a["wind"][i],
            float(b["voltage"][i]),
            float(b["rpm"][i]),
            dt=0.04,
            substeps=substeps,
        )
        assert np.isfinite(y).all()
        obs = a["Y"][sl, 3:6] * 180 / np.pi
        pred = rates(y)
        row.update(finite=True, **metrics(obs, pred))
        if label:
            np.savez_compressed(
                OUT / f"{label}-{start}-{alpha}.npz",
                time=a["t"][sl],
                observed_rates=obs,
                model_rates=pred,
                commands=commands,
                model=y,
            )
    except (ValueError, AssertionError, FloatingPointError) as exc:
        row.update(finite=False, score=1e6, error=str(exc))
    return row


def summarize(base, new):
    bm = {(r["start"], r["alpha"]): r for r in base}
    ratios = [r["score"] / bm[r["start"], r["alpha"]]["score"] for r in new]
    local = [r["local"] / bm[r["start"], r["alpha"]]["local"] for r in new]
    amplitude = [r["amplitude"] / bm[r["start"], r["alpha"]]["amplitude"] for r in new]
    phase = float(
        np.mean(
            [
                abs(r["phase_error_deg"])
                for r in new
                if r["observed_qr_coherence"] >= 0.5
            ]
        )
    )
    oldphase = float(
        np.mean(
            [
                abs(r["phase_error_deg"])
                for r in base
                if r["observed_qr_coherence"] >= 0.5
            ]
        )
    )
    channels = np.mean([r["rate_rmse"] for r in new], axis=0) / np.mean(
        [r["rate_rmse"] for r in base], axis=0
    )
    return dict(
        mean_score_ratio=float(np.mean(ratios)),
        worst_score_ratio=max(ratios),
        mean_local_ratio=float(np.mean(local)),
        mean_amplitude_error_ratio=float(np.mean(amplitude)),
        phase_error_deg=phase,
        baseline_phase_error_deg=oldphase,
        rate_error_ratios=channels.tolist(),
        improved=sum(v < 1 for v in ratios),
        cases=len(ratios),
    )


def main():
    protocol = dict(
        names=NAMES,
        bounds=BOUNDS,
        default=DEFAULT,
        training=TRAIN,
        manual_validation=MANUAL_CHECK,
        automatic_validation=AUTO_CHECK,
        duration_s=20,
        initial_alpha_train=0.35,
        initial_alpha_validation=[0.25, 0.35, 0.45],
        model="Production Trout Lake 1.758 kg / 132 mm; no adoption of previous candidate bundle",
        weights=dict(local=0.4, log_band_amplitude=0.4, pitch_yaw_phase=0.2),
        band_hz=[1.2, 1.9],
        optimizer="Bounded Powell; 2 specified starts; max 220 evaluations each; best finite training score frozen before validation",
        gates=dict(
            manual_score_ratio=0.9,
            auto_score_ratio=0.95,
            maximum_rate_error_ratio=1.05,
            maximum_local_ratio=1.05,
            maximum_amplitude_error_ratio=0.95,
            phase_increase_deg=5,
        ),
        bounds_interpretation="Exploratory uncertainty bounds, not confidence intervals or new measurements. Pull scale +/-15%; causal first-order actuator lag 0..120 ms.",
        limitations="Only three training manual episodes and two validation manual episodes within one historical flight. Reused log evidence. Cross spectra from short driven windows do not establish causal damping. No course/velocity drift objective and no automatic promotion.",
    )
    save("protocol.json", protocol)
    raw = json.loads(
        (prior.historical.DATA.parent / "2026-02-15 15-38-50/MODE.json").read_text()
    )
    mt = np.array([r["TimeUS"] / 1e6 for r in raw])
    mv = np.array([r["ModeNum"] for r in raw])
    for start in TRAIN + MANUAL_CHECK:
        t = np.arange(start, start + 20, 0.04)
        assert (mv[np.searchsorted(mt, t, side="right") - 1] == 0).all()
    for start in AUTO_CHECK:
        t = np.arange(start, start + 20, 0.04)
        assert (mv[np.searchsorted(mt, t, side="right") - 1] != 0).all()
    trace = []
    status = []
    with ProcessPoolExecutor(max_workers=4) as pool:

        def batch(starts, theta, alpha=0.35, label=""):
            return list(pool.map(evaluate, [(s, theta, alpha, label) for s in starts]))

        base = batch(TRAIN, DEFAULT)
        save("baseline-training.json", base)
        for seed, x in enumerate([DEFAULT, [1.0, 1.0, 0.5, 0.5, 1.0, 0.04]]):

            def objective(theta):
                rows = batch(TRAIN, theta)
                fit = float(np.mean([r["score"] for r in rows]))
                penalty = 0.005 * float(np.mean(np.log(theta[:5]) ** 2))
                trace.append(
                    dict(
                        seed=seed,
                        theta=list(theta),
                        fit=fit,
                        objective=fit + penalty,
                        finite=all(r["finite"] for r in rows),
                    )
                )
                if len(trace) % 20 == 0:
                    print("evaluation", len(trace), "fit", fit, flush=True)
                return fit + penalty

            opt = minimize(
                objective,
                x,
                method="Powell",
                bounds=BOUNDS,
                options=dict(maxfev=220, xtol=0.005, ftol=0.001),
            )
            status.append(
                dict(
                    seed=seed,
                    success=bool(opt.success),
                    message=str(opt.message),
                    nfev=int(opt.nfev),
                )
            )
            save("optimization.json", trace)
            save("optimizer-status.json", status)
        selected = min([r for r in trace if r["finite"]], key=lambda r: r["objective"])
        save("selection.json", selected)
        print("Selected", selected, flush=True)
        rows = []
        for family, theta in [("baseline", DEFAULT), ("candidate", selected["theta"])]:
            for alpha in [0.25, 0.35, 0.45]:
                rows.extend(
                    dict(r, family=family)
                    for r in batch(MANUAL_CHECK + AUTO_CHECK, theta, alpha, family)
                )
        save("validation.json", rows)
    summary = {}
    for name, starts in [("manual", MANUAL_CHECK), ("automatic", AUTO_CHECK)]:
        r = [x for x in rows if x["start"] in starts]
        summary[name] = summarize(
            [x for x in r if x["family"] == "baseline"],
            [x for x in r if x["family"] == "candidate"],
        )
    checks = {}
    for name, threshold in [("manual", 0.9), ("automatic", 0.95)]:
        r = summary[name]
        checks[name] = bool(
            r["mean_score_ratio"] <= threshold
            and max(r["rate_error_ratios"]) <= 1.05
            and r["mean_local_ratio"] <= 1.05
            and r["mean_amplitude_error_ratio"] <= 0.95
            and r["phase_error_deg"] <= r["baseline_phase_error_deg"] + 5
        )
    summary.update(gates=checks, passed=all(checks.values()), production_changed=False)
    save("summary.json", summary)
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == "__main__":
    main()
