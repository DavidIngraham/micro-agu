"""Physical response metrics; spectral features are validation diagnostics."""

import numpy as np
from scipy.signal import csd, savgol_filter, welch


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
    return {
        "rate_rmse": rmse.tolist(),
        "acceleration_rmse": acc.tolist(),
        "mean_rate_bias": (model - actual).mean(axis=0).tolist(),
        "observed_band_rms": ao.tolist(),
        "model_band_rms": am.tolist(),
        "peak_hz": float(f[peak]),
        "observed_qr_phase_deg": float(np.angle(co[peak]) * 180 / np.pi),
        "model_qr_phase_deg": float(np.angle(cm[peak]) * 180 / np.pi),
        "phase_error_deg": phase_error * 180 / np.pi,
        "observed_qr_coherence": coherence,
        "local": local,
        "amplitude": amplitude,
        "phase": phase,
        "score": 0.4 * local + 0.4 * amplitude + 0.2 * phase,
    }


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
    return {
        "mean_score_ratio": float(np.mean(ratios)),
        "worst_score_ratio": max(ratios),
        "mean_local_ratio": float(np.mean(local)),
        "mean_amplitude_error_ratio": float(np.mean(amplitude)),
        "phase_error_deg": phase,
        "baseline_phase_error_deg": oldphase,
        "rate_error_ratios": channels.tolist(),
        "improved": sum(v < 1 for v in ratios),
        "cases": len(ratios),
    }


def legacy_residual(obs, pred):
    a, m = obs[50:-10], pred[50:-10]
    e = m - a
    e -= e.mean(axis=0)
    da = savgol_filter(a, 11, 3, deriv=1, delta=0.04, axis=0)[6:-6]
    dm = savgol_filter(m, 11, 3, deriv=1, delta=0.04, axis=0)[6:-6]
    de = dm - da
    de -= de.mean(axis=0)
    rate = (e / np.maximum(a.std(axis=0), 1)).ravel() * np.sqrt(0.4 / (6 * len(a)))
    acc = (de / np.maximum(da.std(axis=0), 5)).ravel() * np.sqrt(0.4 / (6 * len(da)))
    f, po = welch(a, fs=25, nperseg=256, noverlap=128, axis=0)
    _, pm = welch(m, fs=25, nperseg=256, noverlap=128, axis=0)
    band = (f >= 1.2) & (f <= 1.9)
    ao = np.sqrt(po[band].sum(axis=0) * (f[1] - f[0]))
    am = np.sqrt(pm[band].sum(axis=0) * (f[1] - f[0]))
    amplitude = np.log(np.maximum(am, 0.001) / np.maximum(ao, 0.001)) * np.sqrt(0.4 / 3)
    peak = np.flatnonzero(band)[np.argmax(po[band, 2])]
    _, co = csd(a[:, 2], a[:, 1], fs=25, nperseg=256, noverlap=128)
    _, cm = csd(m[:, 2], m[:, 1], fs=25, nperseg=256, noverlap=128)
    coherent = abs(co[peak]) ** 2 / max(po[peak, 1] * po[peak, 2], 1e-12) >= 0.5
    phase = np.array(
        [
            np.cos(np.angle(cm[peak])) - np.cos(np.angle(co[peak])),
            np.sin(np.angle(cm[peak])) - np.sin(np.angle(co[peak])),
        ]
    ) * np.sqrt(0.1)
    if not coherent:
        phase *= 0
    r = np.r_[rate, acc, amplitude, phase]
    assert np.isclose(r @ r, metrics(obs, pred)["score"], rtol=1e-9)
    return r


def timing(obs, pred, commands):
    a, m, u = obs[50:-10], pred[50:-10], commands[50:-10]
    result = {}
    averages = 1 + (len(a) - 256) // 128
    for name, command, axis in [
        ("differential_to_yaw", u[:, 2] - u[:, 1], 2),
        ("common_to_pitch", u[:, 1:3].mean(axis=1), 1),
    ]:
        f, pu = welch(command, fs=25, nperseg=256, noverlap=128)
        _, pa = welch(a[:, axis], fs=25, nperseg=256, noverlap=128)
        _, pm = welch(m[:, axis], fs=25, nperseg=256, noverlap=128)
        _, ca = csd(command, a[:, axis], fs=25, nperseg=256, noverlap=128)
        _, cm = csd(command, m[:, axis], fs=25, nperseg=256, noverlap=128)
        coa = abs(ca) ** 2 / np.maximum(pu * pa, 1e-20)
        com = abs(cm) ** 2 / np.maximum(pu * pm, 1e-20)
        band = (f >= 1.2) & (f <= 1.9)
        peak = np.flatnonzero(band)[np.argmax(pa[band])]
        result[name] = {
            "frequency_hz": float(f[peak]),
            "command_std_mm": float(command.std()),
            "observed_coherence": float(coa[peak]),
            "model_coherence": float(com[peak]),
            "observed_phase_deg": float(np.rad2deg(np.angle(ca[peak]))),
            "model_phase_deg": float(np.rad2deg(np.angle(cm[peak]))),
            "phase_error_deg": float(
                np.rad2deg(np.angle(cm[peak] * np.conj(ca[peak])))
            ),
            "welch_averages": averages,
            "coherence_screen_pass": bool(
                command.std() > 0.1 and coa[peak] >= 0.5 and com[peak] >= 0.5
            ),
            "interpretable": bool(
                averages >= 6
                and command.std() > 0.1
                and coa[peak] >= 0.5
                and com[peak] >= 0.5
            ),
            "qualification": "Six-average minimum is a quality heuristic, not a significance test. Feedback still prevents causal phase interpretation.",
        }
    return result


def response_residual(obs, pred):
    """Uncentered rates and accelerations; engineering scales, not a likelihood."""
    a, m = obs[50:-10], pred[50:-10]
    da = savgol_filter(a, 11, 3, deriv=1, delta=0.04, axis=0)[6:-6]
    dm = savgol_filter(m, 11, 3, deriv=1, delta=0.04, axis=0)[6:-6]
    return np.r_[
        ((m - a) / np.maximum(a.std(axis=0), 1)).ravel() * np.sqrt(0.5 / (3 * len(a))),
        ((dm - da) / np.maximum(da.std(axis=0), 5)).ravel()
        * np.sqrt(0.5 / (3 * len(da))),
    ]


def residual_diagnostics(obs, pred, commands):
    """Descriptive correlations, with no white-noise confidence claims."""
    e = (pred - obs)[50:-10]
    u = commands[50:-10]
    channels = {
        "differential_yaw": (u[:, 2] - u[:, 1], e[:, 2]),
        "common_pitch": (u[:, 1:3].mean(axis=1), e[:, 1]),
        "throttle_pitch": (u[:, 0], e[:, 1]),
    }
    result = {}

    def correlation(x, y):
        if x.std() < 1e-12 or y.std() < 1e-12:
            return None
        return float(np.corrcoef(x, y)[0, 1])

    for name, (command, error) in channels.items():
        result[name] = {
            "lag_seconds": (np.arange(1, 26) * 0.04).tolist(),
            "past_command_vs_error": [
                correlation(command[:-k], error[k:]) for k in range(1, 26)
            ],
            "future_command_vs_error": [
                correlation(command[k:], error[:-k]) for k in range(1, 26)
            ],
            "error_autocorrelation": [
                correlation(error[:-k], error[k:]) for k in range(1, 26)
            ],
        }
    return result
