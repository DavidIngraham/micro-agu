"""Produce review figures and report from frozen, separately evaluated fits."""

import json
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parent
LABELS = {
    "legacy": "Previous objective",
    "response": "Direct response",
    "response_nuisance": "Response + nuisance",
}


def read(name):
    return json.loads((OUT / name).read_text())


def main():
    summary = read("summary.json")
    selections = read("frozen-selection.json")
    validation = read("validation.json")
    synth = read("synthetic-summary.json")
    read("audit.json")
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5), layout="constrained")
    for ax, group in zip(axes, ["manual", "nonmanual"]):
        x = np.arange(4)
        for i, key in enumerate(LABELS):
            s = summary[key][group]
            ax.bar(
                x + (i - 1) * 0.25,
                [s["mean_local_ratio"], *s["rate_error_ratios"]],
                0.25,
                label=LABELS[key],
            )
        ax.axhline(1, color="black", lw=1)
        ax.set_xticks(x, ["Local dynamics", "Roll rate", "Pitch rate", "Yaw rate"])
        ax.set(
            title=group.title(),
            ylabel="Error relative to production (lower is better)",
            ylim=(0.85, 1.13),
        )
        ax.grid(axis="y", alpha=0.2)
    axes[0].legend(fontsize=8)
    fig.savefig(OUT / "heldout-dynamics.png", dpi=170)
    plt.close(fig)
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5), layout="constrained")
    amplitudes = {}
    for ax, group in zip(axes, ["manual", "nonmanual"]):
        x = np.arange(3)
        amplitudes[group] = {}
        for i, family in enumerate(["baseline", *LABELS]):
            rows = [
                r
                for r in validation
                if r["family"] == family and r["episode"].startswith(group + "-")
            ]
            amplitude = np.mean([r["model_band_rms"] for r in rows], axis=0)
            amplitudes[group][family] = amplitude.tolist()
            ax.bar(
                x + (i - 1.5) * 0.18,
                amplitude,
                0.18,
                label=LABELS.get(family, "Production"),
            )
        obs = np.mean([r["observed_band_rms"] for r in rows], axis=0)
        amplitudes[group]["observed"] = obs.tolist()
        ax.plot(x, obs, "k_", ms=25, mew=3, label="Flight")
        ax.set_xticks(x, ["Roll", "Pitch", "Yaw"])
        ax.set(title=group.title(), ylabel="1.2–1.9 Hz rate RMS (deg/s)")
    axes[0].legend(fontsize=8)
    fig.savefig(OUT / "oscillation-amplitude.png", dpi=170)
    plt.close(fig)
    timing = {}
    for group in ["manual", "nonmanual"]:
        timing[group] = {}
        for path in ["differential_to_yaw", "common_to_pitch"]:
            by = {
                f: {
                    r["name"]: r["command_response"][path]
                    for r in validation
                    if r["family"] == f
                    and r["alpha"] == 0.35
                    and r["episode"].startswith(group + "-")
                }
                for f in ["baseline", *LABELS]
            }
            common = [
                n
                for n in by["baseline"]
                if all(v[n]["interpretable"] for v in by.values())
            ]
            timing[group][path] = {
                "windows": common,
                "mean_absolute_phase_error": {
                    f: float(np.mean([abs(v[n]["phase_error_deg"]) for n in common]))
                    if common
                    else None
                    for f, v in by.items()
                },
            }
    (OUT / "timing-summary.json").write_text(json.dumps(timing, indent=2) + "\n")
    (OUT / "band-amplitudes.json").write_text(json.dumps(amplitudes, indent=2) + "\n")
    table = []
    for key, label in LABELS.items():
        for group in ["manual", "nonmanual"]:
            s = summary[key][group]
            table.append(
                f"| {label} | {group} | {s['mean_local_ratio']:.3f} | {s['uncentered_response_ratio']:.3f} | {s['rate_error_ratios'][2]:.3f} | {s['mean_amplitude_error_ratio']:.3f} |"
            )
    params = [
        "Rotor inertia scale",
        "Yaw stiffness scale",
        "Yaw damping scale",
        "Pitch damping scale",
        "Pull gain",
        "Actuator lag (s)",
    ]
    pt = [
        "| "
        + name
        + " | "
        + " | ".join(f"{selections[k]['theta'][i]:.5f}" for k in LABELS)
        + " |"
        for i, name in enumerate(params)
    ]
    text = (
        r"""# Improved grey-box identification: first reusable pipeline

The direct-response objective improves local validation dynamics modestly; it does not resolve the missing oscillation response. Bounded wind/initial-state corrections help known-physics synthetic recovery but add little on the real flight. No production physics, firmware, LQI gain or browser deployment changed.

## Implemented

Reusable `microagu_control.sysid` modules provide validated prepared-log sessions, recorded-input Rust replay, the exact legacy objective, a direct-response objective, bounded per-training-window initial-angle/wind corrections, finite-difference least squares, sensitivity diagnostics, episode-isolated free-running evaluation, and `fit`/`validate` CLI operations with input/source manifests. Original inputs are read-only. No research-module imports or hidden deployment actions exist in the package.

This is the analysis foundation, not the complete workbench UI. Direct raw DataFlash ingestion, automatic maneuver discovery, full latent time-varying disturbance estimation, controller synthesis/install UI, and runtime gain loading are not implemented by this change. Prepared NPZ inputs use the existing causal extraction. Ground-wind CSV support is tested synthetically; these historical flights have no measured ground wind input.

## Protocol and scope

Five 20 s training windows from three manual episodes; three manual validation windows from two other episodes and nine non-manual validation windows. Every validation replay is repeated at initial canopy angles 0.25, 0.35 and 0.45 rad. These 36 replays are sensitivity cases, not 36 independent flights. All data are reused historical Trout Lake evidence. Complete declared episodes are separated and time overlap is rejected, but episode labels must be supplied correctly.

Use the same six exploratory parameter bounds and two starts as the previous study. Each start has a 2,000 individual-replay ceiling, counting finite-difference probes. Candidates are frozen by training cost before any validation. Validation never optimizes wind or initial states against its output. The new objective is the mean of normalized, uncentered rate error and Savitzky–Golay angular-acceleration error, plus the previous weak scale-factor regularization:

$$
J_{\mathrm{response}}=\tfrac12\operatorname{mean}[(e_\omega/s_\omega)^2]+\tfrac12\operatorname{mean}[(e_{\dot\omega}/s_{\dot\omega})^2]+J_{\mathrm{prior}}.
$$

The first 2 s and final 0.4 s are excluded as in the legacy study. Rate normalization floors are 1 deg/s; acceleration floors are 5 deg/s². These are engineering weights, not sensor-noise covariances or a calibrated likelihood. Rate/acceleration residuals share information. Spectral amplitude and phase no longer determine the new fit; they remain independent diagnostic outputs.

Optional nuisance variables are one initial canopy-angle offset (bounded +/-0.1 rad) and constant horizontal wind corrections (bounded +/-2 m/s) per training window. Prior scales are 0.1 rad and 2 m/s, with residual weight 0.02. Ground wind, if provided, substitutes a time-aligned slow horizontal mean and an explicit airborne-difference uncertainty. No sample-for-sample ground gust forcing or arbitrary latent force is inferred. Aircraft geometry and mass remain the as-built profile, including 132 mm spacing.

## Baseline reproduction

All five training scores reproduce exactly (maximum absolute difference zero). The legacy fitted parameter vector and objective reproduce the previous gradient study to numerical precision. This separates changes in estimation policy from packaging changes.

## Reserved-window results

Ratios below one improve on the production model. Local dynamics is the previous centered rate/acceleration metric. Response loss is the new uncentered rate/acceleration metric. Amplitude loss is squared log band-RMS mismatch, not the physical amplitude itself.

| Method | Windows | Local dynamics | Response loss | Yaw-rate RMSE | Amplitude loss |
|---|---|---:|---:|---:|---:|
"""
        + "\n".join(table)
        + """

![Held-out local dynamics](heldout-dynamics.png)

Direct-response fitting reduces local error 1.6% in manual windows and 7.2% in non-manual windows, compared with production. The legacy objective increases that error 7.5% and 8.3%. Roll and pitch account for most of the new improvement; yaw-rate RMSE is still 0.6% and 3.1% worse. Oscillation-amplitude loss becomes about 3.6% and 2.1% worse than production, respectively. This is a more useful local-dynamics compromise, not an across-the-board fidelity improvement.

![Remaining oscillation-amplitude mismatch](oscillation-amplitude.png)

Nuisance fitting changes the real-data validation results only slightly. It therefore remains optional. The source data do not establish that its wind corrections represent real wind; they may absorb other mismatch.

## Fitted parameters and limitations

| Parameter | Previous objective | Direct response | Response + nuisance |
|---|---:|---:|---:|
"""
        + "\n".join(pt)
        + """

Both direct-response fits reach the 120 ms lag upper bound and nearly the yaw-damping lower bound. This is an effective command-response parameter, not a measured servo time constant. It could absorb synchronization, routing, disturbance or model errors. Do not widen bounds or promote the candidate on this evidence alone. `audit.json` reports local response sensitivities and integration-resolution checks. The full conditional Jacobian can be rank deficient at zero lag; that is flagged rather than turned into a confidence interval.

## Known-physics synthetic experiments

Generate trajectories using the real Rust simulator at a known six-parameter vector, then fit with the reusable estimator. Two training windows and a third validation window use historical command patterns; the initial full state is known. This is an implementation/recovery test under the assumed model, not evidence that the real aircraft follows that model.

"""
    )
    for name, r in synth.items():
        text += f"- **{name}**: parameter-error norm in bound-width units {r['normalized_parameter_error']:.6g}; held-out response loss with known wind {r['known_wind_validation_response_loss']:.6g}.\n"
    text += """
The noise-free case recovers all six parameters to within 0.0001% of their bound widths. With 0.1 deg/s Gaussian gyro noise and a deliberately biased wind prior (north +0.6 m/s, east -0.4 m/s), nuisance fitting reduces the parameter-error norm by about 58%, but that error was already small. Held-out response loss improves only about 5%. The fitted wind components do not recover the complete imposed bias, so this is not proof of an accurate wind estimator.

## Decision and next step

Keep the legacy baseline available and make direct-response fitting the transparent analysis default. Leave nuisance correction opt-in. Do not deploy these fitted aircraft parameters or retune LQI from them yet: yaw response and oscillation amplitude remain unresolved, with a lag-bound warning. The next physics investigation should audit input timing/routing and the command/yaw discrepancy across complete episodes, then determine whether a dynamic disturbance model is supported. Prospective flight data and ground-wind measurements remain necessary to establish generalization.

## Reproduction

Run `study.py`, `synthetic.py`, and `audit.py` with the simulation analysis environment, then `report.py` with NumPy/Matplotlib. Set `MICROAGU_REPLAY_DATA` to the prepared flight-data folder when needed. `session.json` records explicit input paths, segments and roles. The study asserts baseline equivalence before fitting. See the package README for the reusable CLI. CLI smoke-test artifacts demonstrate actual fit and validation execution; the smoke fit intentionally has a one-replay budget and is not a scientific result.
"""
    text += "\n## Command-response phase audit\n\nCommon coherence-screened nominal-initialization windows; mean absolute circular error in degrees. Short-window feedback correlations are descriptive, not causal transfer functions.\n\n| Group | Path | Windows | Production | Legacy | Response | Response + nuisance |\n|---|---|---:|---:|---:|---:|---:|\n"
    for group, paths in timing.items():
        for path, r in paths.items():
            vals = [r["mean_absolute_phase_error"][f] for f in ["baseline", *LABELS]]
            text += (
                f"| {group} | {path} | {len(r['windows'])} | "
                + " | ".join("n/a" if v is None else f"{v:.1f}" for v in vals)
                + " |\n"
            )
    episodes = read("episode-audit.json")
    baseline = [r for r in episodes if r["family"] == "baseline"]
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5), layout="constrained")
    x = np.arange(len(baseline))
    for i, key in enumerate(LABELS):
        rows = [r for r in episodes if r["family"] == key]
        axes[0].bar(
            x + (i - 1) * 0.25,
            [r["response_loss"] / b["response_loss"] for r, b in zip(rows, baseline)],
            0.25,
            label=LABELS[key],
        )
    axes[0].axhline(1, color="black", lw=1)
    axes[0].set(ylabel="Response loss / production", title="Complete manual episodes")
    for n, marker in [(128, "o"), (256, "s")]:
        values = [
            next(v for v in r["spectral"] if v["nperseg"] == n)["channels"][
                "differential_yaw"
            ]["observed_coherence"]
            for r in baseline
        ]
        axes[1].plot(x, values, marker + "-", label=f"{n}-sample Welch")
    axes[1].axhline(0.5, color="grey", ls="--", lw=1)
    axes[1].set(
        ylabel="Observed steering/yaw coherence",
        ylim=(0, 1),
        title="At observed 1.2–1.9 Hz yaw peak",
    )
    labels = [
        r["episode"].replace("manual-", "")
        + (" *" if r["kind"] == "unused_check" else "")
        for r in baseline
    ]
    for ax in axes:
        ax.set_xticks(x, labels, rotation=45)
        ax.set_xlabel("Manual episode start (s); * excluded from fit/selection")
        ax.legend(fontsize=8)
    fig.savefig(OUT / "episode-audit.png", dpi=170)
    plt.close(fig)
    text += """

## Full-episode follow-up: revise the short-window phase interpretation

The earlier approximately 165-degree command/yaw mismatch is not a robust plant-phase estimate. A 20 s record with a 256-sample Welch window yields only two overlapping averages after trimming. The original coherence threshold alone was insufficient to support that interpretation.

Across the five longer manual episodes, using 128-sample Welch windows and 10–26 overlapping averages, observed steering/yaw coherence at the oscillation peak is only about 0.02–0.08. The phase changes substantially with window length and is not a reliable discriminator between models here. Feedback, nonstationarity, coupled inputs and unmeasured disturbances can explain weak association; these data do not uniquely identify which mechanism dominates. The software now records the averaging count and refuses to label a two-average phase estimate interpretable, even if its coherence exceeds 0.5. A six-average minimum is an explicit quality heuristic, not a statistical significance guarantee or proof of causality. Historical scalar phase metrics remain unchanged for baseline reproduction.

![Full-episode response and coherence](episode-audit.png)

Two additional manual episodes (903.80–926.12 and 1087.20–1106.00 s), excluded from this comparison's fitting/selection and previous 20 s gradient validation, improve direct-response loss by about 4.0% and 10.2% against production. The legacy candidate worsens it by about 11% and 20%. These remain historical same-flight checks; broader prior studies may have examined overlapping data. They strengthen evidence for the direct-response objective while leaving overall physics adequacy unresolved.

Causal source timestamps show 95th-percentile sample ages of about 20 ms for IMU and 40 ms for actuator outputs. There is no 120 ms sample-age discrepancy visible in this audit. This does not measure mechanical servo response, firmware/sensor internal latency, or prove perfect synchronization. The fitted lag remains a bound-limited effective parameter requiring separate physical investigation.
"""
    (OUT / "README.md").write_text(text)
    print(json.dumps(timing, indent=2))


if __name__ == "__main__":
    main()
