"""Render the frozen gradient study results without rerunning physics."""

import json
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parent


def read(name):
    return json.loads((OUT / name).read_text())


def main():
    selected = read("selection.json")
    sensitivity = read("sensitivity.json")
    summary = read("summary.json")
    validation = read("validation.json")
    names = [
        "Rotor inertia",
        "Yaw stiffness",
        "Yaw damping",
        "Pitch damping",
        "Pull gain",
        "Actuator lag",
    ]
    fig, axes = plt.subplots(1, 2, figsize=(12, 5), layout="constrained")
    for ax, (label, r) in zip(axes, sensitivity.items()):
        values = np.array(r["cosine_similarity"])
        im = ax.imshow(values, vmin=-1, vmax=1, cmap="RdBu_r")
        for i in range(6):
            for j in range(6):
                ax.text(
                    j,
                    i,
                    "inactive"
                    if i in r["inactive_columns"] or j in r["inactive_columns"]
                    else f"{values[i, j]:.2f}",
                    ha="center",
                    va="center",
                    fontsize=7,
                )
        ax.set_xticks(range(6), names, rotation=45, ha="right")
        ax.set_yticks(range(6), names)
        ax.set_title(label.replace("constrained", "Previous candidate").title())
    fig.colorbar(im, ax=axes, label="Cosine similarity of residual sensitivity")
    fig.suptitle(
        "Which parameter changes look alike? (local, conditional sensitivities)"
    )
    fig.savefig(OUT / "sensitivity.png", dpi=170)
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(9, 5), layout="constrained")
    for tag in selected:
        trace = read(f"trace-{tag}.json")
        y = np.minimum.accumulate([r["objective"] for r in trace])
        ax.plot(
            np.arange(1, len(y) + 1),
            y,
            label=tag.replace("least_squares", "Least squares"),
        )
    ax.set(
        xlabel="Training batch evaluations (including gradient probes)",
        ylabel="Best training objective",
        title="Same objective, starts, bounds and 220-evaluation ceiling",
    )
    ax.legend()
    ax.grid(alpha=0.2)
    fig.savefig(OUT / "convergence.png", dpi=170)
    plt.close(fig)
    winners = {
        method: min(
            (k for k in selected if k.startswith(method)),
            key=lambda k: selected[k]["objective"],
        )
        for method in ["Powell", "least_squares"]
    }
    families = ["previous_constrained", *winners.values()]
    fig, axes = plt.subplots(1, 2, figsize=(11, 4), layout="constrained")
    for ax, group in zip(axes, ["manual", "nonmanual"]):
        x = np.arange(3)
        for i, family in enumerate(families):
            s = summary[family][group]
            ax.bar(
                x + (i - 1) * 0.25,
                [
                    s["mean_score_ratio"],
                    s["mean_local_ratio"],
                    s["mean_amplitude_error_ratio"],
                ],
                width=0.25,
                label=family.replace("least_squares", "Least squares").replace(
                    "previous_constrained", "Previous constrained"
                ),
            )
        ax.axhline(1, color="black", lw=1)
        ax.set_xticks(x, ["Combined score", "Local dynamics", "Band amplitude loss"])
        ax.set(
            ylabel="Ratio to production model (lower is better)", title=group.title()
        )
    axes[0].legend(fontsize=8)
    fig.savefig(OUT / "validation.png", dpi=170)
    plt.close(fig)
    timing = {}
    starts = {
        "manual": [380, 400, 440],
        "nonmanual": [1440, 1470, 1500, 2130, 2160, 2190, 2760, 2790, 2820],
    }
    for group, window_starts in starts.items():
        timing[group] = {}
        for pathway in ["differential_to_yaw", "common_to_pitch"]:
            by_family = {
                f: {
                    r["start"]: r["command_response"][pathway]
                    for r in validation
                    if r["family"] == f
                    and r["alpha"] == 0.35
                    and r["start"] in window_starts
                }
                for f in ["baseline", *families]
            }
            # All models share exactly the same subset for comparisons.
            common = [
                s
                for s in window_starts
                if all(v[s]["interpretable"] for v in by_family.values())
            ]
            timing[group][pathway] = {
                "common_windows": common,
                "total_windows": len(window_starts),
                "families": {
                    f: {
                        "mean_absolute_phase_error_deg": float(
                            np.mean([abs(rows[s]["phase_error_deg"]) for s in common])
                        )
                        if common
                        else None,
                        "rows": [dict(start=s, **r) for s, r in rows.items()],
                    }
                    for f, rows in by_family.items()
                },
            }
    (OUT / "timing-summary.json").write_text(json.dumps(timing, indent=2) + "\n")
    tables = []
    for family in families:
        for group in starts:
            s = summary[family][group]
            tables.append(
                f"| {family} | {group} | {s['mean_score_ratio']:.3f} | {s['mean_local_ratio']:.3f} | {s['rate_error_ratios'][2]:.3f} | {s['phase_error_deg']:.1f} |"
            )
    optimizer = []
    for k, v in selected.items():
        optimizer.append(
            f"| {k} | {v['evaluations']} | {v['objective']:.6f} | {v['wall_seconds']:.1f} |"
        )
    body = (
        r"""# Finite-difference fitting study — Trout Lake

Research only. No production physics, controller gains, browser runtime or deployment changed.

## Findings

Finite-difference least squares works with the existing Rust simulator and reaches essentially the same solution faster. Its best objective is 0.763407 versus Powell's 0.763021, a 0.051% difference. The two least-squares starts finish in 104 and 78 unique training batches; Powell uses 220 and 214. Including both starts, that is 182 versus 434 batches. These are two starts on one problem, not a general optimizer benchmark. Powell reached its function-call limit; least squares met its termination tolerances, which do not establish a global optimum.

The selected gradient candidate improves the combined validation score by about 43%, but local dynamics error rises 7.5% in manual windows and 8.3% in non-manual windows. Yaw-rate RMSE rises 9.4% and 7.8%, respectively. Its yaw oscillation RMS is 7.92 versus 21.85 deg/s observed in manual windows, and 7.17 versus 14.44 deg/s in non-manual windows. The faster optimizer has not resolved the model-fidelity problem. The previous constrained fit remains the better local-error compromise; no new candidate is promoted.

Numerical derivatives are usable. Halving the sensitivity step changes each active column by less than 0.01%. At the prior constrained candidate, the residual Jacobian has condition number 12.2 in normalized bound units: there is overlap among parameter effects, but no severe local rank deficiency. Pull gain and lag have sensitivity cosine -0.73, rotor inertia and yaw stiffness -0.72, and pitch/yaw damping +0.70. These are local directions, not posterior parameter correlations. At production's zero lag, lag sensitivity is exactly zero at this perturbation size because the sampled exponential filter is effectively instantaneous.

Allowing each training window its own unknown initial canopy angle removes at most 1.1% of any parameter's sensitivity-column norm; condition number remains 12.2. Within this local calculation and after discarding the first two seconds, initial canopy angle does not explain the fitting limitation. This does not eliminate wind or other unmodeled forcing as confounders. Doubling physics substeps from 32 to 64 changes checked rate RMSE by less than 1.2e-7 deg/s, so integration resolution is not the limiting issue here.

The new command-response check exposes a separate discrepancy: manual differential-command/yaw phase remains approximately 165 degrees wrong in the two windows meeting the coherence screen. The earlier pitch/yaw phase metric does not constrain this. In non-manual windows the equivalent phase error is about 29 degrees. The evidence is insufficient to infer a reversed aerodynamic response or an actuator delay: pilot/autopilot feedback, unmeasured forcing, and short records can all affect these associations.

## Recommended next experiment

Use finite-difference least squares for subsequent calibration, retaining multiple starts and an occasional Powell cross-check. A differentiable Rust rewrite is not justified by this six-parameter problem. Before expanding parameter bounds, investigate the manual command/yaw discrepancy across contiguous episodes: audit channel sign, routing, and timestamps, then compare frequency-dependent command/response behavior and residual motion across windows. Distinguish disturbance-driven motion from commanded response before adding a command-phase penalty. A single lag or further damping reduction should not be used to absorb a discrepancy whose physical source is unresolved. Keep local-rate constraints explicit in any next calibration; this study intentionally preserved the original objective to isolate the optimizer comparison.

## Protocol

Use the existing six-parameter, recorded-input, 20 s manual-flight fit. The production Trout Lake mass and 132 mm geometry are fixed. ArduPilot is not in the replay loop. Train on starts 280, 310, 535, 555 and 1988 s; evaluate on the previous three manual and nine non-manual windows, each at initial canopy angles 0.25, 0.35 and 0.45 rad. These are reused windows from the same flight, not new independent evidence.

Both optimizers use exactly the same residual objective, parameter bounds and two starts. Each start has a ceiling of 220 unique five-window simulation batches; finite-difference probes count toward this budget. Early convergence is allowed. Parameters are normalized by their bound widths. Powell uses a scalar objective; bounded trust-region least squares uses a bound-aware central-difference residual Jacobian with step 0.001 of the parameter range. All candidate choices are frozen using training scores before evaluating validation windows. Select the best training run per method, not the best validation run.

The sum of squared residuals reproduces the earlier objective:

$$
J=0.4J_{\mathrm{local}}+0.4J_{\mathrm{amplitude}}+0.2J_{\mathrm{phase}}+0.005\operatorname{mean}(\log^2 s_i).
$$

Local residuals are centered rate and angular-acceleration errors scaled by observed variability. Amplitude residuals are log RMS ratios in 1.2–1.9 Hz. Two sine/cosine residuals encode circular pitch/yaw phase error without an angle-wrap discontinuity. The five scale factors have weak log regularization; lag does not. This is the original unconstrained objective, not the second study's local-regression penalty. The prior constrained candidate is a reference, not an equal-budget optimizer competitor.

Every replay asserts equality between squared residual norm and the existing scalar score. Identifiability SVD excludes regularization residuals. Sensitivity columns use full-bound parameter units; these local, conditional diagnostics are not physical confidence intervals.

## Optimization

| Run | Simulation batches | Best objective | Batch wall seconds |
|---|---:|---:|---:|
"""
        + "\n".join(optimizer)
        + """

![Training convergence](convergence.png)

## Evaluation

Ratios below one improve on production. Local error includes rate and angular acceleration. The combined score trades local performance against oscillation amplitude and inter-axis phase; it must not be interpreted as a percentage improvement in physical fidelity.

| Candidate | Windows | Combined ratio | Local ratio | Yaw-rate RMSE ratio | Pitch/yaw phase error (deg) |
|---|---|---:|---:|---:|---:|
"""
        + "\n".join(tables)
        + """

![Evaluation against production](validation.png)

## Sensitivity and identifiability

![Sensitivity similarity](sensitivity.png)

See `sensitivity.json` for singular values, derivative-step consistency, column norms and weak directions. An inactive zero-lag column is a feature of the sampled exponential lag model: tiny time constants have numerically zero influence at the 40 ms input step. A nonzero starting lag is necessary for local derivative methods to explore this coordinate. The 40 ms second start provides that test without changing the simulator.

## Command-to-response timing

`timing-summary.json` records differential-command/yaw and common-command/pitch cross-spectral phase at the observed output peak in 1.2–1.9 Hz. Comparisons use the same nominal-initialization windows across models, with command standard deviation above 0.1 mm and both observed and simulated coherence at least 0.5. No timing score enters fitting or selection.

These short windows provide only a few Welch segments, so coherence and phase have substantial uncertainty. Manual pilot feedback can correlate commands with disturbances; non-manual windows additionally contain autopilot feedback. These are descriptive associations, not identified transfer functions, measured actuator delay, or causal plant damping. No phase is converted into a time delay because a coupled oscillatory response is not a pure delay.

## Reproduction

Run `uv run --project research python research/active/gradient-fit-2026-10-18/run.py` from the repository. `report.py` needs NumPy and Matplotlib and only reads saved results. Raw input locations and the historical data loader are inherited from `manual-coupling-fit-2026-10-18/run.py`; set `MICROAGU_REPLAY_DATA` when necessary. Replay NPZ files are local regenerable artifacts; compact metrics, protocols, traces and figures are versioned.
"""
    )
    body += "\n## Command-response phase results\n\nMean absolute circular phase error in degrees; same screened windows for all models, nominal initial angle only.\n\n| Windows | Path | Count | Production | Previous constrained | Powell | Least squares |\n|---|---|---:|---:|---:|---:|---:|\n"
    for group, pathways in timing.items():
        for path, values in pathways.items():
            vals = [
                values["families"][f]["mean_absolute_phase_error_deg"]
                for f in ["baseline", *families]
            ]
            formatted = [f"{v:.1f}" if v is not None else "n/a" for v in vals]
            body += (
                f"| {group} | {path} | {len(values['common_windows'])}/{values['total_windows']} | "
                + " | ".join(formatted)
                + " |\n"
            )
    (OUT / "README.md").write_text(body)
    print(
        json.dumps(
            {
                "winners": winners,
                "timing_common_windows": {
                    g: {p: v["common_windows"] for p, v in r.items()}
                    for g, r in timing.items()
                },
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
