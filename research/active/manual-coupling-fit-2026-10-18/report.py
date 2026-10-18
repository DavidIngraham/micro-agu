"""Publish the two-stage manual coupling experiment without changing selections."""

import json
from pathlib import Path
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parent
read = lambda n: json.loads((OUT / n).read_text())
first = read("summary.json")
second = read("constrained-summary.json")
a = read("selection.json")
b = read("constrained-selection.json")
rows = read("validation.json")
more = read("constrained-validation.json")
manual = {380, 400, 440}
automatic = {1440, 1470, 1500, 2130, 2160, 2190, 2760, 2790, 2820}
fig, axs = plt.subplots(1, 2, figsize=(11, 4.5))
for ax, (label, starts) in zip(axs, [("Manual", manual), ("Non-manual", automatic)]):
    sets = [
        (
            "Flight",
            [r for r in rows if r["family"] == "baseline" and r["start"] in starts],
            "observed_band_rms",
        ),
        (
            "Baseline",
            [r for r in rows if r["family"] == "baseline" and r["start"] in starts],
            "model_band_rms",
        ),
        (
            "First fit",
            [r for r in rows if r["family"] == "candidate" and r["start"] in starts],
            "model_band_rms",
        ),
        ("Constrained", [r for r in more if r["start"] in starts], "model_band_rms"),
    ]
    for j, (name, r, key) in enumerate(sets):
        ax.bar(
            np.arange(3) + (j - 1.5) * 0.19,
            np.mean([x[key] for x in r], axis=0),
            0.19,
            label=name,
        )
    ax.set_xticks(range(3), ["Roll", "Pitch", "Yaw"])
    ax.set_title(label)
    ax.set_ylabel("1.2–1.9 Hz rate RMS (deg/s)")
    ax.grid(axis="y", alpha=0.2)
axs[0].legend(fontsize=8)
fig.suptitle("Coupled-band amplitude: measured flight versus model replays")
fig.tight_layout()
fig.savefig(OUT / "band-amplitude.png", dpi=170)
fig, axs = plt.subplots(1, 2, figsize=(11, 4.5))
for ax, (label, name) in zip(axs, [("Manual", "manual"), ("Non-manual", "automatic")]):
    for j, (stage, s) in enumerate([("First fit", first), ("Constrained", second)]):
        ax.bar(
            np.arange(3) + (j - 0.5) * 0.3,
            100 * (np.array(s[name]["rate_error_ratios"]) - 1),
            0.3,
            label=stage,
        )
    ax.axhline(5, color="#b74b4b", linestyle="--", label="5% regression limit")
    ax.axhline(0, color="black", linewidth=0.7)
    ax.set_xticks(range(3), ["Roll", "Pitch", "Yaw"])
    ax.set_title(label)
    ax.set_ylabel("Demeaned rate RMSE change (%)")
    ax.grid(axis="y", alpha=0.2)
axs[0].legend(fontsize=8)
fig.tight_layout()
fig.savefig(OUT / "local-error.png", dpi=170)
table = "| Fit / checks | Combined score change | Band amplitude loss change | Pitch/yaw phase error | Yaw-rate RMSE change | Gates |\n|---|---:|---:|---:|---:|---|\n"
for stage, s in [("First", first), ("Constrained", second)]:
    for name in ["manual", "automatic"]:
        r = s[name]
        table += f"| {stage} / {name} | {100 * (r['mean_score_ratio'] - 1):+.1f}% | {100 * (r['mean_amplitude_error_ratio'] - 1):+.1f}% | {r['phase_error_deg']:.1f} deg | {100 * (r['rate_error_ratios'][2] - 1):+.1f}% | {'PASS' if s['gates'][name] else 'FAIL'} |\n"
param = "| Parameter | First fit | Constrained fit |\n|---|---:|---:|\n"
for key, x, y in zip(read("protocol.json")["names"], a["theta"], b["theta"]):
    param += f"| `{key}` | {x:.6g} | {y:.6g} |\n"
text = f"""# Manual-flight rotor/suspension coupling fit

This study targets local body-rate response, oscillation amplitude, and pitch/yaw phase. **Accumulated course and earth-frame velocity drift are not acceptance criteria.** Two bounded training searches were completed. First-stage gates pass: **{first["passed"]}**. Constrained-stage gates pass: **{second["passed"]}**. No production model or LQI gains were changed by these scripts.

## Configuration, split, and uncertainty

The production Trout Lake baseline retains 1.758 kg mass, historical 132 mm spreader geometry, inertia/CG configuration, and aerodynamic coefficients. The earlier multi-parameter candidate is not used as the base. This isolates rotor/suspension coupling and effective actuator input.

Training comprises five 20-second windows starting at 280, 310, 535, 555, and 1988 s, spanning three MANUAL episodes. Manual checks at 380, 400, and 440 s belong to two other manual episodes. Nine non-manual check windows start at 1440, 1470, 1500, 2130, 2160, 2190, 2760, 2790, and 2820 s. Mode membership is checked over each complete window. All segments are from the same previously examined flight; none is an independent flight test.

Search bounds, relative to the original model: rotor inertia and yaw stiffness 0.5–1.5 times; pitch and yaw damping 0.2–1.5 times; effective pull scale 0.85–1.15; causal discrete first-order input lag 0–120 ms. These are exploratory uncertainty bounds, not measurements or statistical confidence intervals. The pull scale multiplies the nominal line pull after the baseline zero offset; it is not a change to the aero coefficients. The lag operates on that pull at the 25 Hz command rate, initialized from the first available command. The kernel still applies geometric routing/slack. No servo position feedback exists to uniquely identify actual line motion.

## Objectives and acceptance

After excluding two seconds for initialization and 0.4 s at the end, local rate and smoothed angular-acceleration errors are demeaned and normalized by observed variability. The coupling-band loss uses squared log RMS amplitude ratios for all three axes in 1.2–1.9 Hz. Pitch/yaw phase uses circular phase error at each observed yaw-band peak, only where observed pitch/yaw coherence is at least 0.5. Weights are 40% local response, 40% amplitude, and 20% phase, plus small first-stage regularization. This prevents the local waveform term from being the sole incentive to suppress predicted oscillations.

The first stage uses two prescribed Powell starts with a 220-evaluation budget each. Candidate selection uses training only and is written before validation. Validation spans three initial canopy angles (0.25, 0.35, 0.45 rad). Gates require at least 10% combined improvement on manual checks and 5% on non-manual checks; no rate channel or local score more than 5% worse; at least 5% improvement in amplitude loss; and phase no more than 5 degrees worse.

The first fit exposed a local-response tradeoff. A second stage therefore constrains mean training local loss and each mean training rate RMSE to no more than 3% regression, with two starts and 180 evaluations each. It selects the best feasible evaluated training point. **Its validation repeats already inspected segments; it is explicitly reused evidence, not fresh confirmation.** Search budgets and convergence statuses are saved. No globally optimal or uniquely identified parameter estimate is claimed.

## Results

{table}

Baseline pitch/yaw phase errors are approximately 108.7 degrees on manual checks and 80.3 degrees on non-manual checks. Percentage amplitude-loss changes refer to squared logarithmic amplitude mismatch, not literal changes in oscillation amplitude. The figures show the actual RMS amplitudes and the local waveform tradeoff.

![Band amplitude](band-amplitude.png)

![Local waveform errors](local-error.png)

The constrained yaw-band RMS remains below flight: **6.18 versus 21.85 deg/s** on manual checks (baseline 2.57), and **5.56 versus 14.44 deg/s** on non-manual checks (baseline 2.17). Roll-band amplitude also remains strongly underpredicted. Thus the improvements do not constitute a complete match of the observed oscillation. Both damping multipliers remain near the search floor; the bounded search cannot establish whether lower damping would help or whether a different input/coupling mechanism is required.

## Parameters

{param}

The first fit pushes rotor inertia and pull scale near their upper bounds and both damping multipliers near their lower bounds. This does not establish that the physical rotor inertia is wrong or that damping is exactly one-fifth of the previous value. Such boundary behavior can compensate for missing input physics, forcing, or hidden-state uncertainty. The first fit selects only 2.7 ms additional lag, below a command sample interval. The constrained fit selects 45.8 ms, about one command interval, with only a 3.5% pull-scale increase. These are effective input-path estimates; the data do not uniquely separate physical servo lag from timing, routing, and other modeling errors.

## Mechanism ablations

The first selected fit was replayed with rotor inertia, suspension terms, and pull scale separately restored to baseline, plus a no-lag case. These are post-selection diagnostics; no alternate model is selected from validation results. Suspension changes contribute most to the improved combined fit; rotor inertia and effective pull also contribute. Removing the fitted tiny lag changes the result negligibly. Detailed training/manual/non-manual scores are in `ablations.json`.

## Limits and decision

**The constrained fit is a substantially improved coupling candidate, close to but not meeting all predeclared gates.** Non-manual checks pass: combined score improves 35.5%, pitch/yaw phase error drops from 80.3 to 29.0 degrees, mean local score improves 1.8%, and yaw-rate RMSE increases only 1.7%. Manual checks improve 32.6% overall and phase error drops from 108.7 to 43.2 degrees, but yaw-rate RMSE increases 6.5%, exceeding the frozen 5% limit. The aggregate manual local score increases 4.7%, within its separate limit. This is a modest manual waveform tradeoff, not evidence that the coupling improvement is worthless. We retain the candidate without silently relaxing the threshold or overwriting production parameters.


This experiment tests an existing coupled mode, rather than adding an artificial oscillator. The observed pitch/yaw relationship comes from driven, potentially pilot-coupled motion. Neither spectral coherence nor a fitted damping coefficient is an independent damping-ratio measurement. A larger predicted oscillation can improve amplitude and inter-axis phase while worsening its timing relative to the flight trajectory; both comparisons matter.

The numerical gate decisions above apply to these bounded candidates and these data, not to the viability of the model architecture. No browser runtime or controller tune is overwritten by the study. Any accepted follow-on candidate must still be evaluated for control-system stability and performance before deployment.

A numerical check doubling Rust substeps from 32 to 64 changes rate RMSE by less than 6e-8 deg/s and phase error by less than 1.2e-6 degrees in the checked manual and non-manual cases. Frozen replay scores were reproduced after the research import/refactoring changes. `candidate-config.json` records the constrained physical parameters plus the separate experimental actuator preprocessing; it is not a drop-in native configuration and is not promoted.

## Reproduction

```sh
uv run --project research python research/active/manual-coupling-fit-2026-10-18/run.py
uv run --project research python research/active/manual-coupling-fit-2026-10-18/audit.py
uv run --project research python research/active/manual-coupling-fit-2026-10-18/constrained.py
python research/active/manual-coupling-fit-2026-10-18/report.py
```

Uses the Rust Python extension and original flight replay inputs selected by `MICROAGU_REPLAY_DATA`. Analysis needs NumPy/SciPy and plotting needs Matplotlib. Raw replay NPZ arrays are generated locally and ignored; protocol, selection, optimizer traces, validation metrics, source provenance, and figures are versioned.
"""
(OUT / "README.md").write_text(text)
