"""Plot and document the coupled-oscillation investigation."""

import json
from pathlib import Path
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parent
read = lambda f: json.loads((OUT / f).read_text())
sampling = read("sampling-comparison.json")
survey = read("survey.json")
shapes = read("motion-shape.json")
modes = read("model-modes.json")
spectra = read("spectra.json")
fig, axs = plt.subplots(1, 3, figsize=(12, 4))
for ax, start in zip(axs, [1440, 2100, 2760]):
    for key, color in [
        ("raw_50hz", "#17252d"),
        ("antialias_25hz", "#3976be"),
        ("attitude_derived", "#de9e36"),
        ("causal_25hz", "#a088b5"),
    ]:
        d = spectra[str(start)][key]
        ax.semilogy(d["frequency"], np.array(d["psd"])[:, 2], label=key, color=color)
    ax.set_xlim(0.3, 4)
    ax.set_title(f"{start}–{start + 90} s")
    ax.set_xlabel("Frequency (Hz)")
    ax.grid(alpha=0.2)
axs[0].set_ylabel("Yaw-rate PSD ((deg/s)²/Hz)")
axs[0].legend(fontsize=8)
fig.suptitle("The coupled oscillation survives the sampling/attitude cross-check")
fig.tight_layout()
fig.savefig(OUT / "sampling.png", dpi=170)
fig, axs = plt.subplots(1, 2, figsize=(11, 4.5))
for label, color, manual in [
    ("MANUAL", "#de9e36", True),
    ("Other/mixed modes", "#3976be", False),
]:
    r = [x for x in survey if (set(x["mode_counts"]) == {"0"}) == manual]
    axs[0].scatter(
        [x["start"] for x in r],
        [x["gyro_band_rms"][2] for x in r],
        label=label,
        color=color,
        s=20,
    )
axs[0].set_xlabel("Flight time (s)")
axs[0].set_ylabel("1.2–1.9 Hz yaw-rate RMS (deg/s)")
axs[0].legend()
axs[0].grid(alpha=0.2)
axs[1].scatter(
    [x["pitch_relative_to_yaw_phase_deg"] for x in shapes],
    [x["pitch_per_yaw_amplitude"] for x in shapes],
    color="#17252d",
    label="Flight segments",
)
for label, color in [
    ("baseline", "#84929f"),
    ("candidate", "#3976be"),
    ("half_yaw_damping", "#de9e36"),
]:
    row = next(x for x in modes if x["variant"] == label)
    m = min(row["modes"], key=lambda x: abs(x["frequency_hz"] - 1.56))
    a = m["payload_rate_magnitude"]
    axs[1].scatter(
        m["pitch_yaw_phase_deg"],
        a[1] / a[2],
        marker="x",
        s=75,
        color=color,
        label=f"Model mode: {label}",
    )
axs[1].set_xlabel("Pitch relative to yaw phase (deg)")
axs[1].set_ylabel("Pitch/yaw rate amplitude ratio")
axs[1].legend(fontsize=8)
axs[1].grid(alpha=0.2)
fig.suptitle(
    "Observed driven motion versus model eigenvectors — qualitative comparison"
)
fig.tight_layout()
fig.savefig(OUT / "coupling.png", dpi=170)
table = "| Variant | Nearby mode | Damping ratio | Pitch/yaw amplitude | Pitch relative phase |\n|---|---:|---:|---:|---:|\n"
for label in [
    "baseline",
    "candidate",
    "no_rotor_gyro",
    "diagonal_payload_inertia",
    "half_yaw_damping",
]:
    row = next(x for x in modes if x["variant"] == label)
    m = min(row["modes"], key=lambda x: abs(x["frequency_hz"] - 1.56))
    a = m["payload_rate_magnitude"]
    table += f"| {label} | {m['frequency_hz']:.3f} Hz | {m['damping_ratio']:.3f} | {a[1] / a[2]:.2f} | {m['pitch_yaw_phase_deg']:.1f} deg |\n"
text = f"""# Coupled pitch/yaw oscillation investigation

**The 1.5–1.7 Hz component is supported by both logged gyro and attitude motion, exists in MANUAL, and has a gyroscopic-looking pitch/yaw relationship. The Rust model already contains a nearby coupled mode.** The current evidence points toward a mismatch in excitation, damping, or operating-point dependence, rather than proving that another degree of freedom is needed. No simulator parameters or controller gains were changed.

## Sampling and measurement audit

The original extracted IMU instance-0 messages arrive at approximately 50 Hz, ATT and RCOU at 25 Hz. Previous replay preparation takes the latest available sample at each 25 Hz grid point. We compared that stream with native-rate gyro interpolation, anti-aliased 50→25 Hz conversion, and body rates derived from successive ATT quaternion rotations using actual timestamp differences and midpoint timestamps.

Across 1440–1530, 2100–2190, and 2760–2850 s, 1.2–1.9 Hz yaw-rate RMS from native-rate gyro is **13.76, 19.39, and 19.13 deg/s**, versus **13.58, 19.16, and 18.93 deg/s** from attitude-derived rates. Gyro/attitude coherence at 1.5625 Hz exceeds 0.9999 for yaw, with about 6 degrees phase difference. Anti-aliasing changes this band RMS negligibly. The broad peak shifts somewhat between segments: it should not be treated as an exactly fixed 1.5625 Hz oscillator.

Thus the peak was not created by our latest-sample replay resampling. ATT uses the same underlying IMU and EKF, so this is a kinematic consistency check, not independent optical truth. Upstream sensor processing or aliasing before the recorded 50 Hz stream cannot be conclusively excluded from these records alone.

![Sampling comparison](sampling.png)

## Does it require the autopilot loop?

No. Three surveyed windows entirely in logged MANUAL mode (300, 390, 540 s starts) have 1.2–1.9 Hz yaw-rate RMS of **12.28, 14.49, and 20.97 deg/s**. The log identifies firmware `3a2da6cf`; its `ArduPlane/mode_manual.cpp` passes pilot commands to outputs and resets the attitude controllers. Its paraglider mixer maps pilot roll and flare to brakes without an attitude-feedback term. This establishes that an AP attitude loop is not necessary for the component. It does not exclude pilot excitation, pilot feedback, or amplification by AP in other modes.

The 30-second survey covers 300–3330 s where the prior validity mask passes. It is a diagnostic survey, not a separately frozen validation dataset. Mode switches can occur inside windows; only wholly MANUAL windows support the manual comparison. Controller-loop causal attribution cannot be made from input/output coherence alone.

## Motion shape

Across the three manual windows and three representative later windows, pitch-rate amplitude is **0.37–0.47 times yaw-rate amplitude**, with pitch approximately **76–96 degrees behind yaw** at the selected band peak. This is near quadrature, rather than two channels simply duplicating one another through a fixed sensor-axis rotation. It is consistent with a precessional/gyroscopic coupled motion, but not unique proof of that mechanism.

![Coupling and flight-mode comparison](coupling.png)

## What the existing model contains

We solved straight-level relative equilibria at 22.2 V with fixed 6 mm nominal effective common pull, leaving both brake lines engaged. This avoids interpreting a numerical derivative across a slack/one-sided-brake kink. Each reported equilibrium has maximum residual below 1e-4 (actual residuals below 2e-13). A finite-difference Jacobian of the 4 ms Rust flow map gives the small-perturbation eigenmodes. Rotor speed is held at its trim value in the perturbation chart; motor-speed perturbations are excluded from this diagnostic. This is model-only analysis, not a flight damping estimate. Repeating the linearization with a 2 ms flow interval changes the selected baseline/candidate frequencies by less than 3e-8 Hz; see `resolution-check.json`.

{table}

The candidate contains a mode near **1.59 Hz** whose pitch/yaw relationship is closer to the observed one than the baseline mode. Removing rotor angular momentum leaves the nearby 1.77 Hz mode almost entirely pitch motion (yaw magnitude about 0.03 relative to pitch near 1.0). Removing payload inertia cross terms barely changes the coupled mode. In this model, rotor gyroscopic terms are therefore a strong source of the pitch/yaw coupling.

Halving yaw damping alone produces a mode near 1.51 Hz and a phase near -79 degrees. This is a useful sensitivity result, **not a fitted or accepted calibration**. The modal damping ratio actually rises in that comparison because coupled eigenmodes change shape: reducing one mechanical damping coefficient does not necessarily reduce the damping ratio of a selected mode.

Driven motion in flight is not the same as a free-mode eigenvector. At 1.5625 Hz, the model's pure differential-brake forced response has pitch/yaw amplitude 0.149 and phase +178.7 degrees for baseline, versus 0.188 and -162.4 degrees for the candidate. Those differ substantially from the flight association. The flight includes correlated common brake, throttle, pilot/controller feedback, disturbances, and changing trim, so this is not a direct causal transfer-function fit. It shows why matching a natural frequency alone would be insufficient.

## Interpretation and next experiment

1. **Sampling artifact from our 25 Hz preparation:** strongly disfavored by the native-rate and attitude checks.
2. **An AP-only feedback oscillation:** ruled out as a necessary explanation by the MANUAL segments and matching firmware path.
3. **A coupled aircraft response involving rotor angular momentum:** consistent with the quadrature motion and model sensitivities; still a hypothesis for the observed component.
4. **A missing model degree of freedom:** not established. A nearby mode is already present, particularly in the candidate, but its observed excitation/amplitude is not reproduced.

The next useful fit should target the complex pitch/yaw relationship and excitation of this existing mode across operating points, not merely its frequency or an integrated trajectory score. Start with manual segments to reduce automatic-controller confounding, retain commanded versus actual brake-motion uncertainty, and vary rotor angular momentum, yaw suspension terms, and input routing within physically justified bounds. Use held-out segments for validation. Compare common and differential input paths jointly; do not assume every yaw spectral peak is a pure differential-brake response.

The existing logs still do not uniquely identify free-decay damping. We should retain a damping uncertainty range rather than tuning to an assumed value. No production promotion or LQI retuning follows from this investigation alone.

## Reproduction and provenance

```sh
uv run --project research python research/active/coupled-oscillation-2026-10-18/investigate.py
uv run --project research python research/active/coupled-oscillation-2026-10-18/shape.py
uv run --project research python research/active/coupled-oscillation-2026-10-18/modes.py
python research/active/coupled-oscillation-2026-10-18/report.py
```

The raw extracted message directory is resolved alongside `MICROAGU_REPLAY_DATA`. JSON source hashes and sampling cadence are recorded in `source-audit.json`; numerical results, surveyed windows, spectra, and model sensitivities are versioned. Matplotlib is needed only for the report plots. No core simulator, production profile, firmware, or browser runtime is modified.
"""
(OUT / "README.md").write_text(text)
