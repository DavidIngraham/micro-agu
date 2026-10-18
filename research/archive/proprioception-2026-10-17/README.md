# LQI relative-angle proprioception and airspeed experiment

AI-assisted research, 17 October 2026. Actual ArduPlane WASM, payload EKF, the
existing Rust 9-DOF model and as-built 132 mm geometry stay in the loop. This is a
simulation experiment, not a sensor qualification or a new flight tune.

## Findings

For the broader stability/performance assessment, including twelve additional
settled-flight step/gust runs, see the [holistic follow-up](holistic/README.md).
It identifies the altitude-reference governor and launch-capture confound,
and distinguishes updraft rejection from crosswind recovery. The original
energy-focused findings below are retained as the first campaign's results.

**Completed 92 closed-loop runs. Airspeed sensing helped efficiency; additional
relative-angle sensing did not unlock a substantial improvement in this model.**
See [paired results](RESULTS.md), [all fixed-gain cases](fixed/RESULTS.md), and
the [comparison figure](comparison.png).

- Adding the pitot to unchanged LQI reduced median paired modeled shaft power
  3.86% in AUTO and 11.05% in CRUISE. This was a tradeoff: CRUISE altitude RMS
  error increased 35.89%; AUTO altitude RMS was essentially unchanged.
- Adding the realistic angle sensor to the pitot-equipped LQI changed shaft
  power by −0.00% / −0.09%, disturbed pitch RMS by −0.21% / +0.30%, and altitude
  RMS by +0.73% / +0.91% in AUTO / CRUISE. These are small changes, not evidence
  of a meaningful performance improvement. Scheduling variability is not
  independently estimated at this small effect size.
- The stronger pitch-penalty candidate won the four-case tuning screen by only
  0.18% on the declared combined score. On twelve held-out cases it reduced
  disturbed pitch RMS by 2.89% in AUTO and 1.92% in CRUISE, with essentially
  unchanged shaft power and slightly worse altitude RMS (+0.29% / +0.84%).
  This gain change has not been tested with airspeed alone, so its benefit must
  not be attributed to proprioception.
- With the same pitot, the standard controller remained more efficient than
  fixed-gain sensor-equipped LQI: median paired LQI shaft power was 6.20% higher
  in AUTO and 2.21% higher in CRUISE. Tracking was mixed: LQI altitude RMS was
  68.29% worse in AUTO and 17.70% better in CRUISE. These are conditional scenario
  comparisons, not a universal controller ranking.
- Direct ideal angle/rate substitution entered a poor operating point with
  nearly continuous full braking and recurring large pitch excursions. Its
  late pitch RMS was about 2.5–3.0° in the four diagnostic cases, versus about
  0.03° with the sensor-aware observer. The traces show saturation and excursions;
  that RMS alone should not be interpreted as a measured unstable pole or a
  continuously sustained sinusoidal oscillation. See [raw-feedback traces](raw-feedback.png).

The angle-sensor result is conditional on a controller/plant sharing the same
model family, with coefficient perturbations. Real canopy mechanics omitted
from that family could make proprioception more valuable than these tests show.
This experiment does not establish the global performance ceiling of LQI.

All 92 runs completed without the harness's envelope/ownership/altitude stop.
The raw-feedback configuration is nevertheless unacceptable by its saturation
and tracking behavior. Three sensor/frame tests, TypeScript checks, native
firmware builds, gain-equivalence checks, actual sensor-use checks and the
native-torque power audit passed. All ten temporarily changed source/artifact
files were restored byte-for-byte; `restored.json` records verification.
Exact firmware was archived for every run. No model/tune was promoted or published.

## Question and comparisons

Does a calibrated airspeed sensor plus relative canopy orientation sensing improve
the current LQI? Can modest retuning make useful additional gains?

The fixed-gain comparisons use exactly the existing controller matrices. The
generated diagnostics flag and optional observer matrices do not change those
feedback gains. `audit.py` checks the matrices against the original header.

| Variant | Airspeed | Canopy feedback | Scope |
|---|---|---|---|
| sensorless | Existing estimate | Existing observer | All 16 cases |
| airspeed | Noisy SITL pitot | Existing observer | All 16 cases |
| sensor | Noisy SITL pitot | Noisy/delayed relative angles, augmented observer | All 16 cases |
| standard | Noisy SITL pitot | Standard controller | All 16 cases |
| angles | Noisy SITL pitot | Direct ideal angle replacement | Four diagnostic cases |
| full | Noisy SITL pitot | Direct ideal angles and relative rates | Four diagnostic cases |

The direct replacements are diagnostic ablations, **not guaranteed performance
upper bounds**: replacing an observer's outputs also changes the effective
feedback dynamics. In particular, raw rate substitution can expose feedback that
was previously suppressed by the observer. The physical-sensor approximation
uses an augmented observer instead.

## Sensor model and coordinates

- Airspeed: existing `ARSPD_TYPE=100` SITL pressure sensor, enabled for AHRS use;
  matched ratio 1.99, known zero offset, and the default 2 Pa pressure noise.
  TAS is converted to EAS before entering the JSON pitot interface. Installation
  errors, pitot misalignment, bias drift and icing are not modeled.
- Relative angle sensor: all three components of canopy-to-payload relative
  orientation, sampled at 50 Hz, 0.5° independent Gaussian rotation-vector noise
  per axis, and 40 ms transport delay. These are assumed device capabilities.
  A suspension-line angle sensor need not measure the same physical quantity.
- Payload attitude, gyro, position, velocity and wind remain onboard estimates.
  No truth is supplied to the EKF. The realistic angle case does not consume the
  simulator's relative-rate channel; rates remain estimated by the observer.
- Quaternion transport is scalar-first. The LQI chart uses
  `log(q_relative_trim^-1 * q_payload^-1 * q_canopy)` for its three articulation
  coordinates. Its canopy rates are **canopy-frame absolute angular velocity**,
  not relative rates. The diagnostic rate case reconstructs these from relative
  rate plus onboard payload gyro, then rotates into canopy axes and removes trim.
- The sensor-aware observer has twelve measurements: the existing nine plus
  relative angles. Its Kalman gain is synthesized using the original model and
  the stated angle-noise scale. The controller's feedback gain remains unchanged
  in the fixed-gain campaign. Delay is not explicitly augmented into this observer.

Tests independently check quaternion ordering, heading invariance, rate-frame
conversion, seeded noise and delay. Campaign diagnostics check actual sensor use
and selected feedback mode, not just configured parameters.

## Campaign and tuning protocol

The same sixteen retained weather/physics scenarios are used: eight AUTO and
eight CRUISE. A run launches in AUTO, waits thirty seconds, selects the task and
controller, settles twenty seconds, applies wind/turbulence/thermal disturbances
for eighty seconds, then recovers in calm air for eighty seconds. Log truth is
used for evaluation only. Safety screening retains envelope, ownership, finite
state and altitude failures. Completion alone does not establish stability.

These are paired scenarios with fixed seeds, not an independently sampled flight
population. Startup differs with airspeed configuration, and browser scheduling
introduces small timing differences; this is not an identical-state replay.

Retuning uses only cases 0, 1, 6 and 9. The two predeclared candidates are:

- **efficient:** common-brake effort weight 3000 (from 1000), plus brake-memory
  weight 3000 with its existing 3-second time constant; pitch weight remains 30.
- **damping:** pitch/pitch-rate weight 90 (from 30), retaining common effort 1000
  and zero brake-memory weight.

Both candidates use the same realistic angle and airspeed sensor model. The
remaining twelve cases are held out from candidate selection. Results should be
read as a small design exploration, not a global optimum for LQI.

## Metrics and artifacts

Report truth-based oscillation RMS, rate RMS, late recovery, tracking, upper brake
saturation, common brake and modeled shaft energy. Power uses the retained rotor
RPM and native propeller map, rather than the older quasi-steady throttle-to-RPM
approximation. It excludes motor/ESC losses and electronics, so it is **not
battery energy or endurance**. The calculation is checked against the native
Rust torque function.

`fixed/RESULTS.md`, candidate `RESULTS.md` files and `analysis.json` contain
per-case and aggregate results. `LQID` retains controller state, integral and
allocation diagnostics; `LQIP` identifies sensing mode and pre/post correction
canopy channels. Angle/rate estimation errors compare the logged controller
state with the nearest approximately 10 Hz truth sample, so their small residual
errors include sampling misalignment.

The temporary ArduPilot patch adds optional research fields to the existing JSON
socket transport. It does not add a native WASM/shared-memory ABI. The patch,
generated matrices and exact firmware are archived under this directory.
`prepare.py` snapshots every file it changes, including pre-existing local edits.
`prepare.py restore` restores those bytes and verifies their hashes. Nothing is
published; production defaults and the previously packaged simulator are restored.

## Reproduction

Use the WSL development environments already configured for this repository.
`prepare.py` deliberately refuses to overwrite an existing snapshot. Reproduce
in a fresh copy of this experiment directory, or inspect and preserve that
snapshot before another preparation. Never delete an original snapshot to bypass
the guard while its experiment is still installed.

1. Generate the fixed design with `research/archive/legacy-control/design_mimo.py --diagnostics`.
2. Run `prepare.py <design-directory>`, build with
   `viewer/tools/build/build_sitl_wasm.sh`, and archive with `install.py <name> archive`.
3. Run `run.py fixed sensorless,airspeed,sensor,standard 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15`.
4. Run the diagnostic variants `angles,full` on `0,1,6,9`.
5. Install/build each candidate in sequence; never rebuild while a run is active.
6. Run `analyze.py <group> <design-name>` and `audit.py`.
7. Restore using `prepare.py restore`, including after any interrupted campaign.

The runner preserves existing result files instead of overwriting them. A new
firmware or sensor configuration must use a fresh group directory.
