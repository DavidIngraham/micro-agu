# Filtered common-brake cost: implemented, not selected as the default

AI-assisted implementation and analysis. Existing flight-log calibration and
as-built 132 mm riser spacing are unchanged. No publication or deployment.

## Result

The optional filter penalizes sustained common brake while leaving short damping
inputs less expensive. It uses a three-second time constant, the actual allocated
command, engagement initialization and output matching. The physical observer
remains 16 states; the LQI gains now include three integral states and one brake
memory state. Zero memory cost reproduces the previous controller gain.

We ran 40 WASM SITL/EKF flights: 16 zero-cost references, eight weight-1000 screens,
and 16 weight-3000 cases. Every run completed without an envelope stop. Weight
1000 failed the initial tracking screen; weight 3000 passed, narrowly for AUTO
cross-track error in the expanded comparison. Neither demonstrates an energy win.

Full 16-scenario comparison (eight AUTO, eight CRUISE per controller):

| Metric | AUTO zero / weight 3000 | CRUISE zero / weight 3000 |
|---|---:|---:|
| Median disturbed altitude RMS, m | 3.552 / 3.682 | 2.018 / 1.041 |
| Median disturbed cross-track RMS, m | 5.332 / 6.639 | 0.355 / 0.000 |
| Maximum late pitch AC RMS, deg | 0.039 / 0.126 | 0.041 / 0.041 |
| Mean late additional common brake, % travel | 12.926 / 11.727 | 10.474 / 6.875 |
| Median estimated shaft power, W | 73.027 / 72.836 | 67.784 / 70.923 |
| Median estimated shaft Wh/ground-km | 4.852 / 4.885 | 4.345 / 4.650 |

The stronger cost reduced mean late brake travel by 9.3% relative in AUTO and
34.4% in CRUISE. Ratios of median shaft power changed -0.3% and +4.6%; energy per
ground kilometre changed +0.7% and +7.0%. These percentages are descriptive, not
paired causal estimates or statistically significant effects. Actual initial
estimator states and trajectories differ despite matched scenario/weather seeds.
Ground distance includes the full traveled path, not useful mission progress.
Power is the same quasi-steady shaft estimate as earlier work, not battery power.

The weight-3000 CRUISE pitch recovery remained quiet in every tested case. AUTO
also includes commanded turns; its largest late pitch AC RMS was 0.126 degrees
(case 10). Completion and small late oscillations are not a robust-stability proof.

## What the diagnostics established

`ARSPD_TYPE=0` is inherited from ArduPilot's paraglider defaults. The LQI receives
AHRS synthetic airspeed rather than a dedicated airspeed sensor. Its vector-air
measurement and scalar TAS agree, but can both disagree substantially with truth.

In zero-cost case 6, late estimated TAS was 5.643 m/s versus truth 3.988 m/s. In
case 9 it was 4.992 versus 3.866 m/s. Both used a 4.4 m/s speed target and held the
speed integral at its +50 bound. In case 9 that integral contributed +27.349
percentage points of common brake. Feedforward, state feedback and the other
integrators brought requested common brake to 26.375%; allocation applied almost
the same value. This explains the repeated approximately 26% plateau in these
runs: it is not evidence of an energy-optimal equilibrium or a brief damping pulse.

Other zero-cost runs underestimated TAS and released common brake. This estimator
variation is an important confound in energy comparisons. The current evidence
does not identify the underlying wind-estimator/startup mechanism or prove that
every sustained-brake case has the same cause. Diagnostic command reconstruction
agrees with the logged command to better than 1e-5 normalized brake.

## Decision and next step

Keep the new feature available in synthesis but retain memory weight zero in the
normal generated gains. Rebuild local native/WASM artifacts with diagnostics off.
Do not publish the experimental weights. The normal generator defaults remain
zero common trim, instantaneous common cost 1000 and pitch-rate cost 30.

The next control change should address the confidence and priority of synthetic
airspeed tracking. Investigate wind/airspeed observability, then test a soft speed
band or confidence-aware speed integrator before making energy claims. A filtered
actuator cost cannot correct a biased measurement; an integral can oppose its
steady-state penalty. Keep altitude/heading performance and oscillation recovery
as acceptance requirements rather than trading them away for brake travel.

## Evidence

- `TUNING_RESULTS.md`: the initial eight-case screen, saved before expansion.
- `RESULTS.md`, `comparison.png`: expanded comparison and plots.
- Each trial directory: exact generator/backend/gain snapshots, design report,
  firmware manifest, scenario results, 5 Hz onboard diagnostics and analysis.
- Local firmware snapshots are ignored by Git; manifests and source are retained.
- Validation: synthesis equivalence/filter/physical-boundary tests, six native
  controller tests, native Plane ParagliderMIMO integration, browser MIMO checks,
  viewer unit tests and TypeScript/Vite build. Logs are in `validation/`.

The scenario ranges are sensitivity assumptions, not calibrated probabilities.
Tuning and evaluation cases overlap; this is not independent flight validation.
