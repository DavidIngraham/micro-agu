# Low-speed EKF wind / sideslip experiment

AI-assisted implementation and analysis, 2026-10-16. Local research only; no hardware deployment or public simulator publication.

## Result

The change fixes a real low-speed fusion limitation, but is not an efficiency win. Across 16 fresh paired sensorless LQI scenarios, median late TAS error RMS fell from 0.800 to 0.182 m/s and maximum error RMS from 3.340 to 1.618 m/s. Every adaptive run applied every scheduled late sideslip update on core 0. Five runs still reached the speed-integrator limit, unchanged from baseline.

Median within-scenario shaft-power changes were +0.76% AUTO and +8.71% CRUISE; energy per ground distance changed +2.07% and +11.05%. Mean late additional common brake increased from 13.52% to 15.77% AUTO, and 10.86% to 12.50% CRUISE. Maximum late pitch AC RMS stayed below 0.05 degrees. This is evidence of settled behavior in these scenarios, not a general stability proof.

The remaining speed bias is consistent with weak along-track wind observability in straight flight. Zero sideslip supplies a direction constraint, not an independent forward-speed measurement. Residual errors and accumulated controller integrals can sustain brake use even after the estimate improves. The prior sensor-equipped reference remains much better at avoiding common brake. Do not present this change as replacing an airspeed sensor or as a validated efficiency improvement.

Case 1 illustrates why better absolute estimation error need not save power: the old estimate was 1.87 m/s versus 4.38 m/s truth, with zero late common brake. The new estimate was 4.83 m/s versus 3.96 m/s truth, with 26.31% late common brake. The error became smaller but changed to an overestimate, making the controller brake an already slower aircraft. These are different closed-loop trajectories, not a replay comparison at identical states.

See [RESULTS.md](RESULTS.md), [comparison.png](comparison.png), and [comparison.json](comparison.json).

## Implementation

- Plane passes `AIRSPEED_MIN` as minimum normal forward-flight EAS to AHRS once per second. This is operating-envelope information, not a synthetic measurement or trim-speed constraint.
- DAL records it in a new `RFWS` replay message. Existing replay structures and parameter indices are unchanged. Logs without this message retain unspecified speed and the historical 5 m/s policy.
- EKF3 sideslip threshold: `clamp(0.5 * minimum_EAS * EAS2TAS, 1, 5)` m/s. Invalid/unspecified operating speed keeps 5 m/s. Current paraglider defaults specify 5 m/s minimum EAS, giving approximately 2.58 m/s threshold at this site's altitude. Conventional aircraft with threshold capped at 5 retain the old measurement variance.
- Wind activation retains the existing >5 m/s horizontal ground-speed and drag-fusion paths. Slow forward-flight vehicles additionally permit activation after >3 m absolute height change from the pre-arm reference or >5 s of vehicle-reported flight. Existing arming/on-ground and navigation-aiding checks still apply. This avoids requiring positive ground speed in a headwind; it is not a redesign of general flight detection.
- For the slow-speed path, sideslip variance is `0.03 + (max(GPS speed accuracy, EK3_VELNE_M_NSE) / max(forward_TAS, threshold))^2`. The velocity floor is a conservative noise allowance, not a complete propagation of correlated GPS/attitude uncertainty. Wind-state covariance already enters the innovation covariance and is not added again here.
- Existing positive-forward-speed gating, covariance checks, innovation clipping and fusion-state masks remain. Neither the physics model nor controller gains were changed.
- Optional SITL-only `AP_NAVEKF3_LOW_SPEED_DIAGNOSTICS` reports whether the actual beta update was applied, body air velocity, wind states/covariances, innovation, and threshold. It defaults off.

## Evidence and limitations

Both fresh variants flew the same 16 scenario definitions from `../controller-stability-2026-10-16/cases`, eight AUTO and eight CRUISE. Each performs 30 s of flight before controller engagement, then 180 s evaluation; disturbances run from 20 to 100 s. Tracking is evaluated during disturbances; late metrics use 150–180 s; power uses 20–180 s.

This is a scenario sensitivity comparison, not a probability distribution or exact replay of identical estimator states. Startup timing and sensor randomness differ. There is only one fresh flight per scenario/configuration; no confidence intervals or statistical significance claim is warranted. The sensor reference is from the preceding campaign. Power is a quasi-steady shaft estimate, not measured electrical power; ground distance includes the flown path, not net mission progress.

The implementation changes activation, fusion threshold and measurement noise together. This comparison does not isolate each contribution. It does not establish behavior during GNSS loss, unusual launch/landing states, or real flight. Retain the changes for further estimator research rather than promote them as an efficiency fix.

## Validation and reproduction

- Native Plane and Replay builds succeeded.
- `build/sitl/tests/test_low_speed`: three tests passed, covering envelope/atmosphere scaling, fallback/floor behavior, airborne evidence and low-speed noise.
- Diagnostic browser integration test `viewer/tests/integration/ekf-low-speed.ts`: 30 s armed ground hold produced no beta-fusion calls; normal-speed launch then produced applied airborne updates. Launch uses real-time speed as in the campaign harness; accelerating the assisted launch sequence itself was not a valid test setup.
- 32/32 fresh campaign flights completed; each has archived build metadata, full samples and LQI diagnostics. Adaptive runs additionally retain `N3LD` lines.
- TypeScript checking and the viewer production build passed. The final non-diagnostic firmware passed the browser MIMO controller-switching/manual-ownership/steering integration test. The final local firmware restores the original production gains and disables both diagnostic streams; `audit.py` verifies their absence in the WASM and validates the retained 32 runs.

`run.py <variant> [comma-separated IDs]` runs the currently built firmware and refuses to overwrite existing case results. `report.py` compares `baseline` and `adaptive` after the shared `../filtered-brake-2026-10-16/analyze.py <absolute-trial-directory>` analysis. The adaptive directory contains the estimator patch and policy header; its build was against AP revision `010ada6a4f0b26910fc18ecca176d2e65c5ed9d7` plus the existing controller work. Baseline was built before applying the estimator patch, using identical diagnostic controller gains.

For diagnostic builds set `EMCC_CFLAGS=-DAP_NAVEKF3_LOW_SPEED_DIAGNOSTICS=1` when invoking `viewer/tools/build/build_sitl_wasm.sh`. Run `./waf clean` for the WASM board when changing that environment-only flag: Waf does not reliably invalidate objects for it. Rebuild without the flag and verify the final WASM has neither `N3LD,` nor `LQID,` before using a normal build.
