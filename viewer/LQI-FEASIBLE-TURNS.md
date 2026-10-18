# Browser LQI feasible-turn release

AI-assisted implementation and validation, 2026-10-17. This is a browser simulator controller, not a qualification for flight hardware.

The optional LQI controller uses a nonlinear steady-turn trim table, actuator-specific anti-windup and observer equilibrium recentering. Its feedback and observer gain matrices remain those of `responsive_softspeed`. It retains EKF airspeed, the onboard canopy observer and the as-built 132 mm physics model. No pitot or canopy truth input is enabled. PG + TECS remains the default controller.

The browser configuration sets `CP3_RMAX=25`, `CP3_TMAP=1`, `CP3_AWMODE=1`, and `CP3_OBSTRIM=1`. The release firmware rejects rate-limit parameters above 25 deg/s. An engagement-only two-second smooth blend from previous applied controls prevents a sharp brake transient while the observer settles. Actual allocated outputs continue to feed the observer during the blend. The existing travel/slew allocator remains active.

## Evidence and interpretation

Twelve paired disturbed cases at 25 deg/s compared the candidate with the preceding LQI at the same commanded rate. Median paired tracking RMS improved 28.7%, altitude RMS 32.4%, and modeled shaft power 2.8%; all twelve cases improved on those measures. Relative pitch-rate RMS improved in eight cases; peaks were mixed. These comparisons precede the engagement-blend change, which affects only the first two seconds after engagement. They do not compare against PG + TECS, prove stability margins, or establish flight safety.

The original absolute turbulence screens remain recorded as failed. Their binary result obscured relative improvements and conflated ongoing forcing with recovery. All weather-off recovery cases settled. No new physical safety boundary is inferred from that finding. The 40 deg/s experiments are not deployed.

Final-binary qualification covers paired PG + TECS/LQI controller and mode transitions, including FBWA, CRUISE and AUTO, plus nominal 10 and 25 deg/s reversals and recovery. The transition protocol was specified before execution. The initial release candidate failed a handover comparison; the revised blend was checked against the same protocol and additional fresh transition cases. Full summaries and exact firmware hashes are bundled in [the validation artifact](assets/runtime/lqi-feasible-turns-validation.json).

All 16 final-binary flights passed. Transition comparisons required candidate peak relative pitch rate no greater than 1.2 times the matched baseline plus 2 deg/s, and altitude excursion no greater than 1.2 times baseline plus 1 m. Ownership had to match within five seconds; finite-state, 15 m altitude-floor and 60/45 degree roll/pitch operational guards remained active. These are explicit simulation regression tolerances, not calibrated physical limits. Calm turn tests retained the original tracking, motion and recovery checks; only the research-diagnostics requirement was omitted for the telemetry-free binary.

## Reproduce the firmware

Use ArduPilot base `010ada6a4f0b26910fc18ecca176d2e65c5ed9d7` and apply [lqi-feasible-turns.patch](assets/runtime/lqi-feasible-turns.patch) in a clean checkout. This is a complete patch against that base, replacing the older `lqi-filter-tune.patch` recipe; do not stack both patches. Build using `tools/build/build_sitl_wasm.sh` as described in `SIMULATOR.md`.

The patch includes the preceding reference filter and tune as well as the new changes. It contains no research printf telemetry. The source patch was reapplied in a temporary directory and checked byte-for-byte against the compiled source. Gain hashes and release hashes are recorded in [build.json](assets/runtime/build.json). Separate experiments in the local ArduPilot checkout are not the source of this packaged release.
