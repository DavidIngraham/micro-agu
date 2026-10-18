# Browser LQI longitudinal expansion

The optional LQI now permits **0.75 m/s climb and 1.0 m/s descent**, previously 0.5 m/s each. The browser now selects LQI on startup; PG + TECS remains selectable. The 35 deg/s turn-reference limit, responsive_softspeed gains, observer, EKF airspeed, 132 mm geometry and 0.7 s climb-reference filter are unchanged. No pitot or canopy truth is fed to the controller.

All 12 initial nominal flights passed, including 1 m/s climbs, but the latter used nearly all of the 60% nominal throttle ceiling. The lower climb limit reserves more actuator authority. Numerical trim failures are not interpreted as physical infeasibility.

Reduced-voltage validation exposed an existing integration defect: voltage-compensated motor feedback was being compared with uncompensated controller requests. This produced false anti-windup residuals and sustained climbing even with the old limits. The firmware now removes the active compensation scale from allocator, observer and TECS tracking feedback. Actual motor output and real saturation remain unchanged.

After the fix, all **14 LQI validation flights**, three targeted voltage tests and eight paired disturbance comparisons passed their frozen gates. Cases include 0/25/35 deg/s turn demand, 20.4–22.2 V, Dryden turbulence, thermal crossings, coefficient uncertainty, actuator slew variation, controller changes and FBWA/CRUISE transitions. One of four classical-controller comparator flights exceeded the quiet pitch-rate RMS gate (3.88 vs 3 deg/s); it remains reported as a failure.

Across eight matched disturbance cases, LQI/PG+TECS ratios were:
- Relative pitch-rate RMS: median 0.653; worst 0.694.
- Relative pitch-rate peak: median 0.529; worst 0.787.
- Modeled shaft energy: median 0.956; worst 0.978.

These are finite simulator tests, not flight qualification or a statistical reliability estimate. Energy excludes electrical losses and servo consumption. The 3.8–4.4 m/s range is a target clamp, not a guaranteed bound on actual airspeed.

[Full campaign and retained failures](../research/archive/lqi-longitudinal-2026-10-17/README.md).
[Machine-readable validation](assets/runtime/lqi-longitudinal-validation.json).
[Complete firmware patch](assets/runtime/lqi-longitudinal.patch), applied to ArduPilot 010ada6a4f0b26910fc18ecca176d2e65c5ed9d7, replaces the earlier feasible-turn patch recipe. Do not stack both patches. Exact source and binary hashes are in build.json.
