# Airspeed sensor substantially reduces persistent braking

AI-assisted SITL study. Thirty-two corrected sensor-on flights completed: the
same 16 scenarios with zero memory cost and with memory cost 3000. AHRS reported
using the airspeed sensor in every captured active-control diagnostic. The EKF,
132 mm geometry, flight-log calibration and force model remain unchanged.

## Sensor effect with zero memory cost

Compare against the retained 16 sensorless zero-cost runs:

| Metric | AUTO sensorless → sensor | CRUISE sensorless → sensor |
|---|---:|---:|
| Median late TAS error RMS, m/s | 0.298 → 0.113 | 0.865 → 0.112 |
| Mean late common brake, % travel | 12.926 → 1.645 | 10.474 → 0.296 |
| Maximum late pitch AC RMS, deg | 0.039 → 0.043 | 0.041 → 0.039 |
| Median disturbed altitude RMS, m | 3.552 → 3.698 | 2.018 → 1.644 |
| Median disturbed cross-track RMS, m | 5.332 → 4.576 | 0.355 → 0.411 |

Across all 16 runs, the largest absolute late mean airspeed bias fell from
2.936 to 0.014 m/s. Late speed-integrator saturation fell from four cases to zero.
The substantial reduction in common brake occurred without retuning LQI gains.
This supports the diagnosis that synthetic-airspeed errors, rather than only
actuator cost, were driving the excessive steady braking.

Median within-scenario energy changes, using the same quasi-steady shaft estimate:

- AUTO shaft power -2.82%, shaft Wh/ground-km -4.64%; both improved in 8/8 cases.
- CRUISE shaft power -1.14%, shaft Wh/ground-km -6.94%; energy improved in 5/8
  cases, with individual energy changes ranging from -21.20% to +2.71%.

Tracking is mixed: AUTO altitude RMS increased slightly while cross-track RMS
fell; CRUISE altitude RMS fell while cross-track RMS increased by 0.056 m.
All cases completed without an envelope stop, and late pitch motion remained
small. This is regression evidence, not a robust-stability proof.

## Filtered penalty with the sensor

Both sensor-on weights completed all scenarios with zero late speed-integrator
saturation. Weight 3000 changed median within-scenario shaft power by -0.08% AUTO
and -0.12% CRUISE, and energy per ground-km by -0.41% and -0.21%. These changes
are too small to establish a useful gain in this campaign. Mean late common brake
was 1.566% AUTO and 0.968% CRUISE, compared with 1.645% and 0.296% at zero weight.
Keep the optional penalty disabled by default.

## Transport correction found during the experiment

Initial four-case screens for each weight are retained under `sensor-zero` and
`sensor-3000`, but excluded from the comparison above. They exposed an additional
positive airspeed bias of roughly 0.13 m/s near the simulated flight altitude.

The browser emitted true airspeed in the JSON `airspeed` field. ArduPilot's
`libraries/SITL/SIM_JSON.cpp` assigns that field directly to `airspeed_pitot`;
`AP_HAL_SITL/sitl_airspeed.cpp` interprets it as equivalent airspeed when generating
pressure. AHRS then applies EAS-to-TAS conversion. Sending TAS therefore applied
the altitude correction a second time.

The browser adapter now converts TAS to EAS using absolute altitude and the same
tropospheric atmosphere as `AP_Baro_atmosphere.cpp`. The corrected series are
`eas-zero` and `eas-3000`. This changes the sensor transport, not aerodynamic
forces or the truth trajectory calculation. Unit tests use ArduPilot's reference
density values at -4000 and 6000 m; integration checks cover nonzero home altitude,
wind, turbulence and the existing physics parity fixtures.

## Limits and validation

The sensor is a calibrated SITL pressure sensor, with default 2 Pa noise enabled,
known zero offset and matched pressure ratio. Installation angle, propwash,
calibration drift, blockage and real hardware performance are not established.
The JSON feed represents airspeed magnitude, without a separate pitot alignment
model. A physical sensor at this low flight speed still needs validation.

Scenarios and weather seeds match the earlier campaign, but startup, estimator
states and trajectories differ; these comparisons are not identical-state replays
or statistical confidence intervals. Ground distance includes all traveled path.
Shaft energy is not electrical battery energy or a measured endurance result.
The native Python JSON adapter also has a TAS assignment; it was not used or
corrected in these browser trials and needs the same contract review before
native sensor comparisons.

Validation includes 42 viewer unit tests, TypeScript/Vite build, native/WASM
physics parity through 1200 airborne and 5000 surface steps, the 32 sensor-on
flights, diagnostic/source audit and the normal browser MIMO integration test.
The eight pre-correction sensor screens are preserved separately.

The normal firmware was restored with zero memory weight and diagnostics off.
Sensor overrides are confined to research case files. Browser assets are rebuilt
locally; the published simulator and its default sensor configuration are unchanged.
See `RESULTS.md`, `comparison.png`, `comparison.json`, per-case logs and snapshots.
