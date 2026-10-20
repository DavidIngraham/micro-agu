# Simulated lidar through the ArduPilot sensor path

Implemented October 20, 2026. Reusable browser sensor support plus six research
landing cases. No firmware patch, physics coefficient changes, gain changes or
deployment. The published configuration still omits this optional sensor.

## Sensor and transport

The payload-mounted downward beam intersects the flat simulation ground plane.
The reusable model supports a body-frame offset, slant range, minimum/maximum
range, a tilt limit, sampling rate, delay, Gaussian noise, bias, quantization and
seeded independent dropouts. Default experimental values:

| Setting | Value |
|---|---|
| Minimum / maximum range | 0.1 / 40 m |
| Sample rate | 20 Hz |
| Sensor delay | 50 ms |
| Noise standard deviation | 2 cm |
| Quantization | 1 cm |
| No-return probability | 1% per sample |
| Maximum beam tilt | 60° from downward vertical |
| Mount location | Payload state origin |

These generic values are assumptions, not a specification or characterization of
a selected lidar. Surface reflectivity, vegetation, weather attenuation, beam
footprint, multipath and non-flat terrain are not modeled.

The range travels through JSON `rng_1` → existing ArduPilot SITL rangefinder
(RNGFND1_TYPE 100, orientation 25) → MAVLink DISTANCE_SENSOR → landing harness.
No sensor truth bypass is used. The SITL driver labels the MAVLink type unknown;
that does not change the modeled single-beam lidar geometry.

An invalid return is encoded as 41 m, beyond the configured 40 m maximum,
instead of zero or a retained valid measurement. The firmware and sensor limits
must match. The browser's small generated MAVLink dialect previously omitted
DISTANCE_SENSOR; the generator and dialect now include it. The first diagnostic
attempt was rejected by the missing-range guard and is excluded from the final
results. Its local traces remain in ignored `rejected-decoder-runs/`.

`PhysicsConfig.lidar` enables the sensor. `Physics.lidarReading` is diagnostic
provenance on the simulation side; flight logic must use received AP telemetry.
`Simulator.state.rangefinder` exposes the decoded downward range, validity and
reception time for future consumers. Firmware and Rust WASM binaries are unchanged.

## Landing behavior

The prior landing reference generator and flare gains are unchanged. Range is
converted to vertical AGL using **EKF attitude**:

$$h_\text{AGL}=r\cos(\phi)\cos(\theta).$$

Above 10 m, relative barometric/navigation altitude may be used when no valid
range is available. Near the ground, a short gap can be bridged for up to
150 ms from the last valid packet using fused climb rate. The decoder rejects
out-of-range values; messages older than 200 ms are not accepted as fresh.
Beyond the bridging window, the research case stops with
`Lidar unavailable near ground`. It does **not** inject truth AGL.
This is a test guard, not an implemented flight go-around or failure controller.

Truth is used only to generate the physical sensor, establish the test target
and assess outcome. The high-altitude target setup is unchanged from the previous
study. LQI and its observer still use the original sensing configuration; enabling
a rangefinder does not replace the EKF with truth or introduce canopy truth.

AP telemetry time is message publication time, not the physical sample's capture
time. The 50 ms sensor delay is therefore only part of the total delay.
AP sensor update, telemetry scheduling and host transport add latency. The flare
currently does not predict through that delay.

## Results

Four nominal cases complete. The 20% independent-dropout stress case and the
100% failed-sensor case stop at the missing-range guard. All six receive thousands
of AP range messages, including explicit invalid ranges in failed-sensor cases.
See [results.md](results.md) and [summary.json](summary.json).

The calm feedback case reaches the near-ground boundary at about 1.35 m/s sink
and 4.45 m/s ground speed, versus 0.81 m/s and 3.25 m/s in the earlier ideal-AGL
case. The sampled flare onset is at approximately 0.67 m true AGL rather than
the earlier 0.96 m. These single runs are not statistically paired, but the
retained range trace directly shows height lag during the descent.

This establishes that the real sensor/telemetry path materially changes flare
timing. The next experiment should predict current height using measured sink
and an explicitly modeled latency budget, then test noise, bias and burst
dropouts. Increasing feedback gains alone does not correct a late trigger.

The sensor model is reusable and opt-in. The landing controller remains research
only, with the earlier limitations in deep-brake aerodynamics and ground contact.

## Verification and reproduction

Six lidar-specific unit tests cover beam geometry, inverted/oblique rejection,
range limits, mounting offset, sample hold, delay, reproducible noise, dropout,
configuration validation and MAVLink decode. All 48 viewer unit tests pass.
TypeScript and package-boundary checks are included in the final validation.

From the repository root:

```sh
control/.venv/bin/python research/active/landing-lidar-2026-10-20/run.py
control/.venv/bin/python research/active/landing-lidar-2026-10-20/report.py
control/.venv/bin/python research/active/landing-lidar-2026-10-20/pack.py
control/.venv/bin/python research/active/landing-lidar-2026-10-20/validate.py
```

The harness reuses the previous study's `guidance.ts`. `protocol.json` pins the
selected artifacts and sensor adapter source. `evidence.tar.gz` retains all six
final raw traces, range packets and AP logs; its manifest contains checksums.
AI-assisted implementation and analysis.
