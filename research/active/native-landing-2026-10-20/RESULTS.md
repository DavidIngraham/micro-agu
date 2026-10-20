# Native AUTO landing results

Each case uses packaged ArduPlane WASM, its EKF, and the Rust 9-DOF model. No landing RC guidance runs in JavaScript. Truth is used only for setup and touchdown evaluation.

| Case | Sink at contact (m/s) | Ground speed (m/s) | Spot error (m) | LQI approach |
|---|---:|---:|---:|---:|
| calm | 0.96 | 2.65 | 4.88 | 100.0% |
| headwind | 0.95 | 1.02 | 3.00 | 100.0% |
| crosswind | 0.93 | 2.70 | 5.14 | 100.0% |
| dropout | 0.96 | 2.63 | 5.60 | 100.0% |
| failed | 1.09 | 2.36 | 4.22 | 100.0% |
| slow | 0.92 | 2.75 | 3.23 | 100.0% |
| mission | 1.00 | 2.86 | 12.19 | 100.0% |
| baseline | 0.97 | 2.60 | 15.21 | 0.0% |

All cases stayed in AUTO; native final disengaged LQI, and brake flare had zero commanded throttle. Playback ended after fused near-ground/low-speed confirmation. This is not a statistical campaign, a flight qualification, or a validated ground-roll simulation.

The failed-lidar case uses the normal AP landing-height fallback. The simulator endpoint uses fused altitude above the known mission field when HAGL is unavailable. It does not assume zero AGL.

Initial 100%/s brake application at 1.05 m produced 1.71 m/s sink in the long mission. The selected simulator settings use 300%/s and 1.3 m, with unchanged LQI gains. These are airframe-specific experimental flare settings.
