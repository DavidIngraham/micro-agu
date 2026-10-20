# EKF landing results

Existing LQI gains, no pitot or canopy truth. Metrics are sampled before physical ground contact, not after a collision impulse.

| Scenario | Outcome | Spot error (m) | Incoming sink (m/s) | Ground speed (m/s) | Actual flare height (m) |
|---|---|---:|---:|---:|---:|
| calm-feedback | landed | 3.02 | 1.04 | 4.12 | 0.86 |
| cross-feedback | landed | 6.04 | 1.14 | 3.07 | 0.87 |
| dropout-feedback | landed | 1.23 | 0.94 | 3.75 | 0.81 |
| failed-sensor | aborted | — | — | — | — |
| head-feedback | landed | 5.35 | 1.16 | 2.51 | 0.80 |
| slow-sensor | landed | 1.05 | 1.04 | 3.87 | 0.83 |

Successful cases ended with estimated touchdown and a paused simulator. The fully failed sensor must be rejected before final. Any dropout abort is retained, not omitted.

The old raw-range calm feedback trial reached about 1.35 m/s sink. These checks demonstrate the EKF-based path and current worker integration; they are not a statistical head-to-head improvement claim. The truth-height experiment's roughly 0.81 m/s is an ideal-sensing reference, not a hardware guarantee.

The sensor's 50 ms modeled latency still exceeds EKF3's 25 ms range timing assumption. Full canopy collapse, ground rollout and real-flight flare performance are not qualified. See README.md for estimator validity, cancellation, final handoff and endpoint definitions.
