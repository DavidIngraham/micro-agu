# Does sensor-equipped LQI fly better overall?

AI-assisted assessment, 17 October 2026. **It has useful advantages, but the
current implementation is not an overall replacement for the standard controller.**
It trades slower altitude capture and weaker updraft rejection for some better
settled-flight altitude control, less actuator activity, and (with the stronger
pitch penalty) better crosswind response. Relative-angle sensing alone is not
responsible for most of those advantages.

The recommended matched-reference tuning follow-up is now complete:
[66-run tuning and fresh validation study](../../lqi-balanced-2026-10-17/README.md).
The results below remain the evidence for the earlier gains and governors.

This assessment reanalyzed the original 92 runs and added **12 actual
ArduPlane/EKF/Rust-9DOF runs**, using archived firmware without changing gains.
The realistic relative-orientation sensor remains 50 Hz, 0.5° noise per axis,
40 ms delay; the normal noisy SITL pitot and payload EKF remain active.

## Stability comes before the score

Standard, pitot-only LQI, sensor-equipped LQI, and the pitch-penalty retune all
passed the original campaign's new operational screens. The twelve additional
flight runs completed. No applied full-brake episodes occurred in those normal
configurations. Both controllers' late relative pitch motion was small in the
original suite (worst band RMS 0.031° standard versus 0.023° sensor-equipped LQI).
These results show bounded, recovering behavior in the tested cases, not a
proof of stability or a measured gain/phase margin. They do not establish a
decisive stability advantage for either controller.

The four previously tested raw angle/rate-substitution diagnostics fail the new
screen because of prolonged upper-brake saturation and large late motion. They
are not evidence for an attainable full-state-control performance ceiling.

## The broader index changes the interpretation

The [retrospective scorecard](REPORT.md) uses fixed dimensional scales, separate
stability gates, tracking/recovery/internal-motion/actuator/energy components,
and coefficients 35/30/20/10/5%. It retains every run, worst cases, worst-quarter
averages, individual metrics, and weight/scale sensitivity. Its scales are
provisional engineering preferences, not certified limits. The new index was
defined after the campaign and has not been used to select or optimize gains.

For the twelve cases held out of the earlier tuning screen, the retuned LQI's
median paired score was **81% worse in AUTO and 23% better in CRUISE**. Removing
energy from the index did not change which controller won each case. The retuned
LQI won two of six AUTO cases and five of six CRUISE cases.

However, AUTO contained an important confound: at weather onset the retuned LQI
was still a median **12.47 m below target**, versus **2.70 m** for standard.
The score therefore includes altitude acquisition, not just gust rejection.
Using 60–100 s for the tracking/internal-motion components instead of 20–100 s
changes the retuned LQI comparison to **20% better AUTO / 44% better CRUISE**.
That sensitivity is a finding, not permission to discard the acquisition penalty.
Mission phase and thermal exposure also differ as trajectories diverge.

Later-window altitude RMS medians are 0.73 versus 1.23 m in AUTO and 0.93 versus
1.99 m in CRUISE, retuned LQI versus standard. LQI's path tracking remains worse.
The original pitch-penalty retune improves some rate/recovery and worst-case
metrics, but does not consistently improve the median new score over fixed-gain
sensor-equipped LQI.

## Settled tests separate command response from launch

The [supplemental experiment](transients-valid/REPORT.md) starts in settled
CRUISE at 60 m, then applies +10 m and -10 m altitude commands, a five-second
1 m/s updraft, and a five-second 2 m/s crosswind. Three physics configurations
were used for each of four controller/sensing configurations. Values below are
medians over the three cases; raw trajectories and worst cases are retained.

| Response | Standard + pitot | Fixed LQI + both sensors | Retuned LQI + both sensors |
|---|---:|---:|---:|
| +10 m climb, 10–90% time | 8.4 s | 16.0 s | 16.1 s |
| -10 m descent, settling time | 56.9 s | 21.9 s | 20.9 s |
| Descent overshoot | 0.88 m | 0.00 m | 0.30 m |
| Updraft peak height excursion | 2.95 m | 3.80 m | 3.83 m |
| Updraft recovery after removal | 7.9 s | 21.4 s | 21.5 s |
| Crosswind peak pitch excursion | 17.0° | 20.7° | 9.8° |
| Crosswind recovery after removal | 5.8 s | 5.6 s | 2.6 s |

Step settling requires ten seconds within 0.5 m of the requested height and
relative pitch-rate RMS below 2°/s. Gust recovery requires five seconds within
0.5 m of pre-gust height and the same rate threshold. These first sustained
in-tolerance intervals are not modal damping-ratio estimates. Gusts run in
sequence: their pre-gust height offsets are retained and excursions are measured
relative to the preceding five seconds, not assumed zero.

The slow altitude acquisition has an identifiable implementation contribution:
the [archived LQI reference governor](../AP_CustomControl_LQI.cpp.experiment.txt)
limits climb and descent to **0.5 m/s**. The standard paraglider controller's
configured limits are 1 m/s climb and 2.5 m/s descent, with its own smoothing.
The LQI 10–90% response of about 16 seconds for an 8 m interval closely follows
that governor. This is not a clean comparison of feedback gain quality alone.

There is also no shared active airspeed-tracking objective. Standard uses
TECS_PG_ENABLE=1, whose paraglider branch controls height with throttle and
pitch-rate damping and assumes natural airspeed. LQI actively uses a speed
target capped at 4.4 m/s TAS. Actual mean TAS was nevertheless close in these
tests: 4.380 m/s standard, 4.357 fixed sensor-equipped LQI, 4.376 retuned LQI.
Ordinary TECS speed-error telemetry reports an inactive zero target in this
branch and must not be used as a performance metric.

An attempted common low-speed parameter setup was rejected at arming because
the firmware requires AIRSPEED_MIN >=5. Those nine preflight attempts are
retained separately and are not counted as flight tests. No arming check was
disabled. The twelve valid runs use the original legal 5/5/7 m/s cruise/min/max
parameters; the controller-specific governor remains part of each implementation.

## What the additional sensing buys

With fixed gains, adding relative angles to a pitot-equipped LQI changed the
original suite's new score by only about -0.5% AUTO / -3.0% CRUISE, with most
individual tracking/motion changes near 1%. The isolated updraft and crosswind
responses are also close to pitot-only LQI. Some step results differ, but there
are only three cases, startup/estimator offsets differ, and repeats were not
used to resolve small differences. The pitch-penalty candidate's stronger
crosswind response cannot be credited to proprioception: that gain set has
not been run with pitot alone.

**Conclusion:** there is a promising control tradeoff, not demonstrated overall
superiority and not a measured LQI ceiling. Before a larger optimization, make
the altitude reference governor configurable and compare at matched command
limits; explicitly retain acquisition, descent overshoot, updraft recovery,
crosswind motion, and path tracking as separate objectives. Then tune the
sensor-equipped controller/observer against this expanded suite and validate on
fresh scenarios. The current evidence does not justify a deployment change.

## Artifacts and verification

- [Original-suite report and score definition](REPORT.md), [machine-readable metrics](metrics.json).
- [Settled-flight report](transients-valid/REPORT.md), [metrics](transients-valid/metrics.json),
  [firmware provenance](transients-valid/runs.json), [restoration](transients-valid/restoration.json).
- Analysis: `../holistic.py`, `../transient_analyze.py`; five analytical metric
  tests in `../test_holistic.py` passed. Temporary harness passed TypeScript checking.
- Runner/harness: `../transient_campaign.py`, `../transient_case.ts`. The runner
  intentionally refuses to overwrite prior backups; reruns require a fresh
  output directory. It restores all five temporary viewer files in a finally block.
- No ArduPilot source edits, gain changes, commit, push, or publication. Original
  viewer artifact/source hashes were verified after restoration. Figures were
  visually checked.
