# Matched-reference, sensor-equipped LQI tuning

AI-assisted simulation research, 17 October 2026. This study implements the next
step from the [holistic assessment](../proprioception-2026-10-17/holistic/README.md).
It is not a deployed tune or flight qualification.

## Outcome

Completed **66 scored runs**: 18 training runs, 24 fresh command/gust runs and
24 fresh AUTO/CRUISE mission runs. Twelve preliminary runs with the confounded
`TECS_SINK_MAX` change are preserved separately and excluded. The selected
`responsive` candidate passed the declared validation checks. See
[results and plots](RESULTS.md), [machine-readable summary](summary.json), and
[provenance/restoration audit](audit.json).

- Fresh command/gust median paired aggregate loss: **27.18% lower than matched
  fixed LQI**, **11.41% lower than matched standard**. Worst-case score was 0.553,
  versus 0.744 fixed and 0.693 standard. The direction of aggregate improvement
  persisted across the named task-preference profiles; the updraft-first
  advantage over standard was only 2.46%.
- Updraft recovery improved from 21.17 s to **12.84 s**, but standard remained
  faster at 7.83 s. Updraft altitude peak fell from 3.78 to 3.32 m, versus
  standard's 2.95 m.
- Climb rise time was 8.50 s versus 8.80 fixed and 8.40 standard. Descent settling
  was 13.21 s versus 23.86 fixed and 16.13 standard.
- Fresh mission altitude RMS medians: **2.38 m AUTO / 0.87 m CRUISE**, versus
  standard's 3.94 / 1.94 m. AUTO cross-track was similar; CRUISE cross-track
  remained worse, 0.190 versus 0.053 m. There are only four cases per mode.
- Modeled shaft energy per ground km was essentially unchanged from fixed LQI
  (+0.08% paired median) and 1.48% above standard in command/gust tests.

The remaining tradeoff is important: climb-command peak canopy-relative pitch
rate was **53 deg/s**, versus **22 deg/s** standard. The new tune reduces that
peak relative to fixed LQI, but does not eliminate it. The aggregate's motion
component uses gust-response RMS, so this peak must be considered alongside the
score. Likewise, lower crosswind task loss does not mean lower peak payload
pitch: selected peak pitch remained 20.77 degrees versus 17.51 standard.

This is a better research candidate, not evidence of uniform superiority or an
optimal LQI. A plausible next target is acceleration/jerk shaping of the LQI
height reference to reduce those command-onset spikes; that change was not
silently folded into this gain comparison. The sensor-aware observer and its
assumed noise/delay were held fixed, so this does not isolate the value of adding
the angle sensor to the newly selected gains.

All twelve temporarily changed source/artifact files were restored byte-for-byte.
The governor change is retained as a reviewable patch with original defaults;
the selected gains and exact runtime are under `designs/responsive`. Nothing
was committed, pushed or deployed. Three selection-rule tests, TypeScript
checking, five firmware builds, gain reproduction, actual sensor/governor-use
checks, physics-asset hashes and final restoration audit passed. Figures were
visually inspected.

## Protocol declared before running

- Retain ArduPlane WASM, payload EKF, noisy pitot, the as-built 132 mm geometry,
  Rust 9DOF plant, and the 50 Hz relative orientation sensor (0.5°/axis, 40 ms delay).
- Add configurable LQI height-governor parameters `CP3_HUP`, `CP3_HDN`, `CP3_HTC`.
  Defaults remain 0.5 m/s up/down and 2 s approach time, preserving prior behavior.
  Invalid/nonfinite values make the backend ineligible. No existing parameter
  indices are changed. The separate `governor.patch` contains only this change.
- Test both controllers with 1 m/s up/down reference caps: `TECS_CLMB_MAX=1`,
  `CP3_HUP=1`, `CP3_HDN=1`, plus the isolated standard trajectory cap in
  `standard-reference-only.patch`. Keep `TECS_SINK_MAX=2.5` because it also
  scales throttle feedforward. An initial attempt that set it to 1 changed the
  controller itself; those preliminary runs are retained but excluded from
  selection and validation. This matches reference rate limits, not every
  aspect of their internal reference shaping. LQI keeps its 2 s approach law;
  standard keeps its existing trajectory smoothing. The airspeed architectures
  remain different: standard paraglider control assumes natural trim speed;
  LQI uses a TAS target capped at 4.4 m/s. No arming checks are bypassed.
  This study matches longitudinal rate limits only. The existing LQI heading-rate
  cap of 10°/s and standard `PG_TURN_RMAX=25` are retained; mission path results
  therefore compare the complete existing lateral implementations, not equal
  lateral reference governors.
- Settle at 60 m before +10 m and -10 m commands, then isolated updraft and
  crosswind pulses. Keep all acquisition, overshoot, recovery and motion metrics.
- Training: the three previously characterized physics cases 1, 3, 13 only.
  Fresh validation: eight new parameter/weather seeds 40–47, generated before
  candidate evaluation. They are not read for selection. Validate command/gust
  response and AUTO/CRUISE mission behavior separately.

## Candidates and objective

All candidates use the same archived linearization, trim, scales, equilibrium
feedforward and augmented observer. Re-solve the discrete Riccati equation and
anti-windup inverse for each cost; retain matrices, local poles and exact firmware.
No model coefficients are fitted to these runs.

| Candidate | Pitch-rate penalty | Height-output multiplier | Height-integral weight | Throttle effort | Extra vertical-velocity weight |
|---|---:|---:|---:|---:|---:|
| fixed | 30 | 1 | 3 | 1000 | 0 |
| damped | 90 | 1 | 3 | 1000 | 0 |
| balanced | 90 | 2 | 6 | 700 | 0 |
| responsive | 90 | 4 | 12 | 500 | 0 |
| vertical | 90 | 2 | 6 | 300 | 4 |

`score.py` declares dimensional squared losses for four equally weighted tasks:
acquisition (rise time, settling, integrated error), descent (settling, overshoot,
integrated error), updraft (height peak, recovery, integrated error), and crosswind
(pitch peak, recovery, integrated altitude error). These form 65% of the score;
relative pitch-rate RMS during gust responses is 20%, actuator activity 10%, and modeled shaft
energy per ground distance 5%. All scales and coefficients are explicit in code.
They are engineering preferences, not certified tolerances.

Reject incomplete/unsettled/unrecovered runs, truth envelope violations,
prolonged upper-brake saturation and appreciable persistent late pitch motion.
Reject a candidate if any task's median loss or energy worsens more than 20%
relative to matched-limit fixed LQI. Rank remaining candidates using 50% mean
plus 50% worst-case score across training scenarios; require at least 5%
improvement. Preserve the fixed tune if no candidate qualifies. Standard is a
comparison, not a training target. Persist selection before any validation run.

Held-out acceptance keeps the same operational gates and 20% median task/energy
regression checks versus matched fixed LQI. Mission acceptance additionally
requires all flights to complete, no envelope/saturation/late-motion failure,
and no >20% median altitude or cross-track error regression per mode, using a
1 m denominator floor to avoid magnifying tiny path errors. A validation failure
does not trigger reselection using those validation results. Step peak relative
angular rates are reported separately even though the declared motion component
uses gust-response RMS.

## Reproduction and isolation

`campaign.py` snapshots every source/artifact it temporarily changes and restores
them in `finally`, preserving concurrent edits. It refuses to reuse an existing
backup or overwrite prior run results. Use a fresh study copy for reproduction.
The existing environment is `/home/david/envs/venv-ardupilot/bin/python`; the
orchestrator provides the Node/emsdk paths. It does not commit, push or deploy.

`protocol.json`, `selection.json`, per-phase metrics, raw JSON, build logs,
candidate runtime hashes, the governor patch and restoration report provide
provenance. `test_selection.py` checks that a low aggregate cannot hide a task
regression or failed run. Actual LQI diagnostics verify pitot use, sensor mode,
and both ±1 m/s governor limits during the experiment.

## Follow-up

The [transition and sensing study](../lqi-transitions-2026-10-17/README.md) addresses this campaign's command-onset motion and incomplete motion objective. It validates a reference filter with unchanged feedback gains on fresh cases, and compares ideal, delayed/noisy, stressed and full-state sensing. Use that study for the latest controller comparison; this report remains the record of the preceding campaign.

