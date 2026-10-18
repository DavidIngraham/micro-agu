# Controller pipeline and gain-file study — 2026-10-18

**Decision: publish `refit_damped` under the user-requested practical non-regression criterion.** The study did not establish a repeatable overall improvement. Additional checks support broadly comparable performance at the current operating limits.

## What is implemented

The [pipeline diagram and CLI documentation](../../simulation/crates/sim-python/python/microagu_sim/pipeline/README.md)
cover measured aircraft inputs, sysid, synthesis, local robustness screening,
paired AP/EKF experiments and artifact export. The measured profile now feeds
identification as well as synthesis; its hash travels with new fits. Legacy fits
are explicitly imported with their original Trout Lake provenance.

The ArduPilot implementation is on local branch `feature/lqi-file-pipeline`,
commit `5e5bba8ee1`, based directly on `20a8fb59c8`. Its parent matches all six
controller/allocator source hashes in the packaged simulator's build manifest.
Only the gain-file loader and associated tests/documentation differ. The loader
uses `AP::FS()` and accepts a 280-byte feedback/anti-windup file, bound to the
compiled observer/model context. No observer, sensing or controller layout is
changed by loading a file. Enable `CP3_FILE`, upload to the filesystem root and
reboot. The browser loads the file before startup and requires a positive firmware
acknowledgement. Missing, incompatible or corrupt files cannot silently activate
LQI in required-file mode.

## Experiment

The nominal fit is the earlier direct-response sysid result: inertia scale 0.891,
yaw stiffness 0.820, yaw damping 0.200, pitch damping 1.226, effective pull 1.069,
and brake lag 0.120 s. Damping and lag remain close to their fitting bounds; lag
is an uncertainty case, not a measured servo property. Measured mass remains
1.758 kg with the as-built 132 mm riser spacing. Physical uncertainty values that
were never measured remain explicitly unknown.

Four gain designs were generated: the fitted model with published weights,
increased pitch-rate penalty (90 to 180), softened integral/throttle weights,
and production-model resynthesis. All passed the observer-in-loop local stability
screen across seven frozen model variants. This screen includes hidden brake lag,
but excludes saturation and EKF dynamics and cannot qualify flight behavior.

Initial firmware experiments were exploratory. A source audit found that the
archived reference implementation omitted later turn/longitudinal behavior. Those
runs are **not release evidence**. The final `published-baseline` campaign uses
the audited published source plus the loader, preserves current governor and
turn/anti-windup settings, and compares the most promising damping candidate with
two runs of exactly the same reference gain file. The observer and no-pitot sensing
remain unchanged. Each flight performs climb/descent commands and vertical/lateral
gust recovery in CRUISE with Rust physics and the actual AP EKF.

## Audited firmware results

| Physics | Gains | Pitch-rate RMS (deg/s) | Pitch-rate peak (deg/s) | Crosswind altitude RMS (m) | Shaft power (W) |
|---|---|---:|---:|---:|---:|
| Production | reference | 2.90 | 89.5 | 0.046 | 73.39 |
| Production | reference_repeat | 3.65 | 105.8 | 0.056 | 73.40 |
| Production | refit_damped | 2.86 | 86.5 | 0.158 | 73.41 |
| Fitted + 120 ms lag | reference | 3.95 | 132.8 | 0.061 | 76.57 |
| Fitted + 120 ms lag | reference_repeat | 2.90 | 72.7 | 0.057 | 76.56 |
| Fitted + 120 ms lag | refit_damped | 2.93 | 71.6 | 0.166 | 76.56 |

All six flights completed with controller ownership retained, no envelope violation,
no upper-brake saturation and no persistent late longitudinal oscillation under the
predeclared operational screens. That is limited scenario evidence, not an envelope
or flight qualification.

The apparent fitted-model pitch-rate peak improvement, 132.8 to 71.6 deg/s, is
almost entirely reproduced by running the **same reference gains again**, which
gave 72.7 deg/s. The original improvement gate therefore blocked promotion. A single repeat
is a diagnostic screen, not a statistical confidence interval or a proof of the
source of the variation. Initialization, EKF state and host-timed command/disturbance
alignment need to be controlled or explicitly sampled before claiming a gain.

Climb and vertical-gust altitude RMS did not improve consistently. Crosswind altitude
RMS increased by about 0.11 m; the large percentage change starts from a very small
baseline and should not be mistaken for a stability failure. Power was effectively
unchanged (less than 0.03% difference). These power figures use recorded RPM and the
propeller shaft-power map, **not measured battery power**.

The initial improvement-based decision selected no candidate. The later user-requested non-regression decision and additional mission/envelope checks are recorded below. Further improvement studies should establish
repeatable initial conditions and simulated-time disturbance alignment, then use
multiple reference/candidate repeats with absolute engineering tolerances for small
errors. Model-fit uncertainty remains independent of this harness repeatability issue.

## Reproduction and validation

From the repository root, use the simulation virtual environment and Node 22+ on PATH:

```sh
simulation/.venv/bin/python -m microagu_sim.pipeline run \
  research/controller-pipeline-2026-10-18/protocol-final.json /tmp/new-design-run
simulation/.venv/bin/python -m microagu_sim.pipeline campaign \
  /tmp/new-design-run research/controller-pipeline-2026-10-18/final-training.json \
  /path/to/ardupilot/build/wasm/bin/arduplane.js viewer /tmp/new-flight-run --workers 3
```

The CLI freezes source/input hashes and copies gain files and firmware into the run.
Existing runs are never overwritten. `reference_repeat` is mandatory. Full trajectories
and large firmware binaries remain local; metrics, protocols, gains, design artifacts
and verification records are versioned. The final synthesized gain files are byte-for-
byte identical to those used in the initial study.

Checks passed: 30 Python/decoder tests, 42 viewer unit tests, TypeScript checking,
viewer production build, WASM ArduPlane build, and MatekH743 ChibiOS build with the
experimental custom-controller flags enabled. Valid/corrupt file startup checks pass;
the earlier in-flight rejection test also retained standard-controller ownership.
No physical SD-card bench test or hardware flight was performed. No PR was opened.

Evidence: [final comparison](published-baseline/comparison.json),
[metrics](published-baseline/metrics.json), [build/source verification](verification.json),
[local stability decision](run-verified/decision.json), and
[measured aircraft profile](aircraft.json).

## Deployment follow-up

The user subsequently accepted broadly comparable performance as the deployment criterion, superseding the original requirement for a clear improvement. Both reference and candidate passed 0 and 35 degree/s turning cases with 0.75 m/s climb, 1 m/s descent, 20.4/22.2 V supply, and controller/mode handovers. Altitude tracking RMS differed by less than 1.4%, and peak roll/pitch excursions were nearly identical. Both completed 240 simulated seconds of AUTO mission operation. An initial cruise-only height guard was corrected to exclude the takeoff phase; the reference was rerun.

The candidate is suitable for the browser simulator on this limited practical non-regression evidence. This is not a formal statistical equivalence test or hardware flight qualification. The physical model, observer, sensor configuration and operating limits are unchanged. The new feedback and anti-windup gains load through the verified file interface. See [deployment acceptance](deployment-acceptance.json).
