# Micro-AGU: Data and Methods

[Flight story](flight-testing.md) · [Design](micro-agu-design.md) · [Steering](paraglider.md) · [Throttle and pitch](longitudinal-observer.md)

This appendix supports the four project chapters. Flight measurements, the initial simulation tuning, and later controller/model experiments are separate evidence sets.

## Current simulator and source ownership

[Fly the browser simulator](https://davidingraham.github.io/assets/microagu-viewer/simulator.html).
The [reusable Rust 9-DOF kernel and generated Python/TypeScript bindings](../simulation/README.md)
live in `simulation/`; [study drivers, fitting and historical notes](../research/README.md)
live separately in `research/`. ArduPilot retains the lightweight articulated built-in model,
firmware controllers and its standard JSON socket transport. The browser connects
the Rust physics through a local Emscripten socket adapter owned by microAGU;
no custom native WASM physics interface or network server is needed. Earlier commit-pinned links
below remain historical evidence, rather than links to the active implementation.

The current browser uses **132 mm as-built riser spacing**, routing and an updated
bounded damping fit using only the existing Hood River and Trout Lake logs. The
[fit report and reproduction method](../research/archive/as-built-simulator-2026-10-15/README.md)
record input hashes, fit/check windows, unchanged brake authority and limitations.
Held-out mean pitch/yaw rate RMS changes from 19.40/26.46 to 17.47/23.71°/s;
the damping search reaches its upper bound and course prediction remains weak.
This is a model refit, not a new aircraft controller tune or independent flight validation.

The browser starts on the ground, performs assisted overhead inflation and hand
launch, then enters its default AUTO mission after a FBWA climb above 12 m.
Flat-plane contact, inflation time, launch motion and friction are explicit
assumptions: the existing logs do not measure canopy inflation or identify those
constants. The model does not simulate cloth pull-up, terrain or canopy collapse.

The browser's weather panel adds repeatable correlated gusts and an adjustable,
stationary Gaussian thermal column to the mean wind. These are synthetic scenarios,
not weather identified from the flight logs. The combined wind drives both the
Rust dynamics and simulated airspeed/wind measurements. AUTO/LOITER steering nudges
use native ArduPlane stick mixing; the sprung FBWB/CRUISE altitude lever uses its
pitch-stick climb command while the autopilot controls power. See the
[browser simulator guide](../viewer/SIMULATOR.md) for controls and weather limits.

## Design record

The design record includes pages 1, 185, and 323 of `Paraglider.pdf`, SolidWorks files, build photographs, and flight logs. The [design evidence](assets/micro-agu/design-evidence.json) records assembly structure and source hashes. CAD previews were extracted with [SWFormat](https://github.com/KenM76/swformat).

## Flight measurements

The figures use the January 3 and February 15 onboard logs. Height is relative to home, and the ground track is relative to the plotted segment’s starting point. Energy is battery-side electrical energy, calculated from current and voltage and checked against the onboard counters. The AUTO endurance row is part of the full Trout Lake flight.

The force diagram explains the mechanism identified afterward in the articulated model; loaded CG and inertia remain to be measured. The February 15 firmware reports `3a2da6cf`, while its parameter names match later source changes. The inspected implementations agree on the damper sign, but the version string does not identify the exact compiled tree. Filter behavior also matters: the inspected code zeros the signal at zero cutoff, applies positive cutoff changes on reset, and updates at 50 Hz using the throttle-loop timestep.

I reviewed six onboard logs and 54 telemetry logs with their raw companions. A February 22 telemetry recording switches from hardware to SITL; that simulated section is excluded here. The [flight evidence](assets/flight-testing/flight-evidence.json) records source hashes and comparison windows. [Hood River](assets/flight-testing/hood-river-plot-data.csv.gz) and [Trout Lake](assets/flight-testing/trout-lake-plot-data.csv.gz) overview data are sampled at 5 Hz; pitch-rate comparisons use the 25 Hz analysis.

The simulation figures use the Mission Planner SITL recording beginning February 14 at 13:43:43 PST, which continues into February 15. It is kept separate from the aircraft flight data. Recorded parameter values and timestamps are preserved in the [simulation tuning history](assets/flight-testing/sitl-lateral-tuning-history.json).


The [endurance evidence](assets/flight-testing/endurance-evidence.json) specifies energy and capacity windows. The pilot confirmed 6 Ah total capacity, accurate Trout Lake capacity estimates, an unchanged configuration during the long flight, and landing because testing was complete.

### Longitudinal reconstruction

Local throttle–climb fits use non-overlapping 10 s windows, MANUAL or FBWA throughout, throttle standard deviation below 1.5 percentage points, and finite EKF climb data. Climb is negative EKF down velocity, checked against height slope. Turns and vertical air motion remain in the samples. Adjacent windows are not independent repetitions; these are local steady-response fits rather than dynamic transfer functions or extrapolations to idle/full throttle.

Hood River supplied 20 windows: climb ≈ 0.05097 × throttle[%] − 1.809 (R² 0.85). Early Trout Lake supplied 13: climb ≈ 0.06257 × throttle[%] − 2.673 (R² 0.88). The pilot confirmed that the 1.8 m/s climb and 3 m/s sink settings came from Hood River analysis. The selected steady windows do not reproduce those original limit measurements.

- [Longitudinal parameter ledger](assets/flight-testing/longitudinal-parameter-ledger.csv)
- [Hood River fit inputs](assets/flight-testing/2026-01-03-steady-fit.json)
- [Trout Lake fit inputs](assets/flight-testing/2026-02-15-steady-fit.json)

### Lateral flight measurements

The long Trout Lake log contains no roll-gain changes. Loiter radius changed 60 → 40 m at boot 828.69 s in LOITER, then 40 → 30 m at 2349.57 s in GUIDED. The pilot confirmed deliberate tighter-turn testing; the second parameter edit alone does not establish a completed 30 m circle.

Heading rate uses unwrapped ATT.Yaw with a symmetric 2 s difference, not body yaw gyro or ground-course rate. Bank tracking uses CTUN.NavRoll minus CTUN.Roll; cross-track error uses NTUN.XT. The plotted manual interval is shaded. Selected settled windows were 780–825 s (60 m), 850–900 s and 950–1045 s (40 m). These windows do not compare different controller gains.

- [Lateral parameter ledger](assets/flight-testing/lateral-parameter-ledger.csv)
- [Response windows and statistics](assets/flight-testing/lateral-response.json)

## Lateral-controller evidence

The controller sketch is from page 323 of my notebook. The Hood River parameter values come from the January 3 onboard log. The algebra follows [`AP_RollController::get_servo_out`](https://github.com/DavidIngraham/ardupilot/blob/0f1121f32035d218c624fa7e110a124fd7fc6263/libraries/APM_Control/AP_RollController.cpp), [`AP_FW_Controller::_get_rate_out`](https://github.com/DavidIngraham/ardupilot/blob/0f1121f32035d218c624fa7e110a124fd7fc6263/libraries/APM_Control/AP_FW_Controller.cpp), and [`AC_PID`](https://github.com/DavidIngraham/ardupilot/blob/0f1121f32035d218c624fa7e110a124fd7fc6263/libraries/AC_PID/AC_PID.cpp) at the revision reported by that log. The numerical example holds speed scaling constant and stays below output limits.

The gains we selected for this SITL model are `PG_TURN_RMAX=25`, `FF=0.051`, `P=0.04`, `I=0.002`, `IMAX=0.15`, `ACCEL=40`, `FILT=4`, `TC=0.1`, `ASPD=5`, `RDAMP=0.03`, `D_FF=0.02`. They accompany `NAVL1_PERIOD=8`, `NAVL1_DAMPING=1`, `WP_RADIUS=22` and `WP_LOITER_RAD=25`. These are model-specific engineering selections, not general flight recommendations.


### Recorded initial simulation tuning

The February 14 Mission Planner log extends into February 15. Wall-clock timestamps are PST; the overnight span is recording coverage, not continuous active tuning. Parameter histories are recorded values, which can include reloads or automated changes as well as manual edits.

The two AUTO close-ups start at 11:30:00 and 11:35:15 on February 15 and last 45 seconds. They use ATTITUDE.roll and NAV_CONTROLLER_OUTPUT.nav_roll/xtrack_error. They have different navigation demand histories. FF and D_FF differ, so the plots illustrate recorded behavior without isolating a single gain's causal effect. The overview retains mode changes and the surrounding runs.

- [Tuning history](assets/flight-testing/sitl-lateral-tuning-history.json)
- [Comparison windows](assets/flight-testing/sitl-tracking-windows.json)

### Later model assumptions

When the simulated yaw response oscillated, I wanted to understand whether additional damping could be explained by a higher-fidelity model. Without flight data to identify it, we chose a provisional damping term and added propeller torque reaction. The canopy model builds on [Umenberger and Göktogan, ACRA 2012](https://www.araa.asn.au/acra/acra2012/papers/pap151.pdf). Added yaw damping (`Cnr=-0.05`) is an effective provisional term, and propeller torque reaction uses a provisional −0.02 m torque/thrust ratio. Neither is identified from flight logs. Higher-fidelity canopy/payload coupling can contribute apparent damping, but does not establish these numerical values.

## Can the launch start in the right physical state?

My actual launch starts with both payload and canopy upside down. I hold the bottom of the payload, lay the canopy out in a crescent, swing it overhead in about a second, walk two steps once it is inflated, and give it a light toss. I let it glide for about a second before throttling up. I wanted that sequence represented because the release state could matter to the oscillations we were trying to control.

We explored that sequence with provisional geometry and timing. A 1.0 s swing looked more promising than 0.8 or 1.2 s for release orientation, but about 39% of the held trajectory required compression in the assumed suspension constraint. That invalidates a taut-line rigid interpretation of actual inflation. This is a design study, not a validated launch simulation.

![Inverted swing study](https://github.com/DavidIngraham/micro-agu/blob/archive/pre-layout-2026-10-18/scratch/paraglider-tests/pitch-joint/swing-launch/inverted-launch.png)

The durable SITL throw guide is a synthetic initialization aid; it does not implement the overhead inflation sequence. The model assumes an inflated canopy and does not simulate slack lines, inflation or collapse. [Swing results](https://github.com/DavidIngraham/micro-agu/blob/archive/pre-layout-2026-10-18/scratch/paraglider-tests/pitch-joint/swing-launch/inverted-results.json) retain the checks and assumptions.


## Reproducing the figures

The [simulation extraction](assets/flight-testing/analysis/extract_sitl_nav.py), [tuning overview](assets/flight-testing/analysis/plot_sitl_tuning.py), [tracking comparison](assets/flight-testing/analysis/plot_sitl_tracking.py), [lateral flight analysis](assets/flight-testing/analysis/lateral_sysid.py), and [longitudinal fits](assets/flight-testing/analysis/system_id_fit.py) preserve the analysis behind the new figures. They expect the original local log tree and previously extracted JSON/NPZ data; paths need adjustment on another machine. Raw aircraft and telemetry logs are not bundled here. The repository [archive instructions](../README.md#archive-and-restore) describe the separately retained historical simulation scripts, compressed artifacts, hashes, and omitted files.

## Classical lateral analysis and frequency shaping

The current articulated model has **9 mechanical degrees of freedom**: two rigid bodies (12) joined by three positional constraints (−3). Its stored quaternion state, first-order velocities, rotor RPM, and actuator states must not be confused with that mechanical count. The lateral linearization removes global position and heading, leaving 14 internal mechanical states; rotor speed is fixed at equilibrium for this experiment.

The classical study uses the mission profile at Trout Lake mass 1.758 kg, model revision `af6b2ca963`. It computes a central-difference sampled Jacobian and its continuous equivalent, retains the full state order, and checks response against nonlinear sine tests. All feedback channels remain coupled. [Linearization and validation script](../research/archive/legacy-tools/classical_lateral.py), [report generator](../research/archive/legacy-tools/report_classical_lateral.py), and [modal/root-locus annotations](../research/archive/legacy-tools/annotate_lateral_roots.py) reproduce the figures and coefficients.

The [frequency-shaping experiment](../research/archive/legacy-tools/shape_lateral.py) compares six filters at five trims, followed by 18 nonlinear command/delay cases. [Its report generator](../research/archive/legacy-tools/report_lateral_shaping.py) preserves the tables and interpretation. Native backward-Euler low-pass filters are used in nonlinear checks; the experimental notch uses a bilinear discretization. Linear analyses freeze speed scaling at trim. Nonlinear runs recompute it and include actuator travel-rate limits and integral anti-windup. Common delay is applied to both rate observations; estimator errors, wind and unequal sensor delays are not modeled.

The mission profile alone was used in this standalone filter study. The native SITL follow-up is recorded below; the retained dynamic-roll profile and real flight remain separate follow-up checks. The mathematical pure-delay boundaries of several seconds after filtering are not avionics latency requirements: removing high-frequency unity-gain crossings leaves a low-frequency stability boundary. The practical nonlinear comparison used 80 ms. No flight-controller filter default was changed by these experiments.

Raw matrices, traces and local HTML reports remain in Mission Planner. Final figures live under `research/archive/flight-log-analysis/figures/turn-model-spike/classical-lateral/` and `lateral-shaping/`; scripts and the figure manifest provide reproduction and provenance without moving raw flight logs into Git.

## Native lateral filter trial

The subsequent native trial used feature-branch revision `2588ecc44a`, the 9-DOF mission profile at 1.758 kg and 22.2 V, and the existing `ParagliderAutoMission` assertions. Six runs paired 4 and 1 Hz lateral filtering in calm air, the existing repeatable synthetic gust field, and a steady 2 m/s eastward wind. [4 Hz configuration](assets/lateral-native-filter/filter-4.json) and [1 Hz configuration](assets/lateral-native-filter/filter-1.json) differ only in `PG_TURN_FILT`; the two-stage longitudinal damper remains fixed. Recorded parameters were checked against every requested setting.

All six tests passed. [Results](assets/lateral-native-filter/results.json) retain both turn directions, radial/height errors, unfiltered truth heading-rate error, command saturation and brake effort. Settled windows begin 20 seconds after reaching 60 m of each loiter centre and end at mission-item exit. Truth rate is the derivative of unwrapped payload Euler yaw, compared with `PGYR.Tgt` on native timestamps. Above-1-Hz command RMS uses a third-order offline zero-phase Butterworth high-pass. Single paired runs do not establish statistical significance; changed arrival times alter gust phase.

Run [the analysis script](../research/archive/legacy-tools/native_lateral_filter.py) with the local `lateral-native-filter` data directory and `--output` destination. Its six subdirectories are `calm-4`, `calm-1`, `gust-4`, `gust-1`, `crosswind-4`, and `crosswind-1`, generated with [the native runner](../research/archive/legacy-tools/run_gust_native.sh) and the respective configuration. Raw BIN, CSV and truth traces remain in Mission Planner. This trial neither injects the standalone study’s 80 ms delay nor changes branch defaults.

The [pitch-mechanics animation source](../viewer/apps/applets/pitch-fbd/applet.ts) is a separate explanatory illustration. It uses the carriage CAD, a schematic canopy and keyframed planar motion; it is not computed by the plant solver. The locked-joint panel illustrates the initial assumption. The free-joint panel illustrates the payload’s initial nose-down thrust response. It does not depict the ACRA test vehicle or establish that vehicle’s physical suspension stiffness.

## Articulated flight viewer

The [Three.js viewer](../viewer/README.md) consumes simulator truth state: NED position, separate payload and canopy quaternions, joint rotation vector and rate, actual brake travel, motor RPM, throttle and voltage. The 9 mechanical degrees of freedom are three shared translation coordinates, three payload attitude coordinates, and three canopy attitude coordinates. Actuator and motor dynamics add states without adding rigid-body degrees of freedom. A local Python bridge connects the optional UDP output to the browser over WebSocket; recordings use the same JSONL frame format.

The payload is a reduced GLB export of `microAGU.SLDASM`, aligned to the existing estimated Trout Lake CG. Named CAD components retain separate pivots: propeller and motor rotor, plus the two servo horns and capstans. Propeller phase integrates simulated RPM and turns clockwise viewed from behind. At video frame rates, the apparent blade direction can alias. Capstan angle changes by brake travel divided by the measured 27.5 mm drum radius. The first sample defines the visual reference; absolute servo zero and mirrored winding signs still need a physical check. The CAD prop is labeled APC 9x6, while the flown prop was 9x4.5; exporting this mesh does not alter propulsion physics. The canopy, lines and absence of a CAD ballast block are visualization approximations.

The [turn-and-reversal clip](paraglider.md#watching-the-two-bodies-move) uses the mission-profile model at 1.758 kg and 22.2 V. It starts from level-flight trim at 100 m, ramps right brake to 22 mm above trim, reverses to 20 mm of left brake above trim, then releases. Throttle rises by up to six percentage points during the maneuver. The physics runs at 50 Hz for 30 seconds; the browser records a silent 1280 × 720 WebM at a requested 30 fps and real-time playback. MP4 is provided for mobile compatibility. No joint-motion scale factor is applied.

Reproduction sources are [the maneuver generator](../research/archive/viewer-studies/make_showcase.py), [the video renderer](../viewer/packages/applet-ui/src/capture/video.ts), and [the encoding script](../viewer/tools/convert/encode_showcase.sh). The [clip metrics](assets/viewer/reversing-turn-turbulence.json) include the input schedule and SHA-256 of the source truth stream. Raw frames remain in Mission Planner under `analysis/longitudinal-2026-10-09/turn-model-spike/viewer-showcase-turbulence/`; Git contains the final video, poster, metrics and scripts. This is an open-loop visualization exercise, not another native AUTO acceptance result.

### Native AUTO mission video

The gust [viewer clip](paraglider.md#the-controller-flies-the-mission) comes from a fresh successful `test.Plane.ParagliderAutoMission` run on ArduPilot commit `2588ecc44a`, with `mission-controller-candidate.json`, the mission-profile 9-DOF model, 22.2 V and 1.758 kg. The heading-rate gains remain FF 0.009, P 0.005, I 0.002 and roll-rate damping 0.003, with the current native 4 Hz measurement filter. The earlier 1 Hz filter remains a research candidate. Native voltage compensation is enabled; pitch-rate throttle damping uses gain 0.1 and two 1.5 Hz low-pass stages.

[record_auto.py](../research/archive/viewer-studies/record_auto.py) observes the backend response during the unchanged AUTO harness and writes the complete truth state at about 29 Hz. It does not replace any actuator command. The complete test passed, retaining its mission CSV, flight BIN and truth stream in Mission Planner under `analysis/longitudinal-2026-10-09/turn-model-spike/gust-control/two-pole-native-gust/`.

[extract_auto_clip.py](../viewer/tools/convert/extract_auto_clip.py) selects the approach to the first clockwise loiter, beginning when mission item 3's target distance falls below 60 m. The published continuous interval is 237.518–292.503 simulation seconds. Native MODE records confirm AUTO throughout; native SIM roll/pitch versus interpolated truth have RMS differences of 0.0153°/0.0228°, checking timestamp alignment. The inset draws the excerpt's ground track, not a commanded path. [Metadata](assets/viewer/auto-gust-profile.json) preserves timestamps, trace hashes, controller parameters and joint-motion extrema. Video capture and encoding use the same scripts as the open-loop clip, at 1× playback with no motion scaling. The [viewer README](../viewer/docs/RECIPES.md#native-auto-mission-recording) gives the full command sequence.

The scripted maneuver and gust AUTO clip use `--turbulence`: the shared [gust generator](../research/archive/viewer-studies/turbulence.py) applies a uniform smooth multisine air velocity, with zero mean wind, nominal N/E/D RMS 0.5/0.5/0.25 m/s, frequencies 0.08–0.89 Hz, seed 20261011 and a five-second ramp after release. The AUTO excerpt has measured RMS 0.472/0.515/0.259 m/s. The native wrapper feeds the existing physics wind input and returns consistent wind and airspeed to SITL; it leaves controller outputs and test assertions unchanged. There are no spatial gust gradients, artificial joint torques or visual motion multipliers. “Moderate” describes the selected demonstration amplitude, not a validated atmospheric category. The prior calm-air metrics remain archived beside the new metrics. Both full native runs passed.

The capstan markings are procedural visual paint on the rotating CAD parts: black/white quadrants plus one orange radial index. They do not change brake travel, pivot geometry or the recorded physics. The scripted clip retains its original gust truth stream; the AUTO clip now uses the improved two-pole pitch damper. Both use a side-on mechanism camera and actuator overlays. [Longitudinal video diagnostics](../research/archive/viewer-studies/analyze_auto_gusts.py) compare the archived damper-off calm and gusty AUTO excerpts; their frequency estimates use linearly detrended periodograms over the full 55-second windows (approximately 0.018 Hz bin spacing). These independently aligned runs do not isolate the feedback contribution.

The AUTO comparison now shows three conditions with the same two-pole pitch damper and heading-rate controller. The clips start at the same mission event (item 3, target distance below 60 m), rather than the same elapsed simulation time. Each preserves about 55 continuous seconds of native AUTO flight. Altitude in the side profile is minus the truth state's NED down coordinate; the horizontal coordinate is cumulative ground distance. All three excerpts fit the shared 85–95 m display range.

| Condition | Source run under `turn-model-spike/` | Simulation interval (s) | Altitude range (m) | Metadata |
|---|---|---|---|---|
| Calm | `gust-control/two-pole-native-calm` | 233.248–288.198 | 89.45–90.11 | [Calm](assets/viewer/auto-calm-profile.json) |
| Gust | `gust-control/two-pole-native-gust` | 237.518–292.503 | 88.91–90.83 | [Gust](assets/viewer/auto-gust-profile.json) |
| Steady wind | `viewer-auto-crosswind` | 219.353–274.303 | 89.22–90.22 | [Crosswind](assets/viewer/auto-crosswind-profile.json) |

The new steady-wind run uses `record_auto.py --wind-ned 0 2 0`: 2 m/s eastward, applied to the physics after release, with no gust component. It passed the unchanged complete `ParagliderAutoMission` test, including opposing loiters, altitude and corner checks, home loiter and handover. Native SIM/truth roll and pitch alignment RMS is 0.0072°/0.0071° for its excerpt. The wind flag uses the recorded physics vector. The model, mass (1.758 kg), voltage (22.2 V), and controller parameters match the calm and gust runs; the longer overall route time in steady wind is retained in the raw log.

[prepare_wind_clip.py](../viewer/tools/convert/prepare_wind_clip.py) preserves states and timestamps, adds explicit zero-wind annotations to the legacy calm recording, and records both original and prepared clip hashes. Prepared clips and browser captures stay under `viewer-wind-comparison/{calm,gust,crosswind}` in Mission Planner. Only final videos, posters, metadata and their generating scripts are committed here. [run_gust_native.sh](../research/archive/legacy-tools/run_gust_native.sh) reproduces the three conditions using `calm`, `gust`, or `crosswind`.

## Longitudinal joint-control and observer research

These early experiments support the [longitudinal controller story](longitudinal-observer.md). They used the earlier pitch-joint model, before the current 9-DOF Trout Lake plant. Their numerical results describe those historical configurations, not the current mission controller.

### Modeling the moving suspension

The damper problem made canopy–payload coupling central: the longitudinal canopy/payload hinge was tricky, and my throttle pitch damper caused enough self-excitation that I zeroed it out. My thrust line is below the hinge, with the CG below that. I asked us to add the joint to SITL and investigate the coupling before the flight logs were available.

The first optional pitch joint we implemented separates canopy and payload pitch, with coupled inertias, translation and thrust moment. The payload IMU acceleration includes the moving lever-arm contribution. In that early model, roll/yaw remain shared; it predates the 9-DOF model used in the [later native mission study](longitudinal-observer.md#returning-to-native-mission-control). Representative settings are total mass 1.55 kg, canopy 0.19 kg, payload 1.36 kg, inertias 0.025/0.03 kg m² and joint damping 0.015 N m per rad/s. The payload CG is 0.35 m below the hinge. The modeled thrust line is 0.10 m above that CG, hence still 0.25 m below the hinge.

Pitch-sign conventions and aerodynamic blending were corrected during the experiments. Earlier directories explicitly marked before the aero-sign correction are historical results. The 15-degree linear-aerodynamic boundary and high-angle blend are numerical approximations, not a validated stall or collapse envelope.

The native `ParagliderPitchJoint` test and 20 model unit tests passed; the rigid-model extended AUTO test remains part of normal coverage. Local articulated AUTO runs completed, but those use different configurations and should not be pooled with lateral comparisons. [Physics review](https://github.com/DavidIngraham/micro-agu/blob/archive/pre-layout-2026-10-18/scratch/paraglider-tests/pitch-joint/physics-review/summary.json) records the model assumptions and trim analysis.

### From damping to relative-motion control

The longitudinal problem I want to solve is how to use as much control bandwidth as possible without stalling the canopy or driving the canopy and payload into large oscillations. After the flight evidence and articulated-model analysis, I was not convinced that motion-profile limiting alone would get me the performance I wanted. I suspected the controller needed some awareness of the relative angles and rates.

Before taking on an observer, I asked us to try controllers that could read the simulated joint state directly. That gave us a way to ask whether the information was useful at all, and what remained difficult even with perfect access to it. These local truth-state prototypes are research; they are not included in the durable ArduPilot branch.

### First, test whether joint-state feedback helps

We first kept TECS and added canopy/payload rate feedback ahead of motor lag, working through 44 trials. A useful correction was approximately $\Delta u_{\mathrm{throttle}}=-0.56q_{\mathrm{canopy}}+0.16q_{\mathrm{payload}}$, with rates in rad/s. In a matched faster-TECS case, payload pitch-rate RMS fell from 4.69 to 2.95 deg/s. Relative-rate information supplied most of the benefit; matched payload-only feedback achieved 5.02 deg/s. This reduced oscillation but did not demonstrate a large usable bandwidth increase.

I then asked us to try a controller update that treated the longitudinal system more directly. We tested integrated longitudinal control in a subsequent 21-trial study. Strong direct-throttle LQI gains became unstable with actuator timing/slew. The final conservative prototype commands throttle rate at 50 Hz, including motor lag, command state, saturation and conditional integral antiwindup. Its states include payload pitch, relative angle/rate, payload rate, air-relative velocity, motor throttle, previous command and climb-error integral.

| Matched step case | Climb tracking RMS | Payload rate RMS | Settling time |
|---|---:|---:|---:|
| Faster TECS baseline | 0.12851 m/s | 4.6922 deg/s | 12.04 s |
| Prior joint feedback | 0.09708 m/s | 2.9541 deg/s | 5.46 s |
| Conservative integrated | 0.07978 m/s | 2.3915 deg/s | 2.70 s |
| Faster integrated | 0.07123 m/s | 3.5172 deg/s | 2.20 s |

![Step comparisons](https://github.com/DavidIngraham/micro-agu/blob/archive/pre-layout-2026-10-18/scratch/paraglider-tests/pitch-joint/integrated-control/step-comparison.png)

The conservative setting was encouraging: it also improved rate RMS in zero-joint-damping, slower-motor and shifted-thrust-line checks. The severe gust cases showed how much work remained: about 38 deg/s rate RMS and angles of attack outside the calibrated model. At 0.32 Hz the whole-loop gain peak fell from 3.35 to 2.11, with more phase lag; the height outer loop still amplified the inner response. This is not proof of a safe high-bandwidth controller.

![Frequency comparison](https://github.com/DavidIngraham/micro-agu/blob/archive/pre-layout-2026-10-18/scratch/paraglider-tests/pitch-joint/integrated-control/frequency-comparison.png)

A local native AUTO course completed in 1103.6 s with maximum corner overshoot 1.079 m. It used an **airborne preparation fixture**, not a normal launch, and reached 20.16 degrees modeled angle of attack. Course assertions passing therefore do not establish stall protection.

![AUTO longitudinal behavior](https://github.com/DavidIngraham/micro-agu/blob/archive/pre-layout-2026-10-18/scratch/paraglider-tests/pitch-joint/integrated-control/auto-mission-longitudinal.png)

[Integrated summary](https://github.com/DavidIngraham/micro-agu/blob/archive/pre-layout-2026-10-18/scratch/paraglider-tests/pitch-joint/integrated-control/summary.json), [validation](https://github.com/DavidIngraham/micro-agu/blob/archive/pre-layout-2026-10-18/scratch/paraglider-tests/pitch-joint/integrated-control/validation.json), [oracle study](https://github.com/DavidIngraham/micro-agu/blob/archive/pre-layout-2026-10-18/scratch/paraglider-tests/pitch-joint/oracle-control/summary.json) and archived scripts provide provenance. Production sources/binaries were restored after these local trials. A payload-pitch transfer had a right-half-plane zero, while the CG climb transfer did not; inverse pitch response alone does not prove a fundamental climb-bandwidth limit.

### Can I estimate what I cannot measure?

The next question was how to get relative pitch information without a hinge-angle sensor. I asked us to assess observability with and without airspeed, rather than assuming an observer would work.

The nominal nine-state linearization we assessed uses payload pitch, relative pitch, payload/relative rates, two air-relative velocity components, motor state and horizontal/vertical wind. With a known model and constant wind, the local PBH assessment gives rank nine with attitude/gyro/GNSS measurements, with or without a pitot sensor. This nominal result is weaker than it sounds: modeling unknown constant force/moment biases adds ambiguous modes.

With four independent constant acceleration/angular disturbances, the thirteen-state assessment gives zero-frequency PBH rank ten without airspeed and eleven with scalar airspeed: respectively three and two unobservable constant directions. This deliberately permissive bias model exposes ambiguity rather than asserting those biases are a physical plant. Relative rate appears more useful than absolute relative angle; a scalar pitot does not measure airflow direction or vertical wind.

I also questioned whether we needed raw IMU data when ArduPilot already has an AHRS interface. Checking those interfaces clarified what is available and where the lever-arm correction still belongs. AHRS gyro is corrected payload angular rate. AHRS earth-frame acceleration is bias-corrected specific force at the IMU, not automatically translated to the payload CG. The corrected delta-velocity interface includes gravity but likewise does not supply that moving-origin correction. EKF navigation position/velocity handle configured rigid-body IMU lever arms; canopy articulation still requires its own geometry. A practical observer should predict specific force at the actual sensor position and use AHRS correction rather than duplicating calibration or differentiating noisy gyro. AHRS/IMU-derived measurements also have correlated errors.

[Assessment script](https://github.com/DavidIngraham/micro-agu/blob/archive/pre-layout-2026-10-18/scratch/paraglider-tests/pitch-joint/observer-design/assess.py) and [ranks/results](https://github.com/DavidIngraham/micro-agu/blob/archive/pre-layout-2026-10-18/scratch/paraglider-tests/pitch-joint/observer-design/assessment.json) are archived. No flight-ready observer or reliable absolute-angle limiter has been implemented.

### How much model identification can we avoid?

Finally, I wanted to revisit the free-body diagram. If this eventually needs to work on a variety of platforms, every inertia, mass and aerodynamic coefficient we ask a user to identify is a practical obstacle. We looked for terms that cancel and ratios that can replace absolute values.

![Normalized force balance and observer equations](https://github.com/DavidIngraham/micro-agu/blob/archive/pre-layout-2026-10-18/scratch/paraglider-tests/pitch-joint/diagrams/normalized-observer-fbd.png)

Let $\mathbf R$ be suspension force on the payload, $T$ forward thrust, $\mathbf D_p$ payload aerodynamic force, and $\mathbf f_p=\mathbf a_p-\mathbf g$ payload-CG specific force. Define $\tau=T/m_p$, $\mathbf d_p=\mathbf D_p/m_p$, $\kappa_p=I_p/m_p$, $\kappa_c=I_c/m_p$, and $m_a=M_a/m_p$. Vectors $\mathbf a$ and $\mathbf b(\delta)$ run from the hinge to payload and canopy CG in one frame; $z_T$ is the signed thrust moment arm using the diagram convention.

The payload force balance yields:

$$
\begin{aligned}
\frac{\mathbf R}{m_p} &= \mathbf f_p-\tau\mathbf e_x-\mathbf d_p
\end{aligned}
$$

Adding payload and canopy pitch equations cancels their equal-and-opposite internal hinge torque:

$$
\begin{aligned}
\kappa_c\dot q_c+\kappa_p\dot q_p &= \left[(\mathbf b-\mathbf a)\times(\mathbf f_p-\tau\mathbf e_x-\mathbf d_p)\right]_y+z_T\tau+m_a \\
s &= \kappa_c q_c+\kappa_p q_p \\
\dot s &= \left[(\mathbf b-\mathbf a)\times(\mathbf f_p-\tau\mathbf e_x-\mathbf d_p)\right]_y+z_T\tau+m_a \\
q_{\mathrm{rel}} &= \frac{s-\kappa_p q_p}{\kappa_c}-q_p
\end{aligned}
$$

This removes the explicit joint stiffness/damping law from the summed momentum propagation and replaces canopy lift/drag force modeling with measured suspension loading. Uniform gravity cancels when translation is eliminated. Absolute mass can be reduced to inertia/mass ratios; $\kappa_c=(m_c/m_p)k_c^2$ if $k_c$ is canopy radius of gyration. Scaling lengths by a reference suspension length and time by $\sqrt{L/g}$ improves numerical conditioning.

**The external canopy pitching moment $M_a$ does not cancel.** Geometry, inertia ratios, thrust-per-mass/moment arm, payload drag and residual canopy moment still require modeling. Integrating the summed momentum without measurement correction drifts; this cancellation is not a complete observer and does not solve initial-state identifiability. That suggested benchmarking an uncertainty-aware relative-rate observer against simulation truth, then test control with confidence-dependent limits and recorded flight data. Those experiments gave me a reason to pursue relative-rate feedback, but left open how reliably I could estimate absolute canopy angle or protect the wing at the bandwidth I wanted. Before adding that complexity to native control, I returned to the existing loops in the improved simulator.


### Calm-air baseline and voltage-study record

The following records the intermediate damper-off candidate, before the frequency-shaped gust controller described in the main article.

#### Why I still left the pitch damper off

Each revision compared damping off, a small reversed-sign gain, and a reversed-sign gain with filtering that rejects slow motion. Reversing the sign did not fix the excessive turn command in the first revision. Once steering was calibrated, all three tracked similarly, but damping off generally produced less settled pitch-rate variation. The filtered alternative reduced a few transient peaks without enough mission-level benefit to select it.

The wrong-sign flight damper and this result are compatible. Correcting a sign is necessary for the intended feedback action, but it does not establish a useful gain or account for the phase of a coupled canopy–payload system. For this candidate, the larger gains came from removing brake-driven excitation and improving feedforward.

#### The selected candidate and the next limitation

The combined controller passed the existing native AUTO test, including opposing multi-turn loiters, altitude and corner checks, home loiter, and controller handover. Over its roughly 20-minute mission record, maximum waypoint altitude error was 1.67 m and corner overshoot was 0.66 m. The [lateral results](paraglider.md#a-mission-that-tracks-without-the-repeated-rocking) show how much steadier the loiters became.

The same gains remained well behaved with a softer roll model. The first low-voltage test exposed a setup mistake: ArduPlane already had voltage compensation, but the simulation disabled the battery monitor, left compensation off, and did not send the motor model's voltage to the flight controller. Our initial standalone analysis omitted that output scaling too. We corrected the setup and reran both analyses.

With the existing compensation enabled, reducing supply voltage from 22.2 to 20.4 V now increases the throttle command automatically. We kept 22.2 V as the reference because that is where we calibrated these simulator throttle settings; it is not the full-charge voltage of a 6S pack. In matched native runs at 20.4 V, climb height RMS error fell from **0.486 to 0.250 m**, and level-phase error from **0.633 to 0.016 m**. At 22.2 V, compensation on/off gave essentially the same result.

![Native voltage-compensation comparison, showing altitude error, payload pitch rate, TECS demand and final motor command](../research/archive/flight-log-analysis/figures/turn-model-spike/voltage-compensation/voltage-response.png)

There is a tradeoff: climb-capture height RMS error increased from **0.096 to 0.169 m**. Native compensation also raises the maximum throttle returned to TECS, changing its climb feedforward slope before the final output scaling. The corrected linear and nonlinear analysis includes that interaction. This fixes the missing battery path and improves low-voltage tracking; it does not eliminate every capture transient or justify enabling the pitch damper.

Both voltage settings passed the full AUTO regression with the original checks unchanged. Maximum waypoint altitude error was 1.67/1.68 m at 22.2/20.4 V, while corner overshoot was 0.95/2.89 m. The lower-voltage case still turns wider at the worst corner, despite its steady loiter motion.

That calm-flight candidate used calibrated feedforward, reduced lateral feedback, smoother altitude capture, and no pitch-rate throttle damping. The [gust study in the main article](longitudinal-observer.md#shaping-the-damper-around-the-gust-response) adds frequency-shaped damping to that starting point. Relative-motion feedback remains a longer-term avenue. These are powered, up-and-away SITL results; wind, launch, landing and flight validation remain separate work.

The new [articulated flight viewer](paraglider.md#watching-the-two-bodies-move) puts these relative motions back into a picture of the aircraft. Its looping turn-and-reversal clip shows the SolidWorks payload, moving brake capstans, and RPM-driven pusher propeller alongside the canopy. During controller development I can replay the same internal state used in the plots and inspect how payload pitch differs from canopy attitude. A second [AUTO mission clip](paraglider.md#the-controller-flies-the-mission) now shows native steering and throttle control working together as the aircraft enters a loiter at 90 m, taken from a passing mission test with moderate synthetic gusts. Both viewer clips now apply those gusts through the physics model, with nominal horizontal/vertical RMS speeds of 0.5/0.25 m/s.

The [data and methods appendix](data-and-methods.md) collects flight chronology and earlier simulation provenance. The [current plotting scripts](../research/archive/flight-log-analysis/scripts/native-sitl) and [figure manifest](../research/archive/flight-log-analysis/manifest.json) preserve this study. Native implementation and updated test instructions are recorded at [ArduPilot commit `c441b937bd`](https://github.com/DavidIngraham/ardupilot/commit/c441b937bd) on `paraglider-high-dof-control`; raw research logs remain in Mission Planner and the original SITL experiment folders.
## Full-loop longitudinal design

The [consolidated controller article](controller-design.md) describes the selected architecture and tuning. The [post-flight research record](controller-research-record.md) preserves the chronological studies and their historical figures/videos.

The full-loop study reuses the rotor-inclusive plants from the compensated longitudinal analysis: seven local operating cases, each checked at lateral airspeed scales 0.9, 1.15 and 1.4. Ten settings explore three tuning directions. Both baseline and selected settings also pass 126 combined rotor-response sensitivity checks (±30% rotor rate, including nominal), with no unstable closed-loop poles in that set. This tests local model sensitivity; it does not establish flight robustness.

The 15-state plant retains body and relative attitude/rate dynamics, body velocity and rotor RPM. Translation and absolute heading are removed. Frozen-scale steering adds three states; the full longitudinal closure adds altitude, climb PI integration, two pitch filters and the smoothed altitude reference. The height reference is the shaped trajectory: a small 0.1 m ramp over two seconds avoids command and actuator saturation in the linear-validation comparison.

The plant transfer functions are supplied as modal residues and poles. Finite transmission zeros are computed by a generalized eigenvalue problem, with right-half-plane zeros checked directly against the state-space resolvent. No unstable pole-zero cancellation is used to simplify the controller. Loop frequency responses include all feedback paths; the saved crossover phase distances are not standalone stability guarantees for this multiple-crossing loop.

- [Screen and operating-case results](assets/longitudinal-final-design/screen.json)
- [Plant modes and zeros](assets/longitudinal-final-design/plant-modes-zeros.json), [transfer functions](assets/longitudinal-final-design/transfer-functions.json), [zero verification](assets/longitudinal-final-design/zero-check.json)
- [Loop crossings](assets/longitudinal-final-design/loop-margins.json) and [Nyquist plot](assets/longitudinal-final-design/nyquist.png)
- [Rotor sensitivity](assets/longitudinal-final-design/rotor-uncertainty.json) and [full-loop nonlinear validation](assets/longitudinal-final-design/full-loop-validation.json)
- [Matched gust results](assets/longitudinal-final-design/nonlinear-summary.json) and [stronger independent realization](assets/longitudinal-final-design/nonlinear-strong-summary.json)
- [Native comparison](assets/longitudinal-final-design/native-results.json), [selected parameters](assets/longitudinal-final-design/native-candidate.json) and [diagnostic candidates](assets/longitudinal-final-design/candidates.json)

Native runs used the feature branch at 2588ecc44a with a parameter override, then selected TECS_VERT_ACC = 3.5 in both mission-development files. The baseline is the archived passing two-stage-damper experiment; comparing logged TECS, steering and battery parameters confirms that TECS_VERT_ACC is the only change. Both new native AUTO tests pass with original assertions unchanged, no backend errors and no clipped packets. Their full BINs and state histories remain in Mission Planner.

For the native settled-loiter metrics, measurement starts 20 seconds after entering a 60 m radius around the loiter center and ends when the mission item changes. Baseline and candidate encounter different gust phases because trajectory timing changes. Matched truth-sensed simulations provide the causal comparison; native runs verify integration. Calm corner overshoot increases from 0.36 to 0.63 m, while gust height error improves. That tradeoff is retained in the recommendation.

Reproduction scripts:

- [Full-loop linear analysis](../research/archive/legacy-tools/longitudinal_final_design.py): --plants PATH_TO_CLASSICAL_LONGITUDINAL_COMPENSATED --output OUTPUT
- [Nonlinear screen](../research/archive/legacy-tools/gust_control_study.py): --model MODEL_DIR --output OUTPUT --candidates candidates.json; use --seed 20261012 --scale 1.5 for the second realization, or --scale 0 --height-ramp 0.1 for small-command validation.
- [Native runner](../research/archive/legacy-tools/run_gust_native.sh): ARDUPILOT_REPO OUTPUT native-candidate.json gust (then calm).
- [Native summarizer](../research/archive/legacy-tools/summarize_gust_mission.py) on each run, then [comparison report](../research/archive/legacy-tools/report_longitudinal_final_native.py) on the study root.

Raw outputs are under Mission Planner/analysis/longitudinal-2026-10-09/turn-model-spike/longitudinal-final-design. The parent classical-longitudinal-compensated directory contains cached plants and trim states; [the original plant generator](../research/archive/legacy-tools/longitudinal_classical.py) reproduces those. Run the linear analysis again after the nonlinear checks to generate the overlay figure. Final figures, numerical summaries and scripts are archived here; raw logs remain outside Git.


## Landing-envelope refinement

The [landing article](spot-landing.md) extends the established mission controller to approach and flare. Its current comparison uses the complete longitudinal profile on both sides, changing only the lateral measurement cutoff from 4 Hz to 1 Hz. Twelve established conditions are paired, followed by twelve fresh validation cases at seed 2026101307. Two additional illustration flights use the original replay geometry with 0.25 m/s horizontal gust amplitude; they are excluded from both statistical sets.

The local analysis covers nine straight, turning and partially braked equilibria. It uses the logged 50 Hz controller update, command hold, live airspeed scaling and separate 0–120 ms delay cases. Native tests evaluate the complete changing approach/flare trajectory. The [research record](../research/archive/flight-log-analysis/landing-refinement-2026-10-13.md), [exact configurations and scripts](https://github.com/DavidIngraham/micro-agu/tree/main/research/archive/flight-log-analysis/scripts/landing-refinement-2026-10-13), and [parameter audits](assets/landing-refinement/results.json) preserve the distinction. Raw logs remain in Mission Planner/analysis/landing-refinement-2026-10-13.

## Joint tuning and state-space comparison

The [14 October controller study](controller-state-space-archive.md) regenerates six sampled plants with rotor dynamics and altitude, jointly tunes eight existing-controller settings across five of them, and holds out the softer-roll climb. The nominal frequency-domain feedback assembly and augmented state-space solution agree numerically. Separate finite-horizon Gramians, modal input coupling, initial-state transient response, and a 72-design full-state LQI screen explore questions beyond the earlier transfer-function tuning.

The [reproduction guide](../research/archive/state-space-2026-10-14/README.md) declares state/input/output scales, the objective and constraints, solver limits, model hashes, measurement assumptions, and exact validation windows. The study retains both an unconstrained tradeoff and the constrained candidate. Its 648 combined uncertainty cases per conventional tune are a finite stress grid, not a formal robustness certificate. Truth-sensed nonlinear diagnostics and native AUTO regressions are recorded separately. No flight-ready observer or production controller-default change is claimed.
