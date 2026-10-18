# Viewer reproduction recipes

These recipes document individual recordings and historical experiments.
The [workspace README](../README.md) defines the current architecture and build.
Physics is now owned by the Rust simulator in microAGU; ArduPilot owns flight control.

## Run

Node 22+, Python 3.10+:

```sh
cd viewer
npm ci
npm run dev -- --strictPort
# Open http://127.0.0.1:5173
```

Open a JSONL recording with **Open recording**. Playback supports pause, scrubbing,
0.25–4× speed, quaternion interpolation, four orbitable camera modes, body axes,
flight paths, and synchronized joint/actuation charts. The ground grid is 10 m;
it is a datum plane, not terrain. Mouse drag orbits; wheel zooms; right drag pans.
**Inspect joint** gives a closer view of the two bodies.

### Existing research traces

```sh
python -m pip install -r ../viewer/tools/recording/requirements.txt
python tools/convert/convert_trace.py \
  /path/to/high-dof-trace.npz /path/to/replay.jsonl --voltage 22.2
```

The native autotest's `samples` array contains `[time, released, state[26]]`.
It does **not** contain RPM or actuator positions. These remain `null` and the UI
says “Not recorded”; no values are reconstructed or invented. Voltage is explicit
because these old traces don't record it either: use the run's physical voltage.

The converter also accepts `t`, `state`, `rpm`, and `commands` arrays from simulator
replays. `commands` must be the physical model inputs `[throttle, left_mm, right_mm]`;
for rate-limited replays, supply **applied** travel, not pre-actuator demands.
Output creation is exclusive to prevent accidentally replacing a recording.
Raw traces and generated JSONL are ignored by Git and should stay in the existing
research-data locations. Nothing needs uploading to a service.

### Live simulator

Start the bridge in another terminal:

```sh
cd viewer
python ../viewer/tools/recording/bridge.py --udp 9010 --port 8765 --record /path/to/new-run.jsonl
```

Then start the backend on the ArduPilot feature branch:

```sh
cd /path/to/micro-agu/simulation
python json_backend.py --flight trout_lake --profile mission \
  --voltage 22.2 --viewer-port 9010 --viewer-hz 30
```

Start SITL with the existing `sim_vehicle.py` JSON-model command and parameter
files documented in that model's README. Initialize/arm/select mode, then press
Enter in the backend terminal to release, as before. Click **Connect live** in the
viewer. This is a passive viewer: disconnecting does not stop the simulation.
Existing autotest wrappers that instantiate `Backend` directly do not start the
CLI's UDP output; their saved pose traces can be converted instead.

All services bind to loopback; no network-facing server is created. In WSL, run
backend, bridge, and Vite in the same distro; Windows can reach the forwarded
localhost ports. The bridge accepts the standard Vite dev/preview origins only.
It retains one pending message per client and drops superseded frames for slow
clients. The backend uses nonblocking UDP and drops send failures without waiting.
The output rate is **simulation Hz**, so accelerated SITL may emit faster in wall
clock time. Live UI history is limited to 1,800 frames; disconnect to replay those
frames, or load the full recording from disk. Recordings flush every 30 frames.
A simulator restart stops the current recording to avoid mixing time epochs;
restart the bridge with a new filename to record the new run.

## State contract: `microagu.state.v1`

One JSON object per UDP datagram, WebSocket message, or JSONL line.

- `time_s`: simulation seconds, strictly increasing within a recording.
- `state`: 26 finite numbers. Payload occupies indices 0–12, canopy 13–25.
  Each body's layout is `[position_NED(3), quaternion_xyzw(4), velocity_NED(3), angular_velocity_NED(3)]`.
  Quaternions map **body FRD to world NED**. Positions are body CGs in metres.
- `joint_rotvec_rad`: rotation vector of `R_payload^-1 R_canopy`, expressed in
  payload axes. These are the spring model's joint coordinates, **not Euler angles**.
- `joint_rate_rad_s`: `R_payload^-1 (omega_canopy - omega_payload)`.
  Relative angular velocity, not generally the derivative of the rotation vector.
- `brake_mm`: actual left/right actuator positions, before slack/preload adjustment.
- `throttle`: bounded model throttle command, 0–1.
- `rpm`, `voltage_v`: rotor speed and physical supply voltage.
- `released`: false during translating initialization trim, true after release.

RPM, throttle, and brake travel may be `null` only for legacy recordings with
missing channels. The live exporter provides them. The bridge requires complete
live frames. Joint coordinates are derived from the poses, not autopilot estimates.
The viewer uses the proper rotation `[N,E,D] -> [E,-D,-N]` and interpolates body
quaternions using shortest-path slerp. It never feeds truth back into the autopilot.

## SolidWorks assets

The default model is your `microAGU.SLDASM`, exported through the running
SOLIDWORKS 2026 typed COM API. The native export was 21.3 MB; the prepared viewer
asset is 1.68 MB. It retains the named assembly hierarchy and mechanisms, removes
cameras and internal M3 fasteners, and welds/deduplicates/quantizes the meshes.
`assets/models/microagu-payload-tl.glb.json` records provenance and checksums.

The asset is aligned using the simulator's existing **estimated nominal Trout Lake
CG** and `FRD = [CADx, -CADy, -CADz]`. This is not a new CG measurement. The canopy
and suspension lines remain schematic, and the added lead is not represented in
CAD. The CAD prop is named **APC 9x6**, while the flown prop was **9x4.5**; its mesh
is a visual asset and does not change the physics model.

To repeat the export, run this with Windows PowerShell 5.1 while SolidWorks is open:

```powershell
.\tools\export_solidworks.ps1 -Source "C:\path\microAGU.SLDASM" -Destination "C:\path\raw.glb"
```

The script locates the installed interop DLL, connects to the running application,
opens the assembly read-only if needed, and exports a separate copy. It never
saves over the source CAD. If Windows blocks an unsigned script on a UNC share,
copy this authored script into your local working folder and run it there.

Prepare a new output (the tool refuses to overwrite an existing file):

```sh
node tools/convert/prepare_asset.mjs /path/to/raw.glb \
  /path/to/micro-agu/simulation/napkin-payload-model.json \
  /path/to/payload-tl.glb
```

Use **Load payload GLB** for other self-contained GLB assets. Prepare these in
metres with the **payload CG at the origin**, X forward, Y right, Z down. Y-up
exports need reorientation first. Retain named rotor/drum nodes for future
articulation. The bundled Micro-AGU asset now has explicit pivots for the propeller
and motor rotor, and for both capstans together with their servo horns. Motor stator,
servo cases, and payload structure remain fixed.

Propeller angle integrates RPM over simulation time (trapezoidal integration and
partial-interval interpolation), with arbitrary initial shaft phase. Positive spin
is clockwise looking from behind, as specified for this aircraft. Pause and seeking
are deterministic; high RPM can still alias at display refresh rates.

Capstan displacement is `(travel_mm - first_recorded_travel_mm) / 27.5` radians,
using the measured 55 mm drum diameter. The CAD assembly pose is the visual reference
for the first valid brake sample, **not a calibrated absolute servo-zero angle**.
Left is CAD +Y / body -Y; the mirrored winding signs are a visualization convention
pending a physical zero/winding check. Missing channels leave their mechanisms static.
The model-specific rig is applied only when all expected Micro-AGU part names exist;
other uploaded GLBs remain rigid. No controller or physics changes are involved.
Force arrows remain absent until the simulator exports consistently defined forces.

## Recording a showcase clip

Generate a 30-second powered right turn, left reversal and brake release using the
real physics backend (scripted actuators, without a native autopilot):

```sh
python ../research/archive/viewer-studies/make_showcase.py --model-dir /path/to/micro-agu/simulation \
  --output /path/to/Mission-Planner/analysis/viewer-showcase/reversing-turn.jsonl
```

Open that recording, wait for the CAD asset, and select **Record video**, then
**Save video** when recording finishes. Keep the viewer visible during capture.
This exports a silent WebM with a full-aircraft view,
a payload-mounted detail camera, joint angles and actuation readouts. Capture uses
real elapsed time at 1×; the browser requests 30 fps but may drop frames under load.
The propeller can alias at that sampling rate. Convert the downloaded file for the
blog, including an MP4 fallback and poster, with:

```sh
bash tools/convert/encode_showcase.sh /path/to/microagu-flight.webm ../docs/assets/viewer
```

The raw JSONL stays outside Git. Preserve the generator's small metrics JSON next
to the final videos to identify the input schedule and trace checksum.

### Native AUTO mission recording

Run from the ArduPilot feature checkout with the autotest Python environment active.
Use a fresh run directory in the existing research data location:

```sh
PG_GAINS=original PG_VOLTAGE=22.2 \
ARDUPILOT_ROOT=/path/to/ardupilot PG_CONTROLLER_CONFIG=/path/to/micro-agu/simulation/mission-controller-candidate.json \
BUILDLOGS=/path/to/research/viewer-auto-mission \
python /path/to/ArduPilot-tinkering/research/archive/viewer-studies/record_auto.py \
  --model-dir "/path/to/micro-agu/simulation" \
  --output /path/to/research/viewer-auto-mission/truth.jsonl -- \
  --no-check-parameter-leaks --speedup 10 test.Plane.ParagliderAutoMission
```

This observes `Backend.respond` while the existing AUTO harness runs. All native
controller outputs, mission items and assertions are unchanged. After a successful
test, run `extract_auto_clip.py /path/to/research/viewer-auto-mission`. It selects
55 continuous seconds of the first loiter entry, checks native MODE records for
AUTO throughout, and checks truth/native SIM attitude alignment before creating
`auto-clip.jsonl` and `auto-mission.json`. The optional replay annotations `mode`
and `mission_item` come from this check, not from recognizing a circular trajectory.
The video renderer adds an AUTO title and an inset of the excerpt's actual track.

Load that clip and record/save it as above. Encode with:

```sh
bash tools/convert/encode_showcase.sh /path/to/auto-mission.webm ../docs/assets/viewer auto-mission
```

The test includes airborne initialization, a complete route, altitude changes,
opposing two-turn loiters and controller handover. The published excerpt covers
only the first loiter entry, with the heading-rate controller enabled.

## Build and checks

```sh
npm test
npm run build
python -m unittest discover -s tools -p 'test_*.py'
python ../viewer/tools/recording/check_live.py --model-dir /path/to/micro-agu/simulation
# Native exporter/protocol checks, from ArduPilot model directory:
python -m unittest test_json_backend
```

`npm run preview -- --strictPort` serves the production build on port 4173.
Build output and local data are ignored. Third-party runtime: Three.js (MIT).
The viewer source follows the repository's GPL license.

Validation performed: production build, coordinate/sign and quaternion interpolation
checks, replay input rejection, bridge backpressure/reset recording checks, native
protocol tests with real physics stepping, and backend CLI → UDP → WebSocket →
recording integration (400 lockstep responses, 29 truth frames). Browser inspection
used the 432-second `voltage_22_2_on` native SITL recording and the exported CAD;
playback, timeline seeking, camera switching, and plots were checked. No controller
parameters or physics equations were changed.

### Repeatable synthetic turbulence

Add `--turbulence` to either `make_showcase.py` or `record_auto.py`. Both use
`../research/archive/viewer-studies/turbulence.py`: zero mean wind, nominal RMS gust speeds of 0.5 m/s north,
0.5 m/s east and 0.25 m/s down, frequencies 0.08–0.89 Hz, seed 20261011 and a
five-second cosine ramp after release. “Moderate” describes this demonstration's
chosen amplitude; it is not a validated atmospheric turbulence category.

This is a spatially uniform smooth multisine velocity field, not a Dryden model.
It enters the existing aerodynamic/propulsion wind input. The native adapter also
returns matching wind and airspeed to SITL. No cosmetic joint rotations or random
torques are added. Native actuator commands and regression assertions stay intact.
The recorder writes `truth.turbulence.json`; extraction carries its settings into
the final metadata. Report the full test outcome separately from the clip.

The CAD capstans receive procedural black/white quadrant markings and one orange
index stripe in `packages/rendering/src/aircraft/mechanisms.ts`. The paint rotates with each drum using the
existing physical brake-travel mapping; it does not alter the GLB or simulator.
The exported video uses a side-biased inspection camera to expose the drum face.
`../research/archive/viewer-studies/analyze_auto_gusts.py ROOT --output DIRECTORY` compares the retained calm
and gusty AUTO excerpts and creates the longitudinal diagnostic figure/metrics.

Exported videos include a north-up wind flag pointing **toward** the horizontal
wind direction, plus horizontal speed and vertical up/down speed. This reads the
recorded earth-frame wind vector, not apparent wind relative to the aircraft.

### Actuator overlays in exported videos

Video exports are 1280 × 852 at real speed. Below the two camera views, a trailing
10-second graph shows throttle command on a fixed 0–100% axis. Left/right bars
show actual simulated brake travel on a common 0–100 mm axis (a display scale,
not a physical end-stop specification). Missing signals display N/A. The graph
uses recording timestamps and shows no future samples; scrubbing does not
accumulate stale history. The wind flag and capstan markings remain visible.
The published AUTO controls clip uses the two-pole gust-controller run under
`gust-control/two-pole-native-gust`, with its metadata saved alongside the video.

### Calm, gust and steady-wind comparison

`record_auto.py --wind-ned 0 2 0` applies a steady 2 m/s eastward air velocity
after release. Add `--turbulence` only to superimpose the repeatable gust field.
The vector reaches the existing aerodynamic/propulsion wind input and is also
returned as native wind and airspeed sensor data. No controller parameters change.
`research/archive/legacy-tools/run_gust_native.sh` accepts `calm`, `gust`, or `crosswind` for these cases.

`tools/convert/prepare_wind_clip.py RUN OUTPUT [--calm]` prepares a comparison recording
and metadata. The explicit `--calm` option adds zero-wind annotations to a known
legacy calm recording; it never changes its physics states or timestamps.

AUTO video exports include a side profile directly below the north-up ground
track. Its horizontal axis is cumulative ground distance travelled, so a circular
loiter is unfolded; its vertical axis is altitude above the NED origin. A moving
white dot follows the aircraft. The three published loiter-entry clips all use
85–95 m vertically, preserving the visual difference between calm and gusty
altitude response. Other recordings expand the axis in 5 m steps as needed.

### Pitch-sign carriage comparison

`research/archive/legacy-tools/pitch_sign_video.py --states-only` exports the four matched full-state JSONL recordings (including RPM) into the Mission Planner `pitch-sign-video` directory. Link that directory to the ignored `viewer/assets/recordings/pitch-sign` path, start `npm run dev`, then open `/pitch-comparison.html` and select **Record comparison**. The page is a local recording tool, not a published live replay; raw state recordings remain outside Git.

The four views use the existing Trout Lake SolidWorks GLB and mechanism rig. The camera has a fixed side-oblique orientation and orthographic scale. Its origin advances at the initial trimmed velocity, independently of every case's actual trajectory; it does not cancel payload translation or rotate with its attitude. All cases share this reference and scale. The orange point is the modeled upper attachment at body FRD `[0.0053, 0, -0.102]` m, with a trailing 0.8-second path relative to nominal flight. The propeller uses integrated simulated RPM and can alias at the video frame rate. The clip plays 0–5.5 seconds at real speed, followed by 2–5.5 seconds at quarter speed, so the initial instability stays in frame at an inspectable scale. Neither displacement nor rotation is amplified.

Encode the downloaded WebM to `docs/assets/viewer/pitch-sign/pitch-sign-carriage.mp4` with H.264, yuv420p and faststart; extract a poster from the slow replay. Keep the original browser recording in Mission Planner. The prior schematic MP4 remains available as a longer diagnostic, but the article embeds the CAD comparison.

### Altitude-command comparison

Run `research/archive/legacy-tools/altitude_comparison.py` with `--model`, `--raw` (Mission Planner) and `--output docs/assets/viewer/altitude-comparison`. Link the raw directory to ignored `viewer/assets/recordings/altitude-comparison`, then use `/altitude-comparison.html` under the Vite development server to record the four cases. This is a local recording tool. It reuses the actual carriage rig and schematic canopy; the pitch glyphs display world-referenced Euler pitch, not angle of attack or a full three-axis joint rotation. The pitch difference readout is the difference between those two Euler angles.

The fixed-orientation camera follows a centered one-second boxcar average of translation, keeping the complete glider in view. The reference is interpolated at the same replay timestamp as the aircraft. Linear extrapolation at the recording boundaries preserves constant-velocity travel. This replaces the original rounded 50 Hz camera index and two-endpoint average, which introduced visible tracking jitter. The corrected published asset is `altitude-comparison-smooth-camera.mp4`; simulation trajectories and reported metrics are unchanged. Absolute altitude remains in the plot and readout. All graphs use common ranges that include the observed excursions. The raw WebM and JSONL/NPZ traces stay in Mission Planner; final MP4, poster, summary and scripts go in Git. Encode H.264/yuv420p with faststart, at 30 fps, and use a mid-climb frame for the poster.

### Real flight-log replay

Open `/flight-replay.html` in the viewer for onboard-log replay. Convert a DataFlash BIN with Python, NumPy, SciPy and pymavlink:

```sh
python tools/convert/convert_flight.py flight.BIN /path/outside/git/flight.json --bin \
  --start 298.44 --end 1073.56 --title "Hood River · January 3" \
  --summary ../docs/assets/flight-testing/replay/hood-river.json
```

Omit `--bin` to read a directory containing decoded ATT, XKF1, RCOU, MODE and PARM JSON arrays. Open the converted file with the page's file picker, or serve it locally using `?recording=/flight-replays/hood-river.json`. Raw recordings stay in Mission Planner; `assets/recordings/flight-replays` is ignored. Only final videos, images, metadata and generating scripts belong in Git.

The `microagu.flight.v1` format carries estimator payload attitude, core-0 XKF1 local position, flight mode, raw servo PWM, and normalized commands. The converter resamples at 25 Hz, interpolates position and quaternion attitude, and holds discrete outputs/modes. It applies the time-varying SERVO MIN/MAX/REVERSED parameters independently for each flight. A source-sample age over 0.25 seconds marks a gap. Horizontal position is relative to the selected start; altitude is above the EKF origin, not AGL. No canopy attitude or propeller RPM is inferred. The current CAD carriage is illustrative of the build, capstan motion represents estimated commanded position rather than measured travel, and the propeller remains stationary.

The page provides playback, speed selection, scrubbing, a 3D overview PNG, and browser WebM recording. The payload close-up uses a fixed world orientation; the route view preserves real spatial scale. Graphs show the preceding 15 seconds of attitude and output commands. Published recap schedules are specified in `apps/applets/flight-replay/applet.ts`: full-flight overview in 45 seconds, then real-speed excerpts. Hood River uses boot 298.44–1073.56 s, with excerpts 755–785 and 625–645 s; Trout Lake uses 202.81–3474.77 s, with excerpts 635–680 and 2800–2820 s. The selected source directory names identify those schedules; other recordings receive a full-flight overview.

Encode downloaded WebM as H.264/yuv420p, CRF 20, 30 fps, with faststart. Keep browser captures outside Git. Final media and provenance are under `docs/assets/flight-testing/replay/`.

### Pitch-damper mechanical illustration

`/pitch-fbd.html` renders a 14-second looping side-view comparison with the existing SolidWorks carriage: locked relative pitch on the left, independent payload pitch on the right. Motion stays in one plane. The moving hinge, payload CG, assumed combined CG, incremental pusher thrust and curved thrust-moment arrows explain the sign distinction. The prop is stationary; rotor dynamics are not represented in this illustration.

This is a keyframed mechanical explanation, not a solver trajectory or a reconstruction of the ACRA vehicle. Geometry, timing and angular excursions are illustrative. Use **Record animation**, download the WebM, encode H.264/yuv420p at 30 fps with faststart, and extract a poster at 7 s. Raw captures stay in Mission Planner; the final MP4/poster are in `docs/assets/viewer/pitch-fbd`.

## Interactive DOF and joint-response tour

Open `joint-tour.html`; it defaults to the 9-degree disturbance response. A scrolling 0.5 m reference grid shows horizontal translation speed (not terrain height). Motion-tour mode prescribes nine independent coordinates;
response mode offers 60 open-loop actual-model releases (two disturbed bodies, three axes, two profiles, five amplitudes).
The disturbance slider selects 3–15 degrees in 3-degree steps, defaulting to 9. Select canopy-only or payload-only deflection. The selected body rotates about an initial payload axis while the other body retains its attitude. Each perturbation preserves attachment position/velocity by repositioning the selected body about the hinge; both bodies then evolve freely. Trim
actuators remain fixed in open loop. The additional native closed-loop comparisons are described below. Neither mode adds turbulence or artificial animation to the response trajectories. The camera follows nominal forward/vertical motion and actual lateral attachment drift, projected perpendicular to the initial horizontal flight track. Fore–aft and vertical excursions remain visible. The ground grid includes lateral camera travel, and the gray canopy stays on the nominal trajectory. Framing uses the same reference transform. Model amplitudes are unscaled. Torque is canopy-side, payload-frame,
using the same log-Jacobian spring law and relative-rate damper as simulation/crates/sim-core/src/dynamics.rs.

Generate final playback samples at 20 Hz while keeping 100 Hz raw output outside Git:

```sh
python ../research/archive/viewer-studies/make_joint_tour.py --model-dir /path/to/micro-agu/simulation \
  --output assets/recordings/joint-tour/responses.json --raw /path/to/Mission-Planner/joint-tour
```

The generator checks attachment residuals and nonpositive damper power. It records
the model revision and resolved stiffness/damping in each case. These are short
presentation datasets, not relocated flight logs. `Record overview` exports the
nine-motion tour followed by three mission-profile responses as a 96-second WebM.
The interactive page additionally supports the softer-roll alternative and the disturbance-size slider. The existing article video records the original 3-degree cases.

Publish the bounded web bundle after `npm run build`:

```sh
python tools/publish_joint_tour.py /path/to/website/assets/microagu-viewer
```

Only the tour entry point, its JavaScript dependencies, the payload GLB and final
presentation samples are copied. Other local recordings are excluded.

### Native closed-loop release comparison

The selector includes equilibrium-trim open loop and native AUTO closed loop.
AUTO provides 12 releases at 9 degrees (two bodies, three axes, two profiles).
Choosing AUTO locks the amplitude to 9 degrees. The original trim cases remain
available, including the earlier rotor-coupling example. The frozen-command
shadow remains a raw research diagnostic and is not offered in the widget.
Camera framing is shared within each body/axis/profile/amplitude comparison.

`research/archive/sitl-studies/make_closed_joint_tour.py` uses the airborne SITL harness and the branch's
`mission-controller-candidate.json`. AUTO flies a straight northbound leg toward
waypoints 1 km and 2 km from home, at 100 m relative altitude. Physics runs under
native control for 100 seconds before the disturbance; mission item 1 and AUTO
mode are verified before release. This replaces the CRUISE examples that enabled
control simultaneously with the disturbance. It changes both guidance mode and
initialization, so differences cannot be attributed to the mode switch alone.

Plots show truth deviations from release altitude and ground track, not internal
EKF errors; throttle is commanded, brake pulls are actuator positions. These
eight-second diagnostics are not mission acceptance or stability tests. Raw
logs remain in Mission Planner. Packaged traces record parameter readback, source
revision and executable SHA-256. No firmware/controller changes are required.

```sh
python research/archive/sitl-studies/run_closed_joint_tour.py --repo /path/to/ardupilot --raw /path/to/Mission-Planner/joint-tour/auto-suite
python research/archive/sitl-studies/package_closed_joint_tour.py --repo /path/to/ardupilot --raw /path/to/Mission-Planner/joint-tour/auto-suite --output viewer/assets/recordings/joint-tour/responses.json
```

## Manual trim and launch illustration

`trim-launch.html` illustrates the pilot-confirmed inverted start, arcing inflation, upright release and passive glide. It uses the payload GLB with a schematic canopy and explicitly keyframed motion, not the flight solver. No active control or propeller rotation is applied. Use **Start pose**, **Release pose**, pause/scrub and **Record animation** to inspect or export the 18-second sequence. Encode the downloaded WebM with `tools/convert/encode_showcase.sh input.webm ../docs/assets/flight-testing trim-launch`.

The Opale reference is `UEoCY5cZiRg`: French automatic captions reviewed at 6:37–7:53 (temporary knots and symmetry marks), 8:15–9:55 (released-brake inflation/lengthening), and 10:00–10:25 (full-brake inflation check). The pilot’s powered-off drum/glide method is personal testimony, not attributed to that video.

## Spot-landing replay

Open landing.html for the native AUTO approach replay. Drag to orbit, scroll/pinch
to zoom, pause or scrub. Flare close-up jumps to three seconds before the first
active PGFL sample. Slow flare defaults to 0.35x playback from one second before
flare through ground crossing. The aircraft's simulated motion is unscaled.

The earth-fixed 1 m floor grid and target rings convey translation. Throttle,
applied brake pull, downward sink rate and horizontal ground speed are plotted.
The speed panels show the final eight seconds; orange shading starts at the
logged flare entry, dashed lines give the speeds at entry, and a white cursor
tracks playback. Prominent readouts show current speeds and changes since entry.

The replay stops at first payload-CG ground-plane crossing, without impact or
collapse animation. Raw logs stay in Mission Planner. The final-only figure and
native flare annotation are generated by ../research/archive/viewer-studies/landing_final_report.py after
the replay packaging script. The article uses the interactive applet alone.

Build with npm run build, then run tools/publish_landing.py with the website's
assets/microagu-viewer destination to publish the bounded viewer bundle.


The landing applet accepts `?case=before` and `?case=after`. These select a
bounded matched replay pair (the previously published trial 1), with explicit
configuration labels. The plain URL shows the selected after configuration.
`research/archive/legacy-tools/report_landing_lateral.py` at the repository root audits the 12 native
logs and creates the summary plots and bounded replays; `publish_landing.py`
copies both excerpts with the viewer bundle. Raw logs stay in Mission Planner.

### Descending spiral research replay

`landing.html?case=spiral` shows the complete native descending-loiter, tangent-exit and flare experiment. [The study](../../research/archive/spiral-swoop-2026-10-13/README.md) includes the matched gentle-gust check, speed-retention plots and failed development cases. This recording is a spiral landing demonstration, not a validated high-energy swoop.

### Shared paraglider rendering

All canopy applets use `packages/rendering/src/aircraft/paraglider.ts`: an inflated, tapered, 28-cell
photo-inspired canopy with open leading-edge inlets, upper/lower skins and ribs.
The green/black/white/yellow panels follow the HobbyKing V2 reference photo.
Cell count, thickness, taper and cascade topology are illustrative, not measured.
Default effective arc span is 2.15 m, half arc 0.94598569 rad and chord 0.54 m;
the rigging applet overrides span/arc from each simulation's metadata. The
catalogue's 2.4 m span is not substituted into calibrated simulations.
The canopy reference height remains the five-panel mean used by the rigging study.

Suspension branches follow the independent recorded rigid-body transforms.
White risers and rings use a nominal 60 mm extension; their exact construction
and routing remain schematic. Rigging comparisons show 132 / 185 mm anchor
spacing, while the payload CAD itself remains the existing configuration.
No fabric dynamics or extra aerodynamic effects are introduced by this renderer.
Actual-flight replay intentionally remains payload-only; measured flight data
does not include canopy attitude. The damper diagnostic now offers both whole
aircraft and payload-closeup framing with linked orbit.
The FBD and launch illustration use the same canopy in their explanatory frame.

### Native article replays

`pitch-comparison.html` and `altitude-comparison.html` replay the original four
matched damper trajectories with a shared time slider, speed control, looping
playback and linked orbit. `simulation-replay.html?case=turn|calm|gust|crosswind`
replaces the exported turn/AUTO clips while retaining their source recordings,
telemetry plots, ground track and distance/altitude profile. No new simulation is
run in the browser. `pitch-fbd.html` remains an explicitly illustrative animation.
The real-flight replay links load the original converted flight logs, with
buttons for the full overview and the previously featured excerpts.

### Brake-driven fabric deformation

The simulated replays bend the existing aft 40% of the canopy mesh using the
recorded absolute effective left/right brake pull in millimetres (negative body
Y is left). Upper/lower skins and cell ribs deform together; no separate flap,
hinge, or control surface is added. Deformation begins with zero displacement
and slope at 60% chord and blends across the central 12% of each half span.
The visual rule exaggerates travel by 2.5×: 2.5 mm local downward trailing-edge
displacement per 1 mm effective pull, with a small forward curl. This keeps
braking visible at the normal overview camera distance. Inputs are bounded to 0–80 mm
for rendering, and missing/nonfinite values remain neutral. This is an
illustrative shape rule, not a measured fabric response or added aerodynamics.
GPU position/normal morph targets avoid rebuilding geometry each frame; the
line fan uses the identical deformation so its endpoints stay attached.
Neutral explanatory animations and measured-flight replays do not invent brake
or canopy motion. Joint-tour ghosts retain their reference brake setting.
