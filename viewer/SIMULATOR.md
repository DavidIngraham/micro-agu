# Browser flight simulator

The [live flight simulator](https://davidingraham.github.io/assets/microagu-viewer/simulator.html)
runs MicroAGU's ArduPlane firmware locally in WebAssembly. No vehicle connection,
WebSocket server, or account is needed. Use a current browser with WebGL2,
SharedArrayBuffer and service worker support over HTTPS or localhost. The first
visit reloads once to enable cross-origin isolation on GitHub Pages.

## Flying

The example circuit uploads and starts automatically in **AUTO** after sensor
initialization while upright. It then inverts for setup, swings the canopy overhead,
takes two steps and tosses upright. ArduPlane's actual TAKEOFF mode detects the
launch and controls the climb, then the app hands over to AUTO at 15 m above home.
Click the **mode display** to choose a mode;
the displayed value follows confirmed telemetry. The launch button becomes
**Reset simulation** as launch begins. Reset starts a fresh default mission.
The **Mission & weather** panel is closed by default.

**Trail** toggles a gold flight path. History is recorded even while hidden and
cleared on reset. It retains up to 4,096 points, sampled at most four times per
simulation second and only after at least 0.5 m of movement.

**Plots** toggles two compact traces covering the last 120 simulation seconds:
EKF altitude above home (green) and commanded altitude (dashed gold), plus signed
ArduPilot cross-track error in metres. Axes scale automatically; cross-track is
centered on zero. Navigation errors are shown in AUTO, LOITER and CRUISE;
manual modes leave gaps. History accumulates while hidden and clears on reset.

Upright initialization runs SITL at **4× speed**, then waits for confirmation of
`SIM_SPEEDUP=1` before launch. EKF3 still processes simulated sensor measurements;
its alignment requirements and normal arming checks are unchanged. In a local
cached-firmware test, initialization fell from about 21 s to about 6 s. Actual savings
depend on the device; download/compilation and the launch sequence still take time.

* **MANUAL**: direct brake and throttle inputs.
* **FBWA**: native heading-rate steering; pilot controls throttle.
* **FBWB**: steering with automatic altitude and power control.
* **CRUISE**: centered steering holds heading and altitude.
* **AUTO**: follow the mission, with left/right steering nudges.
* **LOITER**: orbit the current position.

The right control moves left/right in assisted modes. MANUAL exposes a lower
semicircle: pull backward/down for symmetric brakes; forward has no brake command.
Release centers it. The **vertical throttle** slider retains its setting in MANUAL
and FBWA. In FBWB/CRUISE it becomes a sprung **ALTITUDE** lever: up commands
climb, down commands descent, and release centers it and holds the current height.
ArduPlane controls power. Keyboard: left/right arrows steer (including AUTO/LOITER
nudges), down pulls both brakes in MANUAL, and W/S adjusts manual throttle or
commands climb/descent in FBWB/CRUISE. Release W/S to hold altitude.
AUTO/LOITER nudges use ArduPlane's native roll-only stick mixing (`STICK_MIXING=4`);
releasing the stick lets navigation resume. The altitude lever maps to AP's pitch
RC input with centered neutral; it does not write motor output.
A lost pointer, window blur or hidden tab centers the stick. Browser
background throttling may pause or interrupt simulation; keep the tab visible while
flying. **Reset simulation** creates a fresh aircraft and clears its mission.

Click **ALT / HOME** to enter a target between 5 and 300 m above home. MANUAL/FBWA
switch to CRUISE when Set altitude is pressed (explained in the input panel).
In AUTO this changes the current leg only; later waypoints retain their heights.
The target shown comes from ArduPlane's navigation telemetry, and confirmation
requires both a command acknowledgement and matching controller demand. The browser
firmware includes a small GCS fix applying altitude requests to CRUISE/FBWB's separate
altitude demand, in addition to the waypoint altitude.

## Missions

Load QGroundControl `.plan` (SimpleItem) or QGC WPL 110 `.waypoints` files, or use the
example circuit. Upload waits for ArduPlane's mission acknowledgement. After launch,
**Fly mission** selects item 1 and AUTO. Select another mode to take over manually.
The route preview and item counter show the loaded route and current sequence.

Supported commands: waypoint, loiter unlimited/turns/time, RTL, land and takeoff.
Supported altitude frames: global MSL and relative to the simulator home. Terrain
altitudes, complex surveys, non-navigation commands and ambiguous coordinates are
rejected instead of silently changed. WPL item 0 is treated as planned home and
replaced by the simulator home. Home is fixed at 45.997° N, 121.536° W, 600 m MSL;
imported coordinates are **not relocated**. Inspect the route before uploading.
The example ends in unlimited loiter rather than an uncommanded landing.

## Wind and camera

Use **Mission & weather** to choose wind speed (0–10 m/s), the bearing wind comes
from (0° north, 90° east), and vertical air (−3 to +3 m/s, positive upward), then
select **Apply weather**. Add turbulence (0–2 m/s horizontal RMS; vertical RMS is
half that) and a thermal (0–5 m/s centre lift, radius 5–200 m). Set the thermal's
north/east position relative to home or use **Place at aircraft**, then apply.
Zero turbulence or thermal lift disables that component. New sessions and resets
start with 0.2 m/s turbulence and a 1.5 m/s thermal of radius 60 m, centred 300 m
north of home on the northbound AUTO leg. Background wind remains zero.
The compact **WIND TRUTH** readout shows instantaneous local horizontal wind speed
and its from-bearing, plus upward/downward air speed, including gusts and thermals.
An enabled thermal appears as a translucent red column with rising tracers. Its
radius and position follow the applied settings; zero lift hides it. Tracer motion
is exaggerated for visibility, and the displayed column extends 200 m above ground
(the synthetic weather field itself has no height cutoff).

The reusable browser environment samples seeded, correlated Gaussian gusts with a
two-second correlation time and a stationary Gaussian thermal column. At one
radius from its centre, thermal lift is 1/e of its centre value. These are synthetic
weather scenarios, not flight-log fits or a boundary-layer model; there is no
thermal drift, compensating sink ring, or altitude-dependent thermal structure.
The same combined wind sample feeds Rust dynamics, JSON airspeed/wind and render
truth at every physics step. Existing aircraft coefficients remain unchanged.

Chase rendering uses a continuous buffered truth clock for the aircraft and camera,
so worker message timing cannot rewind the pose. The chase camera moves with the
sampled payload and smooths its offset; OrbitControls only controls the orbit view.

## Model and visual limits

This version uses the **full 9-DOF canopy–payload research backend**. Its Rust
`sim-core` is compiled into `sim_wasm_bg.wasm`; ArduPlane runs in a
separate WASM module using upstream's unchanged `JSON:127.0.0.1` socket transport.
A dedicated worker runs both modules. The microAGU adapter intercepts the local
UDP peer at `127.0.0.1:9002` in Emscripten's `SOCKFS` layer and places each Rust
sensor response into that socket's receive queue. ArduPilot uses its ordinary
`sendto`/`recvfrom` calls and JSON timestamps for lockstep; no packets leave the
browser, and no WebSocket server or custom native physics ABI is required.
The adapter preserves the native brake rate limits, motor dynamics, IMU lever arm
and RK4 substeps. Flight control remains in ArduPlane.

The primary heading-rate controller remains the deployed baseline. The firmware's
custom-controller frontend now receives its heading-rate reference and can request
symmetric brake through the shared allocator. Returning from custom steering
matches the previous differential demand within the brake limits, then releases
excess matching correction with a 0.25 s time constant. The optional coordinated
LQI backend additionally owns throttle and both brakes; see the selection notes
below and the [firmware contract](https://github.com/DavidIngraham/ardupilot/blob/paraglider-browser-9dof/libraries/AP_CustomControl/MIMO.md).

This deliberately depends on the socket internals of the pinned **Emscripten
6.0.8**, rather than maintaining an ArduPilot transport patch. The build exports
the existing `SOCKFS` runtime through a deployment linker setting. SDK upgrades
must pass the adapter, real-firmware and browser restart tests. Other socket
destinations retain Emscripten's normal behavior. SharedArrayBuffer and isolation
are still needed for ArduPilot's pthreads. The former `sitl-wasm-json` branch is
historical; the active firmware branch is rebased directly onto upstream master.

The app adapts `microagu.state.v1` truth to a render pose for the shared
Three.js canopy, SolidWorks payload and suspension tools: independent payload and canopy positions/quaternions,
rotor RPM, brake travel and joint rotation/rates. Fabric deformation is illustrative.
The model uses as-built 132 mm riser spacing and the bounded existing-log fit in
[the calibration report](../research/as-built-simulator-2026-10-15/README.md).
Reduced flat-plane contact retains the two-body constraint. Scalar overhead
inflation and prescribed hand motion approximate launch; neither cloth pull-up nor
canopy collapse is modeled. Launch/contact constants are assumptions, not log fits.
The assisted inverted trajectory is implemented in `sim-core/src/launch.rs`, with
generated Python and TypeScript bindings. Both body poses and their derivatives
are used for sensor truth; the canopy starts laid out behind the payload and rises
overhead. At release, the position/velocity constraints match the normal solver.
Power remains zero during handling. The
assumed inverted hand height is 1.2 m, rising by 0.6 m through the swing and
another 0.24 m through the toss. Release is at 5.8 s with 4.5 m/s forward and
0.6 m/s upward velocity. Quintic velocity ramps blend the walk into the toss;
the follow camera also blends out of its fixed handling view during inflation.
TAKEOFF engages at release, so stabilization cannot
pull a brake against the operator during inverted handling. The browser sets
TKOFF_ALT=15, TKOFF_DIST=200, TKOFF_LVL_ALT=5, TKOFF_LVL_PITCH=5,
TKOFF_THR_MINSPD=2, TKOFF_THR_MINACC=0, TKOFF_THR_DELAY=0 and
TKOFF_THR_MAX=60. Normal GPS/attitude launch checks remain enabled. The app sends
no RC climb throttle; ArduPlane supplies it. These are simulator-specific launch
assumptions rather than new flight-log measurements. The
legacy reduced surface-launch API remains available for frozen parity fixtures.
Terrain is an illustrative flat field, not a map or elevation database. Grid and
launch-circle markings are shaded on one surface to avoid distant depth fighting.
The simulator is a development aid, not a validated flight-training model.

## Build and verify

The exact source revision and SHA-256 hashes are in `assets/runtime/build.json`.
Firmware source is public in the ArduPilot fork; the external physics source is
in this repository's `simulation/` directory. Project binaries and source
are GPL-3.0-or-later; Rust dependencies retain their own licenses. `assets/runtime/COPYING.txt` accompanies the deployed build.

```sh
# Node >=22; Rust + wasm32-unknown-unknown; wasm-bindgen-cli 0.2.114
# Emscripten 6.0.8 is needed only when rebuilding ArduPilot.
# Checkout DavidIngraham/ardupilot branch paraglider-browser-9dof.
# From microAGU: uv sync --project simulation --locked; then cd viewer.
# Use simulation/.venv/bin/python (uv sync --locked) for configuration export.
# Saved C++ parity fixtures are preserved independently.
PYTHON=/path/to/python bash tools/build/build_highdof_wasm.sh /path/to/ardupilot-browser-9dof
bash tools/build/build_sitl_wasm.sh /path/to/ardupilot-browser-9dof
cp ../COPYING.txt assets/runtime/COPYING.txt
npm ci
npm test
node --import tsx tests/integration/physics.ts
node --import tsx tests/integration/sitl.ts
npm run build
python3 -m http.server 5173 --directory dist
# Open http://localhost:5173/simulator.html
```

The parity test compares 1,200 physics steps and 49 native Python/C++ checkpoints,
including both bodies, sensors, rotor, actuators and joints. The integration test
boots the packaged WASM, confirms MANUAL/FBWA/CRUISE, uploads a mission, arms with
normal prearm checks, launches from ground, climbs into AUTO, checks steering and validates
articulated truth. `tools/build/generate_mavlink.py` regenerates the small browser
dialect from the firmware's pymavlink installation. No control algorithms are
reimplemented in JavaScript.

Publish the generated page and dependencies into the existing Pages repository:

```sh
python3 tools/publish_simulator.py ../../DavidIngraham.github.io/assets/microagu-viewer
```

Review and commit the generated files in that repository, then push its deployment
branch (`master`). The scoped `simulator-sw.js` supplies COOP/COEP headers for Pages;
it has no asset cache and leaves external requests alone. All simulator dependencies
are same-origin. Do not embed the simulator in an ordinary non-isolated iframe;
link to its standalone page.
# Coordinated controller selection

The Settings panel offers **PG + TECS** (default) and **LQI · experimental**.
The status distinguishes selection from firmware-confirmed active control. LQI
automatically waits through assisted launch, MANUAL, FBWA and landing, where the
baseline retains ownership. Switching back to PG + TECS does not restart the
simulator or replace the EKF.

The new LQI uses all three inputs: throttle, symmetric brake and differential
brake. Its final left/right commands pass through a shared travel/slew allocator
and servo calibration, bypassing the legacy mixer. It uses onboard estimates
and an internal canopy observer, never simulator truth. Its matrices are generated
by `simulation/tools/design_mimo.py` from the 132 mm as-built Rust model. This
initial fixed operating-point design is distinct from the archived scheduled
two-input research controller; the switch is for simulation comparison, not a
claim that it outperforms PG + TECS or is ready for flight.

The 2026-10-17 original-observer backport study adds `CP3_HRTC`, a climb-reference
filter applied before altitude-reference integration and feedforward. Zero disables
it; the accepted local research configuration uses 0.7 s with the existing ±0.5 m/s
limits. Gains and sensing are unchanged by this backport; it does not enable a
pitot sensor or canopy-truth feedback. See the
[paired results](../research/lqi-backport-2026-10-17/RESULTS.md).

The preceding public simulator release used the separately validated
`responsive_softspeed` retune with `CP3_HRTC=0.7`. Its observer, sensing, 10% common
trim and 132 mm physics remain unchanged. Two tuning passes and 168 SITL runs
included fresh paired cases, AUTO/CRUISE missions and ten-minute recovery checks.
See the [retune results](../research/lqi-filter-retune-2026-10-17/RESULTS.md).
The earlier crosswind comparison had unmatched gust incidence; that test was
corrected before retuning, and its original results retain an erratum.

The current browser release adds feasible turn trims, corrected anti-windup and
observer equilibrium recentering with the same feedback and observer gains. Its
LQI rate cap is 35 deg/s after paired disturbance and transition qualification,
and a two-second engagement blend addresses the tested
FBWA-to-LQI brake transient. PG + TECS remains the default. See
[release scope and validation](LQI-FEASIBLE-TURNS.md).

To reproduce the published controller, use ArduPilot base
`010ada6a4f0b26910fc18ecca176d2e65c5ed9d7` plus
`assets/runtime/lqi-longitudinal.patch`; validation details and firmware hashes are in
`assets/runtime/lqi-feasible-turns-validation.json` and `assets/runtime/build.json`.
This complete patch replaces the older filter-tune patch; do not apply both.
Apply that patch in a clean checkout before running the firmware build command
above. The local ArduPilot research checkout still contains separate zero-trim
and EKF experiments with a different gain dimension. Rebuilding that checkout
does not reproduce the packaged public runtime. Its research changes were
preserved rather than overwritten during publication.
# Gust controls

The web simulator defaults to the shared Rust Dryden velocity-gust model. Settings
expose **Gust model**, **Gust size** (1–200 m), **Vertical proportion** (0–200%),
and **Turbulence** (horizontal component RMS in m/s). Larger gust sizes produce
longer disturbances at the same speed. Defaults remain 0.2 m/s strength, with a
20 m gust size and 50% vertical proportion. Apply wind updates the configuration;
mean wind and intensity ramp over two seconds in Dryden mode.

Legacy turbulence preserves the original fixed two-second OU model for previous
experiments. Size and vertical controls are disabled in that mode. Existing API
callers that omit `turbulenceModel` retain legacy behavior. The explicit Rust
Dryden binding is available to Python experiments as well as the browser.
The autopilot firmware and 35 deg/s controller tune are unchanged by this weather
update; earlier controller qualification results describe the legacy gust model.


## Current longitudinal envelope

The optional LQI uses 0.75 m/s climb and 1.0 m/s descent limits, with the existing
0.7 s reference filter and unchanged gains and sensing. The firmware also fixes
voltage-compensated throttle feedback to prevent false anti-windup at reduced
battery voltage. PG + TECS remains the default. See [qualification results and
reproduction](LQI-LONGITUDINAL.md); the current complete source patch is
assets/runtime/lqi-longitudinal.patch, replacing the earlier patch recipe.
