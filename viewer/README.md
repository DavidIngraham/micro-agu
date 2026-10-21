# MicroAGU viewer workspace

Shared Three.js rendering, recorded-data applets, and the browser ArduPilot + Rust
simulator. Existing HTML URLs are preserved for blog embeds. Physics lives in
`../simulation`; applets never implement or tune the aircraft model.

## Ownership

| Directory | Owns | Must not depend on |
| --- | --- | --- |
| `packages/contracts` | Validated truth/log formats, pose and mission types, worker protocol, attitude math | DOM, Three.js, applications |
| `packages/playback` | Interpolation, live truth buffering, truth/log → render-pose adapters | DOM, Three.js, rendering, simulator |
| `packages/rendering` | CAD and canopy geometry, asset loading, scene resources, cameras and route display | Simulator, applets, UI |
| `packages/sim-browser` | Rust binding adapter, ArduPilot transport, worker client, browser isolation and runtime integrity | Rendering, Three.js, applets |
| `packages/applet-ui` | Mount lifecycle, touch controls, timeline controls, comparison layout, charts and capture | Simulator, applications |
| `apps/applets` | Each explanation's data selection, annotations, layout and definition | Other applet implementations |
| `apps/simulator` | Simulator HTML, flight controls, default home/mission, input mapping, presentation loop | Private renderer or worker state |
| `assets` | CAD models, bounded presentation recordings, runtime configuration and pinned asset inventories | Experiment execution |
| `generated` | Rust JS/WASM/types, ArduPilot WASM and generated MAVLink dialect | Hand edits |
| `tools/build`, `tools/convert`, `tools/record`, `tools/publish` | Build manifests, data conversion, bounded applet capture and verified publishing | Tuning or flight-study execution |
| `../research/archive/viewer-studies` | Recording harnesses, gusts, scenario generation and analysis | Runtime imports from the web applications |

Packages are private npm workspaces with explicit exports and one lockfile.

The `trim-launch.html` URL now replays the recorded inverted launch, native
TAKEOFF climb and AUTO handoff. It uses the shared `FlightView`, pose interpolation,
rotor phase integration and playback controls; no autopilot or physics WASM loads.
Regenerate its 20 Hz excerpt from the packaged simulator with
`npx tsx tools/record/takeoff.ts` from this directory, review the result, then update
the pinned recording inventory before building/publishing. `takeoff/provenance.json`
records the firmware/model hashes, weather seed and assisted-launch limitations.
`npm run check:boundaries` enforces package direction; TypeScript checks every
application, package, tool and test. There is no implicit `shared` catch-all.

## Develop and test

```sh
cd viewer
npm ci
npm run assets:fetch       # Restore missing published excerpts by pinned SHA-256
npm run dev               # Open a page listed in apps/applets/registry.ts
npm test
npm run test:physics      # Frozen native/WASM reference trajectories
npm run test:sitl         # Real ArduPlane: modes, launch, mission and manual takeover
npm run build
npx playwright install chromium
npm run test:browser
```

Node 24 is used in CI. `npm run assets:fetch` never overwrites a local recording.
Large flight logs, source videos and fitting outputs stay outside this workspace.
The committed `assets/manifests/recordings.json` pins the published excerpts.
Builds report missing excerpts; publishing an incomplete applet fails before
writing anything. Replay pages never download ArduPilot or the physics WASM.

After intentionally regenerating and reviewing research exports, run
`npm run assets:inventory` to update the recording hashes. Builds do not rewrite
these expected hashes or regenerate experiments.

## Optional simulated lidar

`PhysicsConfig.lidar` enables the generic downward lidar in
`packages/sim-browser/src/physics/lidar.ts`; omitting it preserves existing
behavior. Start from `defaultLidar` (0.1–40 m, 20 Hz, 50 ms latency, 2 cm Gaussian
noise, 1% independent no-return probability). Mount offset is in payload FRD
metres. The beam intersects flat ground using payload attitude; output is slant
range, with 1 cm quantization and a 60° tilt limit. This is not a measured model
of a particular device, terrain reflectivity or vegetation.

Configure the existing ArduPilot SITL driver with `RNGFND1_TYPE=100`,
`RNGFND1_ORIENT=25`, and matching `RNGFND1_MIN/MAX` in metres. JSON `rng_1`
carries the result. Invalid returns are encoded above the configured maximum;
**the AP and sensor limits must match**. Missing samples are not represented as
zero distance or silently retained valid readings. The SITL backend identifies
its MAVLink sensor type as unknown even though the simulated modality is lidar.

Request MAVLink `DISTANCE_SENSOR` at the desired rate; it is included in the
generated browser dialect. Consumers must handle invalid/stale returns and
tilt-correct slant range using estimated attitude. Received measurements are
available as `Simulator.state.rangefinder`, including validity and reception
time. `Physics.lidarReading` is
diagnostic truth-side provenance, not a flight-controller sensor shortcut.
The browser configuration enables this sensor. `Simulator.start` configures the
matching AP rangefinder limits and requests the telemetry stream automatically.
The original raw-range comparison remains in
`research/active/landing-lidar-2026-10-20`.

## Native AUTO landing

The default circuit repeats at 60 m using `DO_JUMP`. Select **RTL · land** in
the flight-mode menu to leave the loop and enter its `DO_LAND_START` sequence.
The browser sets `RTL_AUTOLAND=2`, so ArduPlane proceeds directly to the landing
approach and switches back to AUTO. Imported missions without a landing-start
marker retain RTL's return-and-loiter behavior.

Open the flight panel, choose **Landing mission**, then **Upload** and **Fly
mission**. The example captures a clockwise 40 m orbit at 60 m, spirals to 30 m using
`MAV_CMD_NAV_LOITER_TO_ALT`, then exits tangentially north onto a 200 m final
approach and executes `MAV_CMD_NAV_LAND` at home. The circle centre is 280 m
south and 40 m east of home; its western edge is tangent to the approach line.
The loiter-to-altitude item sets param4=1 for cross-track tracking from the
actual exit. Capture uses one full orbit before descent. The mode display remains **AUTO** throughout.
Choose CRUISE or another mode to take over. Imported missions can also contain
ordinary NAV_LAND items. There is no browser LAND mode or land-ahead command.

ArduPlane owns navigation, the glide slope, LQI approach, power-off final and
brake flare. LQI remains eligible during AUTO landing approach, then hands back
to the native paraglider turn controller and brake mixer when AP_Landing enters
final. The browser never generates landing RC commands or switches to MANUAL.
The existing LQI gains and observer are unchanged.

The selected browser configuration enables an experimental native brake flare:
`LAND_FLARE_ALT=3` starts the power-off final; `PG_FLARE_HGT=1.3` starts latched
common-brake application. `PG_FLARE_PCT`, `PG_FLARE_RATE`, and `PG_FLARE_RES`
set nominal travel, application rate and remaining differential travel.
`PG_FLARE_GAIN` supplies bounded sink feedback around `PG_FLARE_SINK`.
Defaults in firmware leave these additional features disabled. This is still
simulation validation, not a flight-qualified landing tune.

Height follows JSON lidar → AP rangefinder → EKF terrain estimate → AHRS,
consumed directly in firmware. A valid terrain estimate is preferred for the
opted-in paraglider landing; unavailable terrain retains ArduPlane's normal
landing-height fallback, rather than assuming zero height. That fallback
requires a correct landing altitude and does not establish performance over
unknown terrain. No physics truth enters the controller. The browser uses
`PG_LAND` and `EKF_HAGL` telemetry only for display and the playback endpoint.
The generic lidar has 50 ms latency; this EKF range path assumes 25 ms, so
using fused HAGL does not eliminate every timing or bias error.

After estimator-confirmed near-ground, low-vertical-speed and slow-horizontal
motion, playback pauses. This avoids extending the rigid-canopy model into
unvalidated collapse/ground-roll dynamics. It does not force disarm. Reset
starts a new session. When HAGL is unavailable, the presentation endpoint uses fused altitude above
the mission landing elevation; this assumes the simulated flat landing field.

See `research/active/native-landing-2026-10-20` for the native mission tests.
The earlier `landing-ekf-2026-10-20` results describe the superseded browser
controller and are reproducible at micro-agu commit `55e5097`.

## Adding an applet

Create `apps/applets/NAME/index.html`, `definition.ts` and `applet.ts` together.
The simulator's HTML lives at `apps/simulator/index.html` beside `main.ts`.
The definition declares its stable public `page`, source `html`, TypeScript `entry`, exact asset requirements
(or pinned dataset groups), controls and initial camera category. Register it in
`apps/applets/registry.ts`; the build and publisher derive their page lists from
that registry. Applet-specific diagrams and scientific annotations stay beside
the definition. Reuse shared rendering, playback and controls through package
exports; do not import another applet's implementation.

Mount through `registerApplet(definition, scope => ...)`. Use the scope for frames,
timers, event handlers, workers, sockets, captures and owned render resources.
Use `ModelAssets` for per-mount CAD loading. Disposal cancels callbacks and pending
loads, terminates workers, releases GPU resources and closes sockets. The exported
handle supports `mount()` and `dispose()`; the browser tests repeatedly exercise
those operations. The simulator's Reset uses that lifecycle without a reload.

## State and clocks

Simulation truth (`microagu.state.v1`) and measured flight data
(`microagu.flight.v1`) remain distinct. Their playback adapters produce a NED/xyzw
`RenderPose`. Missing canopy/rotor observations remain null. Only rendering converts
NED to Three.js coordinates. The simulator worker owns physics time; one app-level
presentation loop samples the live buffer and updates aircraft and camera together.
The renderer does not read telemetry, execute missions, advance physics or own a clock.

## Build and publish

```sh
npm run build
npm run publish -- --destination /path/to/site/assets/microagu-viewer
# Or publish one application:
npm run publish -- --destination /path/to/site/assets/microagu-viewer --app simulator
```

The build combines Vite's dependency manifest, worker output inventory and each
applet's declared assets into `dist/applet-manifest.json`. Publishing verifies every
file hash before copying the selected dependency closure. It never parses minified
JavaScript. Existing Python `tools/publish_*.py` commands delegate to this publisher.
Only publishing the simulator includes its worker, WASM and isolation service worker.

Source folders do not determine published URLs. The build emits each HTML file at
its definition's `page` (for example `pitch-fbd.html` or `simulator.html`) and makes
its compiled asset URLs relative to that location. The dev server serves these
same routes and redirects source HTML paths to them, preserving query parameters.
Blog links and iframe embeds continue to use `/assets/microagu-viewer/<page>`;
link to `apps/applets/<id>/index.html` on GitHub only when referring to source code.

Generated Rust bindings are produced by `../simulation/tools/generate.sh` into
`generated/physics`. Firmware builds write to `generated/ardupilot`; resolved runtime
configuration/provenance live in `assets/runtime`. The public URLs remain `wasm/*`.
Browser startup checks the recorded hashes and uses versioned runtime URLs.

See [simulator operation and rebuilding](SIMULATOR.md),
[individual experiment and capture recipes](docs/RECIPES.md), and
[the reusable physics package](../simulation/README.md).
