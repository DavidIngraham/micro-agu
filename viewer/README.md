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

## EKF landing demonstration

Once flying above 15 m, click the main **MODE** display and choose **LAND · ahead**.
The simulator marks a landing point ahead on the ground and map. This is not RTL:
it does not return to the launch circle. Select any other mode to cancel.

The worker runs shared `session/landing-guidance.ts`: existing LQI in CRUISE on
approach, then an explicit MANUAL handoff at 3 m AGL to the bounded common-brake
flare controller. The HUD distinguishes approach, final, flare and touchdown.
Pilot sticks, altitude and controller changes are locked while landing owns the
inputs; the mode picker remains available for cancellation.

Height follows JSON lidar → AP rangefinder → EKF terrain estimate →
`AP_AHRS::get_hagl` → `EKF_HAGL` named-float telemetry. Negative values invalidate
the estimate. Guidance propagates only the short telemetry age using fused
vertical velocity. It does not feed simulator height or canopy truth into the
controller. Above 10 m, home-relative EKF altitude is a fallback on this flat
simulated field. Below that, unavailable terrain height aborts the sequence.
A pre-final abort stops the descent and requests a climb; after the manual
handoff it keeps power off and releases control to the user. It is not a fully
qualified autonomous go-around system.

The generic sensor still has 50 ms latency while this AP EKF range path assumes
25 ms; using EKF HAGL does not eliminate that mismatch or all measurement bias.
The 10 Hz current-output HAGL stream and 20 Hz navigation stream are checked for
freshness. LQI gains and observer parameters are unchanged.

After near-ground, low-vertical-speed, slowed-horizontal-motion confirmation,
the simulator pauses; reset starts another flight. This is a simulation endpoint,
not confirmation that ArduPlane has disarmed. No force-disarm is sent. This endpoint avoids presenting the rigid
canopy's unvalidated collapse/ground-roll dynamics as a credible rollout.
See `research/active/landing-ekf-2026-10-20` for flight traces, failure cases and
pre-contact landing metrics. This remains a browser experiment, not a new
flight-qualified ArduPlane LAND mode.

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
