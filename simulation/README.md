# MicroAGU simulator

The authoritative runtime is Rust. The same `sim-core` implements full 9-DOF
canopy–payload dynamics for native ArduPilot JSON, Python research, and browser
WASM. ArduPilot retains its lightweight built-in model and flight controller;
the high-fidelity external simulator is optional, like JSBSim for Plane.

## Layout and ownership

| Path | Responsibility |
| --- | --- |
| `crates/sim-core` | Named configuration, validation, forces, RK4, actuators, sensors, contact and assisted launch; no I/O |
| `crates/sim-io` | Configuration schema, recording format and configuration hashes |
| `crates/sim-sitl` | Standalone UDP JSON backend and PWM mapping |
| `crates/sim-cli` | Configuration validation, schema/type generation and batch replay |
| `crates/sim-python` | PyO3 extension, generated Python stubs, NumPy batch API and packaged SciPy compatibility helpers |
| `crates/sim-wasm` | wasm-bindgen adapter and generated TypeScript API |
| `data/presets` | Explicit selected configurations; no fitting at runtime |
| `schemas` | Generated JSON Schema |
| `tests`, crate-local `tests` | Contracts, regression and independent numerical parity |
| `tests/parity` | Immutable numerical outputs from the retired C++ implementation |
| `../research` | Experiments, fitting, controller studies and historical evidence |

Root Python import shims have been retired. Code imports `microagu_sim`;
historical SciPy utilities are explicitly under
`microagu_sim.compat`. Their packaged data retains historical profiles and trim
references. No runtime imports experiments or modifies `sys.path`.

## Development

Install current stable Rust, Python >=3.10 and uv. Development was verified on
Ubuntu/WSL with Rust 1.99.0 and Python 3.12. Other native platforms are not yet
verified. No C++ compiler or Eigen installation is required.

```sh
cd simulation
uv sync --locked
cargo test --locked --release --workspace
cargo clippy --locked --workspace --all-targets -- -D warnings
uv run pytest -q
uv run python examples/replay.py
```

`uv sync` builds the Python extension using maturin/Cargo. `Cargo.lock` pins Rust
dependencies; `uv.lock` pins the development environment. Research has its own
uv project and lockfile. Wheels contain the native library and compatibility
resources and work outside this checkout.

## API and numerical contract

```python
from microagu_sim import Simulator

sim = Simulator(open("data/presets/as-built-trout-lake.json").read())
state = sim.step([0.4, 0.0, 0.0], [0.0, 0.0, 0.0], 0.005)
```

Commands are throttle fraction and left/right effective brake pull in **metres**;
wind is NED metres/second. State packs payload then canopy, each as position NED,
body-to-world quaternion **xyzw**, velocity NED and angular velocity in world
coordinates. Bodies use FRD axes. `step` advances exactly one requested interval;
a failed step does not mutate state, rotor state, launch phase or time.

`Kernel.replay_numpy` accepts contiguous float64 arrays and returns an owned
Nx27 array (26 state values plus RPM), without Python objects per sample. Its
legacy replay convention includes the initial state as row zero and ignores the
last command. `Simulator.step` and the CLI replay instead return interval
endpoints. Actuator/PWM conversion is explicit in the SITL adapters; do not apply
it twice when using the low-level kernel.

Rust validates finite values, dimensions, inertia and supported configuration
versions. `sim-core::Parameters` is the authoritative coefficient definition;
Python defaults, JSON Schema and TypeScript configuration types derive from it.
Packed parameter order exists only for historical compatibility.

## Native ArduPilot

```sh
# In simulation/, first terminal:
cargo run --release -p sim-sitl -- data/presets/as-built-trout-lake-sitl.json --ground
# In a built ArduPilot checkout, second terminal:
Tools/autotest/sim_vehicle.py -v ArduPlane -f paraglider --model JSON:127.0.0.1 \
  --add-param-file /path/to/micro-agu/simulation/sitl.parm --console
```

Initialize and arm, then press Enter in the backend terminal to release. Without
`--ground`, the backend starts in translating airborne trim. `--release` skips
the operator release gate; `--port` selects the localhost UDP port. Duplicate
servo frames return the cached response without advancing time. Discontinuous
frames are rejected. The compatibility Python adapter remains available for
historical recording/experiment hooks:

```sh
uv run python -m microagu_sim.compat.json_backend --profile as_built --ground
```

Firmware source: [paraglider-browser-9dof](https://github.com/DavidIngraham/ardupilot/tree/paraglider-browser-9dof).
Firmware uses simulated measurements through its own AHRS/EKF; rendering uses
truth. Standalone observer/LQI experiments remain separate from ArduPilot.

## Generated bindings and browser build

```sh
rustup target add wasm32-unknown-unknown
cargo install wasm-bindgen-cli --version 0.2.114 --locked
sh tools/generate.sh
```

This generates Python `.pyi` files from PyO3 declarations, JSON Schema and
TypeScript configuration types from Rust definitions, and browser JS/WASM/`.d.ts`
from wasm-bindgen. No handwritten foreign-function declarations or heap offsets
are required. The wasm-bindgen CLI version must match Cargo.lock. CI checks
generated declarations for drift. See [viewer build and deployment](../viewer/SIMULATOR.md).

## Physics provenance and migration gates

The port retains the as-built **132 mm** spacing, propeller tables, RK4 method,
existing-log calibration and firmware tune. No physics retuning accompanied the
language migration. The [calibration study](../research/archive/as-built-simulator-2026-10-15/README.md)
uses Hood River and Trout Lake logs only. The C++ extraction's original hashes
remain in [the migration manifest](../provenance/simulator-migration.json).

```sh
uv run pytest tests/parity -q
cd ../viewer
npm run test:physics
npm run test:sitl
```

Tests compare randomized derivatives, airborne trajectories and 5,000 launch /
flight / contact steps against frozen C++ outputs. The fixture records the original
source revision, hashes and complete input configuration. Recover historical
source from Git when investigating a discrepancy; never regenerate these
expected values with Rust. Saved browser fixtures remain independent
C++ reference traces and are not overwritten by the Rust configuration exporter.

Ground contact, scalar inflation and prescribed hand motion remain development
approximations. Existing logs do not identify cloth dynamics, inflation pressure,
contact friction or launch timing. The model is not a validated training model.
# Dryden gusts

`sim_core::atmosphere::Dryden` provides seeded MIL-F-8785C translational gust
filters, also exported as `microagu_sim.Dryden` and the generated TypeScript
`Dryden` class. `step(dt, speed, lengths, sigma)` takes SI units: seconds,
mean-air-relative speed in m/s, three length scales in metres, and three component
RMS intensities in m/s. Outputs are along-wind, cross-wind and down velocities.
Callers add mean wind and rotate these axes into their world frame.

The longitudinal filter is first order; lateral and vertical filters are second
order. State propagation and process-noise covariance are discretized exactly,
including a small-step series to avoid cancellation. A nonzero seed reproduces
the same time-step sequence; changing timestep preserves statistics, not the
individual random path. Zero dt does not consume randomness. Speed has a 0.5 m/s
floor; the frozen-field model is not a calibrated model of stationary turbulence.

The API accepts independent axis lengths/intensities. The browser exposes a
shared gust size and a vertical/horizontal RMS ratio, with 20 m and 0.5 defaults.
It rotates gusts into NED, uses speed relative to the mean airmass, and ramps mean
wind and gust intensity over two seconds. Thermals remain a separate spatial
field. This is velocity turbulence only: no canopy-scale gradients, angular gusts,
terrain rotor, automatic altitude scaling or flight-data calibration is claimed.

The Rust tests verify stationary variance and analytic autocorrelation at two
timesteps; `viewer/tests/integration/dryden.ts` checks Python/WASM sequence parity.
Reference: https://www.mathworks.com/help/aeroblks/drydenwindturbulencemodelcontinuous.html


# Offline grey-box identification

The reusable [`microagu_control.sysid`](../control/src/microagu_control/sysid/README.md)
package provides prepared-log session validation, Rust recorded-input replay,
bounded fitting, optional initial-state/wind corrections, residual diagnostics,
and separate reserved-episode validation through a CLI. See the
[Trout Lake comparison](../research/active/greybox-pipeline-2026-10-18/README.md) for
baseline reproduction, synthetic parameter recovery and measured limitations.

## Offline controller pipeline

The [CLI pipeline and diagram](../control/src/microagu_control/pipeline/README.md) connect measured aircraft profiles, existing sysid, gain synthesis, observer-in-loop screening and paired ArduPilot/EKF campaigns. It exports compatible gain files without modifying the deployed simulator. The [initial pipeline study](../research/active/controller-pipeline-2026-10-18/RESULTS.md) records the current tuning decision and repeatability limits.

Offline identification and gain synthesis now belong to the separately installable [control package](../control/README.md), using `microagu_control.sysid` and `microagu_control.pipeline`. Selected aircraft inputs and gains live in [aircraft/microagu](../aircraft/microagu/README.md).
