# Routed brakes and 185 mm spreader — 2026-10-15

See [the article](../../../docs/rigging-refinement.md) and [protocol](PLAN.md). This is exploratory system identification and hardware sensitivity analysis, not independent validation or flight firmware release.

## Reproduce

Install Rust and uv, then run `uv sync --project research --locked` from the repo root. From this directory, use the research Python environment:

```sh
export PATH="$(realpath ../.venv/bin):$PATH"
python prepare.py
python verify.py
python study.py
python refine.py
python sensitivity.py
python predict.py
python report.py
```

`prepare.py` checks the installed Rust model. Scripts import `microagu_sim.compat` and its packaged trim seeds. `verify.py --geometry-only` runs geometry and frozen C++ numerical-reference checks without flight data or replacing published results. The source archive and hashes remain historical evidence; current scripts do not extract or compile them. Routing lives in microAGU’s reusable Rust simulator.

Set `MICROAGU_REPLAY_DATA` to the existing turn-model-spike dataset directory. Required files are `2026-01-03-causal-input.npz`, `2026-02-15-causal-input.npz`, and the corresponding `*-propulsion-9x45.npz` files. Input archive hashes are in `input-hashes.json`; raw inputs are not duplicated here. See [data provenance](../../../docs/data-and-methods.md). The default local path in the script supports this workstation only. Predictions and geometry checks can run from the source archive without raw logs; the final convergence replay in verify.py requires them.

The published JSON files retain all 936 replay rows: 480 fit, 384 evaluation, and 72 ring-height/initialization sensitivities. The 24 initial height-sensitivity runs use the first-round selected fit (1.4× damping); sensitivity-final.json uses the final fit with alpha 0.25/0.45. Do not combine them as if they differed only in height. All rows are finite and all selected samples have valid_fraction=1.0. `selected.json` is the first-round ranking; `selected-final.json` includes the declared boundary extension. `summary.json` preserves split and canopy initialization rather than pooling them silently.

`predict.py` generates 18 distinct 80-second simulated pulse responses, then reuses them in the 15 applet pair/scenario combinations. It solves separate straight-level trims, applies the same commanded increments, and enforces the existing actuator rate. No wind or feedback controller. Poses contain [time, 26 state entries, RPM, voltage, throttle, left_mm, right_mm], sampled every 0.1 s. Traces contain [time, relative height, payload pitch rate deg/s, unwrapped ground-course change deg, mean effective pull mm, differential effective pull mm, airspeed m/s]. World frame NED; quaternions xyzw. Three-dimensional paths are simulated payload positions.

The research profiles preserve legacy defaults; `routed_research` and `wide_research` must be selected explicitly. The widened configuration holds canopy aerodynamics fixed. Flattening and induced-drag cases are additional hypotheses in predict.py. Neither the fitted damping boundary nor the canopy transfer factors are measured physical parameters.

## Geometry and hardware provenance

The [photo audit](../tension-stiffness-2026-10-15/photo-analysis/README.md) separates CAD slot centers, manufacturer span, inferred photo dimensions and unmeasured ring geometry. The earlier [brake-routing sensitivity](../brake-routing-2026-10-15/analyze.py) is a simplified vertical-ring/pitch-only calculation; the new three-dimensional implementation follows a straight riser toward a canopy support point. Do not equate their derivative numbers without accounting for that change.

`hardware.json` records the saved SolidWorks configuration and file hashes. The repository part contains both Default and HK-estimate-185mm configurations; the source assembly has not been switched and no manufacturing export is implied. Ground trim, mass/inertia, ring contact, canopy arch and clearance remain measurements to make on the installed hardware.

## Checks and limits

Disabled C++ derivatives match the archived legacy implementation bit-for-bit. Tests cover common-world-rotation invariance, symmetry, the zero-height pitch limit, slack-before-engagement, and substep convergence. Existing simulator verification and JSON backend tests pass. The viewer build/type-check and existing unit tests are run separately. No native flight mission or hardware flight of the new profiles is claimed.

Analysis, code and article prepared with AI assistance. Photos retain their original source pixels; annotations and scientific plots are reproducible source assets.
