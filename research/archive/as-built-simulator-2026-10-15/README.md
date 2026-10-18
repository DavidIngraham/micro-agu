# As-built simulator, 15 October 2026

The external simulator now lives in [simulation](../../../simulation/README.md), not
inside ArduPilot. The built-in ArduPilot model remains lightweight and articulated;
the optional external model provides the full two-body nine degrees of freedom.
Native UDP JSON and browser shared-memory JSON use the same C++ physics source.

## Existing flight evidence

Only the existing Hood River (3 January) and Trout Lake (15 February) logs were
used. The four preprocessed input hashes are recorded in `input-hashes.json` and
checked against the earlier published rigging study. No new flight was introduced.
The measured riser attachment spacing is **132 mm**. The 185 mm proposal is not
used. Flight-specific mass, CG, inertia, brake slack and preload remain unchanged.
Routing is enabled, with the existing uncertain 60 mm ring-height assumption.

Run `python research/archive/as-built-simulator-2026-10-15/calibrate.py` after building the native kernel;
set `MICROAGU_REPLAY_DATA` to the directory containing the four NPZ inputs. The
script reuses the published observation and canopy-initialization method.
The script writes `calibration.json` beside its results; it never overwrites the
runtime configuration. After reviewing a new fit, explicitly copy that artifact
to `simulation/as-built-calibration.json` to promote it. The retained runtime copy
continues to reproduce the deployed model.

The fit varies
only a common pitch/yaw joint damping multiplier in [1, 1.5, 2, 2.5, 3]. Brake
authority is unchanged. The corrected yaw geometry retains the previously fitted
stiffness product; this is not a claim that geometry alone identifies stiffness.

Ten-second fit windows use alternating starts from each flight. The other starts
are checked over 30 seconds at initial canopy angles of 0.25, 0.35 and 0.45 rad.
The objective normalizes roll/pitch/yaw rate, bank and climb errors against the
legacy model on each fit window; bank and climb have weight 0.25, rates weight 1.
These are historical held-out windows, not independent validation flights.

| Mean check-window RMS | Legacy | As-built fit |
| --- | ---: | ---: |
| Roll rate, °/s | 13.46 | 13.23 |
| Pitch rate, °/s | 19.40 | 17.47 |
| Yaw rate, °/s | 26.46 | 23.71 |
| Bank, ° | 5.73 | 5.67 |
| Course, ° | 59.65 | 57.88 |
| Climb, m/s | 0.3390 | 0.3395 |

The selected damping multiplier is **3, at the search bound**. This is a bounded
empirical fit, not a uniquely identified damping coefficient. Large course error
and unknown canopy initial state remain limitations. Results are in `fit.json`,
`checks.json`, and [the deployed calibration](../../../simulation/as-built-calibration.json).
The firmware controller gains have not been retuned by this physics fit.

## Ground and launch scope

`simulation/crates/sim-core/src/surface.rs` (ported from the study’s original C++ implementation) adds flat-plane, inelastic contact impulses for payload and canopy,
with bounded Coulomb friction and the payload–canopy point constraint retained.
It uses the kernel's payload inertia and canopy added-mass matrix. The two contact
radii (100 mm payload, 30 mm canopy) and friction coefficient 0.5 are assumptions;
there is no terrain, detailed landing gear, fabric collision or collapse model.

Launch is an **assisted overhead inflation and hand release**, not a cloth pull-up
from a laid-out canopy. The launcher prescribes a three-second ramp to 4.5 m/s and
raises the payload to 1.2 m. A scalar pressure fraction fills with ram airflow and
an assumed 0.8 s time constant; release requires at least 95% fill. The browser
uses normal FBWA RC control at 65% throttle until it clears 12 m, then selects the
requested mode (AUTO by default). Launch fails visibly if it cannot clear ground.
The browser's canopy deformation is illustrative; it is not a measured cloth shape.

The available replay channels contain payload attitude/rates, velocity, controls
and estimated wind, but no canopy pressure or observed inflation fraction. Thus
none of the inflation, launcher or contact constants are claimed to be fitted to
these logs. Airborne calibration and launch assumptions are deliberately separate.
This simulator is a development tool, not a validated launch or landing predictor.
