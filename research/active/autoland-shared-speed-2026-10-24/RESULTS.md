# Shared minimum speed and native AUTOLAND

The browser now exposes native **AUTOLAND · home** in addition to **RTL · mission
landing**. This change is an experimental simulation feature, not physical-flight
qualification or a new claim of general spot-landing accuracy. Implementation,
testing and this report were AI-assisted.

## Shared parameter

`AHRS_MIN_SPEED` is an integer in metres per second, with firmware default 5 and
an effective range of 1–20. The browser sets it to 3. The parameter replaces the
shared GPS course-capture constant in Plane TAKEOFF, AUTO takeoff, CRUISE and
AUTOLAND, and the EKF2/EKF3 synthetic sideslip-fusion speed gates.

The measurement reference remains specific to each check: GPS course capture
uses **groundspeed**, while synthetic sideslip fusion uses **forward air-relative
speed**. Using groundspeed in the sideslip denominator check would change its
conditioning in wind, so only the threshold is shared. Wind-state initialization,
airspeed uncertainty, GPS quality checks, and unrelated speed limits are unchanged.
The EKF consumes the threshold through DAL. An appended byte in `RFRN` records it
without moving existing fields; old replay messages omit that byte and retain
the 5 m/s fallback. Lowering this threshold still requires adequate GPS course
quality and a valid zero-sideslip assumption.

## AUTOLAND behavior

Native mode 26 generates a loiter-to-altitude segment and a final approach to
AHRS home. It uses the captured takeoff course plus `AUTOLAND_DIR_OFF`, or the
existing compass-at-arming option; it does not choose a rally point or update its
approach direction from the current wind. The browser supplies
`AUTOLAND_WP_DIST=200` m and `AUTOLAND_WP_ALT=30` m, matching the nominal geometry
of the retained mission final leg. Thus the nominal slope is
$30/200=0.15$, or $\arctan(0.15)\approx8.53^\circ$.

The existing paraglider firmware extensions previously admitted only AUTO to
their brake flare, native phase telemetry and LQI approach eligibility. They now
use the native mode's `is_landing()` predicate and retain the LAND flight-stage,
arming and flare guards. AUTOLAND therefore uses the same brake flare and LQI
handover as a mission landing. `LAND_FLARE_AIM=0`, `LAND_FLARE_ALT=3` m and
`PG_FLARE_HGT=1.3` m remain unchanged.

The browser requests an acknowledged AUTOLAND mode change so entry rejection is
reported immediately. Landing status uses the home target independently of
uploaded mission items. While AUTOLAND is active, the map hides the inactive
mission and marks home; it does not synthesize the native generated path. Native
phase and estimator telemetry drive the existing pause after landing. Selecting
another mode leaves AUTOLAND and retains the uploaded mission.

## Verification

The rebuilt, packaged ArduPlane WASM runs the existing Rust physics, simulated
GPS/IMU/lidar and EKF. Control does not consume physics truth. Truth is used only
to measure contact. `integration.json`, `verification.json` and the release
manifest record runtime/configuration hashes and test provenance.

The native integration test:

1. Reads back the browser's shared threshold and AUTOLAND geometry parameters.
2. Confirms rejection while on the ground.
3. Sets the threshold to 20 m/s, launches and confirms rejection because no
   takeoff direction was captured.
4. Changes it to 3 m/s at runtime and captures direction below 5 m/s.
5. Enters AUTOLAND without uploading a mission, leaves it, and re-enters.
6. Confirms home targeting, LQI approach operation and final handover, symmetric
   brake flare, and an estimator-confirmed pause with mode remaining AUTOLAND.

The final recorded calm run captured direction at **4.045 m/s**, contacted
**0.470 m** from home with incoming sink **0.886 m/s**, and produced a maximum
flare brake demand of **71.995%**. This is one feature test with deliberately
changed launch gating; it is not a landing-accuracy distribution or proof of
wind robustness.

Both Playwright browser tests passed: switching into and out of AUTOLAND while
preserving the mission, then completing its landing; and repeating the default
circuit before commanding RTL and completing the original mission landing.
The browser tests use the normal default weather and launch configuration.
All **56 unit tests**, the TypeScript/Vite build, native SITL control integration,
and MIMO handover regression passed. The repository release-integrity checker
and complete firmware patch reverse-application check passed.

The earlier [landing calibration study](../landing-release-2026-10-24/RESULTS.md)
remains historical evidence for its recorded firmware and configuration. Its
headwind and steep-approach limitations remain relevant; this feature does not
claim to fix those limits. The rigid-canopy model still does not validate canopy
collapse or rollout.

## Replay

From `viewer`, run `npm run test:autoland`. Set `AUTOLAND_RESULT` to an absolute
JSON path to save the native result. Run
`npx playwright test tests/browser/autoland.spec.ts tests/browser/rtl.spec.ts`
after `npm run build` to exercise both UI paths. Use `stage.py` only when staging
a new release; it resets publication status and verifies native result hashes
against the selected firmware and configuration. Public deployment verification
is recorded separately in `publication.json`.
