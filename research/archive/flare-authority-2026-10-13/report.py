from study import OUT,RAW,MODEL,SOURCE
from markdown_it import MarkdownIt
import json,hashlib,subprocess
native=json.loads((OUT/'native-results.json').read_text())
s="""# Stronger flare investigation — 13 October 2026

## Result

A faster, slightly later flare gives a modest reduction in forward touchdown speed in the model. More maximum pull alone contributes almost nothing beyond the existing 95% setting. Earlier flare can remove substantially more forward speed, but the aircraft then passes its minimum-sink point and hits the ground descending faster. None of these tests establishes a near-zero-speed, near-zero-sink precision landing.

The retained model and flight defaults are unchanged. The 150 percentage-point/s candidate is a physics-screen result; native firmware currently clamps PG_FLARE_RATE to 100, so it has **not** been tested as a native controller setting. Its small benefit is worth a subsequent isolated controller experiment, not automatic promotion.

## What was run

- 144 initial physics cases: two onset states, six heights (0.5, 0.8, 1.1, 1.5, 2, 3 m), four maximum pulls (60, 80, 95, 100%) and three rates (30, 100, 300 percentage points/s).
- 56 refinement cases: seven additional heights and rates of 100, 150, 200 and 300 percentage points/s, retaining 95% maximum pull.
- 64 fixed-candidate sensitivity cases: current, full travel, earlier and faster schedules, both entries, weaker brake lift/more drag, three smooth-gust phases and ±0.2 m trigger-height error.
- Eight time-resolution checks at 10 and 5 ms, following the 20 ms screen.
- Eight native AUTO spiral-to-landing cases, including a baseline repeat, an earlier-trigger repeat and two moderately gusty missions.

The physics screen uses the 1.758 kg Trout Lake mission profile, 62.40/71.99 mm left/right travel limits, idle throttle and the existing 221.52 mm/s actuator bound. Common brake is a normalized percentage of each side's configured travel, not equal millimeters. Initial trim is retained as differential command; no lateral autopilot operates in this screen. Full common pull removes the remaining steering headroom.

The normal onset state comes from settled idle glide (4.55 m/s airspeed, 1.08° payload bank). The release state is taken directly from the previous full-right brake release at its speed peak (5.42 m/s, 16.77° payload bank). It is still rolling out and sinking at 1.96 m/s: it is not simply a faster straight glide. Each state is translated to the chosen flare-onset altitude and rotated so initial ground velocity points north. This tests available flare response conditional on arriving in that state; it does not show that a guidance controller can deliver it at the correct position and altitude.

## Speed versus sink

Representative nominal results from the 20 ms screen:

| Entry | Schedule | Horizontal touchdown speed | Touchdown sink |
|---|---|---:|---:|
| Glide | Current: 95%, 100 pp/s, 1.1 m | 3.05 m/s | 0.86 m/s |
| Glide | Faster: 95%, 150 pp/s, 1.0 m | 2.78 m/s | 0.85 m/s |
| Glide | Earlier: 95%, 100 pp/s, 1.5 m | 2.29 m/s | 1.16 m/s |
| Release | Current: 95%, 100 pp/s, 1.1 m | 2.53 m/s | 0.76 m/s |
| Release | Faster: 95%, 150 pp/s, 1.0 m | 2.43 m/s | 0.75 m/s |
| Release | Earlier: 95%, 100 pp/s, 1.5 m | 1.98 m/s | 1.29 m/s |

At 5 ms, current versus faster is 3.006/0.871 versus 2.739/0.857 m/s for glide, and 2.504/0.780 versus 2.397/0.768 m/s for release (horizontal/sink). The forward-speed improvement persists with finer time resolution. The sink improvement is only about 0.01 m/s, so describe it as essentially unchanged rather than a meaningful vertical-performance gain.

![Flare tradeoff](tradeoff.png)

![Time histories](flare-traces.png)

More maximum travel is not the missing ingredient. With the 1.1 m/100 pp/s schedule, 95% and 100% are nearly identical because touchdown occurs around the time the ramp reaches its ceiling. Retained differential trim can also saturate one side before the common floor reaches 100%. The prior native spiral landing was already using 95% and the software's maximum application rate.

The more aggressive 300 pp/s, 100% case at 1.1 m reaches about 2.18 m/s horizontal with 1.11 m/s sink from glide, or 1.98/1.10 m/s from release. That is stronger deceleration, but not an unambiguous landing improvement. No tested case rose above its trigger altitude before contact; the visible pitch-up and reduced sink should not be mistaken for a sustained balloon or level skim.

## Robustness

The faster 150 pp/s schedule generally retains its modest horizontal-speed advantage when brake lift is scaled to 0.75 or 0.5 and explicit brake drag to 1.25 or 1.5, respectively. These are uncertainty probes, not measured aerodynamic bounds. They do not introduce realistic stall or canopy collapse.

The three 0.5 m/s horizontal smooth-gust components have a 12 s base period and a 0.125 m/s vertical amplitude. In the short physics screen, the specified wind begins at flare onset while the initial state is held fixed; this is an entry/wind sensitivity test, not a full turbulent approach. Native gust runs apply the existing smooth wind model throughout the flight.

A ±0.2 m onset error is enough to alter the tradeoff. In the release case with a late trigger, the faster schedule actually has slightly more horizontal speed than current at contact. With an early trigger, its sink is slightly worse. There is no universally dominating fixed schedule.

![Sensitivity results](robustness.png)

## Native AUTO and spot accuracy

These runs use the unchanged native AUTO mission, L1/turn controller, TECS and AP_Landing. Only flare height or maximum pull/reserve varies. The faster-than-100 pp/s schedules are excluded because the firmware would clamp them.

| Case | Configured / actual flare height | Horizontal speed | Sink | Target miss |
|---|---:|---:|---:|---:|
"""
for r in native:s+=f"| {r['name']} | {r['configured_flare_height']:.2f} / {r['actual_flare_height']:.2f} m | {r['horizontal']:.2f} m/s | {r['sink']:.2f} m/s | {r['miss']:.2f} m |\n"
s+="""
![Native mission outcomes](native-results.png)

The first fresh baseline crossed the target before the configured height trigger. AP_Landing can enter flare after waypoint passage without an in-range rangefinder; that native fallback produced flare onset near 1.93 m despite LAND_FLARE_ALT=1.1 m. PGFL records the active flare, the LAND message reports about 1.9 m, and truth position is already 0.34 m beyond the target when the first active PGFL sample appears. The subsequent baseline repeat triggers near 1.05 m and has a very different touchdown. These are retained outcomes, not discarded outliers.

Small differences in mission-entry state and timing exist between native runs, and the fallback makes an unpaired comparison particularly misleading. This study identifies the actual trigger difference; it does not isolate every upstream source of repeat variation. It therefore does not claim statistical improvement in spot accuracy from these eight runs.

Earlier 1.3 m flare reduces horizontal speed relative to the normally triggered baseline, but increases sink in both calm and gusty comparisons. The 1.5 m run increases that tradeoff. Removing the last 5% steering reserve for full common travel gives no demonstrated advantage worth adopting.

## Recommendation

Keep the existing maximum pull and steering reserve. A **150 pp/s rate with approximately 1.0 m onset** is a reasonable next isolated native-controller candidate, with the same abort/mode-reset logic retained, but the expected gain is modest. Test it against controlled approach states and actual flare-trigger reasons before considering a default change.

For a dramatic swoop followed by a soft spot landing, the larger problem remains coordinated entry energy, rollout and flare timing. The current model does not demonstrate that simply pulling harder delivers that combination. The native waypoint-passage fallback also needs to be included in approach robustness work; changing flare gains alone cannot fix arriving over the target too high.

All touchdown metrics stop at the first payload-CG ground-plane crossing. They do not model physical skid/contact geometry, impact, a runout, collapse, stall progression or a person landing. Deep-brake aerodynamic response remains unvalidated by flight data. These tests diagnose the simulator and controller tradeoffs, not flight-ready flare settings.

## Files and reproduction

Run `study.py`, then `study.py refine.json`, `study.py robust.json`, `study.py resolution.json`, and `study.py fine.json`. `plots.py` renders the physics plots. `native_batch.py` runs named configs via the existing native harness; `native_analysis.py` extracts contact and first active PGFL metrics; `report.py` assembles this report and provenance.

The scripts, plots and result summaries are retained here. Full trajectories, BIN logs and viewer frames remain in Mission Planner under `analysis/flare-authority-2026-10-13`. The prerequisite onset trajectories are from the preceding spiral-authority study. `provenance.json` records simulator revision and source hashes. Analysis and scripts were prepared with AI assistance.
"""
(OUT/'README.md').write_text(s)
(OUT/'report.html').write_text('<!doctype html><meta charset="utf-8"><title>Stronger flare investigation</title><style>body{max-width:1100px;margin:40px auto;padding:0 24px;font:17px/1.6 system-ui;color:#192638}img{max-width:100%}table{border-collapse:collapse}th,td{border:1px solid #ccd;padding:7px}h2{margin-top:2em}</style>'+MarkdownIt().enable('table').render(s))
files=[MODEL/x for x in ['dynamics.cpp','libdynamics.so','parameters.json','simulator.py','profiles.py','actuator.py','mission-controller-candidate.json','landing_autotest.py']]+[SOURCE/x for x in ['mission-baseline-0-1.5-1.5.npz','mission-right-1-1.5-0.15.npz']]
prov=dict(ardupilot_commit=subprocess.check_output(['git','-C',str(MODEL),'rev-parse','HEAD'],text=True).strip(),files={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},raw_directory=str(RAW),physics_cases=272,native_cases=len(native))
(OUT/'provenance.json').write_text(json.dumps(prov,indent=2))
print('Wrote report and provenance')
