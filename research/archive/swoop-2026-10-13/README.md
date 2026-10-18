# Swoop landing study

Objective: demonstrate a 90-degree turning descent, rollout and flare in native SITL with soft and dramatic settings. Touchdown aspirations are 5 m miss, 1 m/s horizontal and 0.5 m/s downward speed. Report the best feasible result if these are incompatible with the present model.

Bounded sequence: transient authority screening; at most 18 native geometry/flare development runs; freeze two settings; eight validation runs covering calm, gentle gust, steady crosswind and an entry perturbation. Reassess the architecture once if the native scheduled flare cannot use the available physics. No indefinite tuning sweep.

Model: existing 9-DOF Trout Lake mission configuration and selected L1 10 s / damping 1 / measurement cutoff 1 Hz. No aerodynamic coefficients will be changed to create a swoop. Raw logs and full truth recordings remain in Mission Planner/analysis/swoop-2026-10-13. Only scripts, configurations, final figures, summaries and bounded viewer excerpts are archived here.

The simulator has no validated canopy collapse, stall or ground contact dynamics. A high-energy flare remains an extrapolation beyond the moderate-brake flight evidence.

Implementation and analysis assisted by Codex.

## Completed bounded study

[Interactive local report](report.html) · [Portable Markdown report](report.md) · [Final selection](selection.json)

32 open-loop physics screens and 18 native SITL runs were retained: eight development runs, two early dramatic runs excluded after auditing a heading-wrap trigger, and eight final robustness cases. All eight final cases finish within 5 m (worst 3.26 m), but the low-speed and low-swing aspirations remain unmet. This delivers the goal's best-demonstrated-maneuver fallback rather than claiming a settled precision swoop.

The final viewer build and all 12 existing viewer tests passed. Browser checks passed for both styles, the prior calm applet, the report embeds and mobile layout. The ArduPilot feature implementation and aerodynamic coefficients are unchanged.

To open locally after building `viewer` with Node 22+, serve the tinkering checkout with `python3 -m http.server 8076 --bind 127.0.0.1` and visit `http://127.0.0.1:8076/research/archive/swoop-2026-10-13/report.html`. The `viewer/dist` build directory is generated, not committed.
