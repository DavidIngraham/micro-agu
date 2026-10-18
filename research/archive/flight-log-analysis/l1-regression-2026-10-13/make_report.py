import os

from pathlib import Path
import json
r=Path(os.environ.get('PG_RESEARCH_ROOT', '/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/l1-regression-2026-10-13'))
m=json.loads((r/'mission-results.json').read_text())
lines=['# L1 mission and landing regression','','Selected: NAVL1_PERIOD=10 s, compared with 8 s. The 12 s intermediate candidate is retained in the tables but was not selected. All selected inner lateral and longitudinal gains are unchanged. NAVL1_DAMPING remains 1.0.','','## Mission profiles','','The extended native AUTO route includes altitude changes from 45 to 90 m, successive waypoint corners, right and left two-turn loiters, home loiter, and a brief handover to/from legacy control. Normal radius is 25 m; added tests use 20 and 45 m. Waypoint acceptance remains fixed at the existing angle-scaled 22 m setting. Native pass criteria are unmodified.','','| Profile | Period (s) | Native pass | Max corner overrun (m) | Max waypoint height error (m) | Right / left radial RMS (m) |','|---|---:|---|---:|---:|---:|']
lines[4:4]=['## Selection','','Use a 10-second L1 period with damping 1.0 in the complete opt-in development profile. All existing inner-loop gains remain unchanged. Forty-two new native runs were completed during this bounded study. The selected candidate passed all five mission profiles and all twelve landing regression cases. Median miss on that twelve-case set is 1.70 m, worst 2.51 m; separate focused repeats reached 3.28 m.','','This is a practical compromise: it fixes the calm large-amplitude oscillation while preserving ordinary mission performance. It is not a claim that every metric improves, and it does not establish stability for every possible initialization.','']
for name,z in m.items():
 radial=' / '.join(f"{x['radial_rmse_m']:.2f}" for x in z['loiters'].values())
 lines.append(f"| {name} | {z['parameters']['NAVL1_PERIOD']} | {z['passed']} | {z['max_corner_overrun_m']:.2f} | {z['max_waypoint_height_error_m']:.2f} | {radial} |")
lines+=['','| Profile | Worst settled-leg cross-track RMS (m) | Maximum settled-segment command saturation (%) |','|---|---:|---:|']
for name,z in m.items():
 xt=max(x['xtrack_rmse_m'] for x in z['legs'].values())
 sat=max(x['saturation_pct'] for x in [*z['legs'].values(),*z['loiters'].values()])
 lines.append(f'| {name} | {xt:.3f} | {sat:.2f} |')
lines+=['','![Mission comparison](mission-comparison.png)','','## Landing regression','','The twelve conditions were previously independent validation cases; after reuse here they are regression conditions, not fresh held-out evidence. Pre-flare metrics cover the final 20 seconds before the first active PGFL sample. Truth heading rate is reconstructed from simulator attitude and angular velocity; feedback filtering cannot manufacture an apparent improvement. Roll span means the central 90% span, not peak-to-peak.']
if (r/'landing-results.json').exists():
 d=json.loads((r/'landing-results.json').read_text())
 lines+=['','| Case | Roll span 8 → 10 s (deg) | Truth rate RMSE 8 → 10 s (deg/s) | Miss 8 → 10 s (m) |','|---|---:|---:|---:|']
 for i in range(12):
  a=d.get(f'validation-{i:02d}');b=d.get(f'landing10-{i:02d}')
  if a is None or b is None:continue
  lines.append(f"| {i} | {a['roll_90pct_span']:.2f} → {b['roll_90pct_span']:.2f} | {a['rate_error_rms']:.2f} → {b['rate_error_rms']:.2f} | {a['miss_m']:.2f} → {b['miss_m']:.2f} |")
 lines+=['','![Landing comparison](landing-comparison.png)']
lines+=['','## Why 12 seconds was not selected','','The 12 s candidate passed all native mission and touchdown assertions, but landing condition 08 developed a severe approximately 0.2 Hz oscillation: roll span 32.94 deg, truth heading-rate error 47.92 deg/s, and commands reaching their limits. An immediate repeat settled, showing sensitivity rather than eliminating the original failure. Both runs remain in the evidence. Sixteen and twenty seconds settled that case but missed by about 3.8 m. Ten seconds settled calm air and two focused repeats of condition 08, then settled that case again in the complete landing set. Those three 10 s runs missed by 3.28, 2.67 and 2.51 m respectively. This is bounded evidence, not a proof covering every initialization.','','![Rejected 12 s landing comparison](landing-12s-comparison.png)','','The worst 10 s roll span in the twelve-case set is 2.95 deg in case 07 (versus 1.18 deg at 8 s). Its trace shows a larger early excursion that settles before flare, not the repeated large reversals seen in the rejected case. That tracking cost is retained rather than hidden by the touchdown statistics.','','![Case 07 transient comparison](case07-response.png)']
lines+=['','## Reproduction and limits','','Raw files remain in Mission Planner/analysis/l1-regression-2026-10-13. run.py, investigate08.py, test10.py, run10_landings.py and run10_missions.py run the native tests, analyze_missions.py and analyze_landings.py audit logged parameters and compute truth metrics; plot.py and make_report.py generate these results. The three ordinary 8 s mission baselines and twelve 8 s landing baselines are reused audited recordings. Radius variations have new matched 8/12 s runs. Wind is physically applied in the simulation, not visual motion. Gusts are repeatable synthetic fields, not a calibrated turbulence spectrum.','','Implementation and analysis assisted by Codex.']
(r/'report.md').write_text(chr(10).join(lines)+chr(10))
