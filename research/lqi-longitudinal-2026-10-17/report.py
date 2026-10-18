"""Summaries preserve every attempted case, with matched-task paired comparisons."""
import json,hashlib,subprocess,gzip
from pathlib import Path
import numpy as np
from campaign import assess
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
rows=[]
for path in sorted((HERE/'runs').glob('*.json')):
 if path.name.endswith(('-spec.json','-metrics.json')):continue
 rows.append(assess(path))
pairs=[]
for i in range(6):
 a=next((x for x in rows if x['spec']['id']==f'fixed-heldout-{i}-lqi'),None)
 b=next((x for x in rows if x['spec']['id']==f'heldout-{i}-baseline'),None)
 if not a or not b:continue
 good=a['complete'] and b['complete']
 checks={} if not good else {k:a[k]<=1.2*b[k]+2 for k in ['pitch_rms','pitch_peak']}
 pairs.append(dict(case=i,complete=good,checks=checks,passed=good and all(checks.values()),ratios={} if not good else {k:a[k]/b[k] for k in ['pitch_rms','pitch_peak','energy_Wh']},lqi_reasons=a['reasons'],baseline_reasons=b['reasons']))
for i in range(2):
 a=next((x for x in rows if x['spec']['id']==f'fixed-fresh-{i}-lqi'),None)
 b=next((x for x in rows if x['spec']['id']==f'fixed-fresh-{i}-baseline'),None)
 if not a or not b:continue
 good=a['complete'] and b['complete'];checks={} if not good else {k:a[k]<=1.2*b[k]+2 for k in ['pitch_rms','pitch_peak']}
 pairs.append(dict(case='fresh-'+str(i),complete=good,checks=checks,passed=good and all(checks.values()),ratios={} if not good else {k:a[k]/b[k] for k in ['pitch_rms','pitch_peak','energy_Wh']},lqi_reasons=a['reasons'],baseline_reasons=b['reasons']))
fixed=[x for x in rows if x['spec']['id'].startswith('fixed-')]
candidate=[x for x in fixed if x['spec']['scheme']=='lqi']
summary=dict(candidate_cases=len(candidate),all_candidate_pass=len(candidate)==14 and all(x['passed'] for x in candidate),fixed_cases=len(fixed),all_fixed_pass=len(fixed)==18 and all(x['passed'] for x in fixed),rows=rows,pairs=pairs,all_screen_pass=all(x['passed'] for x in rows if x['spec']['id'].startswith('nominal')),all_heldout_lqi_pass=all(x['passed'] for x in rows if x['spec']['id'].startswith('fixed-heldout') and x['spec']['scheme']=='lqi'),all_pairs_pass=len(pairs)==8 and all(x['passed'] for x in pairs))
summary['qualified']=summary['all_candidate_pass'] and summary['all_pairs_pass'] and all(x['passed'] for x in rows if x['spec']['id'].startswith('fix-check-'))
(HERE/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
lines=['# Longitudinal envelope campaign','', 'Packaged ArduPlane firmware, EKF airspeed and onboard canopy observer; as-built 132 mm geometry. AI-assisted simulation research.','',
'Candidate includes the normalized-throttle feedback fix from ArduPilot commit 20a8fb59c8. CP3_HUP=0.75 m/s, CP3_HDN=1 m/s; HRTC=0.7 s. Feedback/observer gains, speed target range, 35 deg/s turn limit and 60% nominal throttle ceiling unchanged.','',
'## Results','', f"Candidate: {sum(x['passed'] for x in candidate)}/{len(candidate)} passes; all fixed-binary flights including classical comparators: {sum(x['passed'] for x in fixed)}/{len(fixed)} passes. Qualified: {summary['qualified']}. Comparator failures are retained and do not excuse any candidate failure.",'', '| Case | Pass | Up mean m/s | Down mean m/s | Relative pitch RMS / peak deg/s | Quiet altitude RMS m | Shaft W |','|---|---|---:|---:|---:|---:|---:|']
for r in rows:
 if not r['complete']:lines.append(f"| {r['spec']['id']} | FAIL: {r['failure']} | | | | | |");continue
 lines.append(f"| {r['spec']['id']} | {r['passed']} | {r['phases']['climb']['vertical_mean']:.3f} | {r['phases']['descend']['vertical_mean']:.3f} | {r['pitch_rms']:.2f} / {r['pitch_peak']:.2f} | {r['quiet']['height_rms']:.3f} | {r['shaft_W']:.2f} |")
for r in rows:
 if r['reasons']:lines.extend(['',r['spec']['id']+': '+', '.join(r['reasons'])])
lines.extend(['','## Matched disturbance comparisons',''])
for key in ['pitch_rms','pitch_peak','energy_Wh']:
 values=[x['ratios'][key] for x in pairs if x['complete']]
 if values:lines.append(f"- LQI / PG+TECS {key}: median {np.median(values):.3f}, p90 {np.percentile(values,90):.3f}, worst {max(values):.3f}, n={len(values)}.")
lines.extend(['','## Voltage feedback defect', '',
'The original firmware failed to settle at 20.4 V even with the old +/-0.5 m/s limits. Recorded initialization shows sustained full throttle far above the 80 m target. The custom allocator compared nominal throttle requests with voltage-compensated feedback, introducing a false anti-windup residual. The fix inverses the active voltage scale for allocation/observer/TECS feedback while retaining real limiting and suppression. Motor output itself is unchanged.',
'Targeted tests cover the old limits at 20.4 V, compensation disabled, and an in-flight voltage change. Original failures and diagnostic initialization traces remain included above. The six matched PG+TECS reference flights use the original binary; that controller does not exercise the changed custom-control feedback. Two additional fresh paired cases use the fixed binary on both sides.', '',
'## Interpretation','',
'The 1 m/s climb passed nominal tests but consumed almost the entire throttle ceiling. The selected lower climb bound reserves more power for disturbances. Descent authority has a different actuator margin.',
'Trim solves are a constrained numerical screen, not proof of feasibility or infeasibility. Initial and flight-seeded grids are retained; actual trajectory speed can differ from the controller target range. No stall/collapse/structural boundary is established.',
'Stability is assessed separately from tracking and efficiency: bounded attitude/airspeed, pitch response, and weather-off quiet recovery. Finite tests do not prove nonlinear stability. Power is modeled shaft power, not measured battery consumption.',
'The thermal is a fixed spatial Gaussian; Dryden is seeded. Model coefficients and actuator slew vary in held-out cases. Identical seeds do not force identical wind histories when controllers follow different trajectories.',
'Raw outputs preserve failures. Existing unrelated controller-design working files were not used as the deployed gain source. The trim Chart and constants are identical to committed design_mimo.py.','', 'The paired comparison gate applies to the candidate; classical-controller recovery failures remain failures in this report. Every LQI case must independently satisfy the absolute gates.','',
'## Reproduce','', 'See REPRODUCE.md for frozen inputs, runtimes, raw traces and commands. Run from the repository: simulation/.venv/bin/python research/lqi-longitudinal-2026-10-17/trim.py. Run campaign.py with the analysis environment and screen, heldout, final, fix-check or fixed; it invokes the real packaged WASM from viewer. protocol.json and the frozen *-specs.json files define the tests.'])
(HERE/'README.md').write_text('\n'.join(lines)+'\n')
print(json.dumps({k:v for k,v in summary.items() if k!='rows'},indent=2))
