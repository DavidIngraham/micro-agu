"""Summarize frozen release gate without changing its thresholds."""
import json
from pathlib import Path
HERE=Path(__file__).resolve().parent
d=json.loads((HERE/'release-validation.json').read_text())
lines=['# Expanded-envelope release validation','',
       'AI-assisted. Six unused paired physics/weather cases; original K/L and EKF sensing. Wind and turbulence removed at 200 seconds. Final recovery scored over seconds 240–250. Original thresholds retained.', '',
       '| Variant | Complete | All gates passed | Quiet recovery passed | Worst quiet heading RMS | Worst quiet pitch-rate RMS | Worst turn pitch-rate peak |',
       '|---|---:|---:|---:|---:|---:|---:|']
for name,rows in d['results'].items():
    complete=[r for r in rows if r['complete']]
    recovery=[r['phases']['recover2'] for r in complete]
    quiet=sum(s['rate_rms']<=3 and s['alt_rms']<=1.5 and s['relative_pitch_rate_rms']<=3 for s in recovery)
    lines.append(f"| {name} | {len(complete)}/{len(rows)} | {sum(r['passed'] for r in rows)}/{len(rows)} | {quiet}/{len(rows)} | {max(s['rate_rms'] for s in recovery):.2f} deg/s | {max(s['relative_pitch_rate_rms'] for s in recovery):.2f} deg/s | {max(s['relative_pitch_rate_peak'] for r in complete for k,s in r['phases'].items() if k in ['turn1','reverse','turn2']):.2f} deg/s |")
lines+=['',f"Disturbance release gate accepted: **{d['accepted']}**.",'',
        'Failed disturbance gates block publication. Controller-transition qualification and production integration are deferred because the prerequisite failed. No remote push or browser deployment was performed. Clean recovery supports distinguishing forced motion from persistent oscillation, but does not waive excessive motion during gusts or establish stability margins.','',
        '## Individual failures','']
for name,rows in d['results'].items():
    for r in rows:lines.append(f"- {name}, case {r['id']}: "+('; '.join(r['reasons']) or 'pass'))
(HERE/'RELEASE-VALIDATION.md').write_text('\n'.join(lines)+'\n')
print('\n'.join(lines[:15]))
