"""Describe the frozen 35-deg/s qualification; retain failed original screens."""
import json
import numpy as np
from build import HERE
D=HERE/'deployment35'
q=json.loads((D/'qualification.json').read_text());d=json.loads((D/'disturbance.json').read_text())
rows=d['rows'];groups={v:[r for r in rows if r['variant']==v and r['complete']] for v in ['lqi25','lqi35','classic35']}
lines=['# 35 deg/s browser qualification','', 'AI-assisted. Same gain matrices, trim map, observer and two-second engagement blend as the deployed 25 deg/s controller. Only the allowed rate limit changes. Frozen protocols are retained beside this report.', '',
 f"Nominal/transition qualification: **{q['accepted']}**, {sum(r['passed'] for r in q['rows'])}/{len(q['rows'])} flights passed.",
 f"Disturbance qualification: **{d['accepted']}**, {sum(r['complete'] for r in rows)}/{len(rows)} flights completed.", '',
 '| Metric (median of six cases) | LQI 25 | LQI 35 | PG + TECS 35 |','|---|---:|---:|---:|']
for k in ['mean_bias','tracking_rms','rate_std','pitch_rms','pitch_peak','alt_rms','shaft_W','shaft_Wh_air_km','quiet_heading_rms','quiet_pitch_rms']:
    vals=[f'{np.median([r[k] for r in groups[v]]):.3f}' if groups[v] else '—' for v in groups]
    lines.append('| '+k+' | '+' | '.join(vals)+' |')
lines+=['','## Frozen disturbance checks','']+[f'- {k}: {v}' for k,v in d['checks'].items()]
lines+=['','## Scope','',
 '- New random cases vary damping/brake coefficients by +/-30%, actuator slew, wind and turbulence. Weather stops at 200 s; quiet recovery is scored in the final ten seconds.',
 '- Original absolute turbulent motion screens are retained in disturbance.json. Their failures are not silently discarded; this release decision uses the explicitly frozen relative-performance and quiet-recovery criteria.',
 '- The 25 versus 35 comparison changes task demand, so increased power is not necessarily reduced efficiency. Shaft power is modeled rather than battery-measured.',
 '- The browser shared turn-reference limit PG_TURN_RMAX must also rise to 35 to make the LQI cap reachable. PG + TECS therefore receives a matched 35 deg/s qualification and remains the default.',
 '- No physical flight-safety certification or 40 deg/s release is implied. These are finite simulation checks, not a proof of nonlinear stability.']
(D/'RESULTS.md').write_text('\n'.join(lines)+'\n');print('\n'.join(lines))
