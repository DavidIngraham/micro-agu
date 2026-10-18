"""Keep dimensional gust outcomes next to normalized task-loss comparisons."""
import json
from pathlib import Path
HERE=Path(__file__).resolve().parent
lines=['# Gust response detail','','Each row is a matched scenario; rates and angles are truth used only for scoring.','',
       '| Set | Cohort | Case | Filter | Crosswind pitch peak ° | Recovery s | Altitude IAE m·s | Crosswind loss |',
       '|---|---|---:|---|---:|---:|---:|---:|']
for phase in ['training','validation']:
    rows=json.loads((HERE/phase/'metrics.json').read_text())
    for cohort in ['local','deployed']:
        for r in sorted([r for r in rows if r['variant'].startswith(cohort)],key=lambda r:(r['case'],r['variant'])):
            if not r['complete']:continue
            w=r['gusts']['crosswind'];rec=w['recovery_after_off_s']
            vals=[w['peak_pitch_excursion_deg'],rec,w['altitude_iae_m_s'],r['tasks']['crosswind']]
            lines.append(f'| {phase} | {cohort} | {r["case"]} | {r["variant"].split("_")[1]} | '+' | '.join('n/a' if v is None else f'{v:.3f}' for v in vals)+' |')
(HERE/'GUSTS.md').write_text('\n'.join(lines)+'\n')
print('\n'.join(lines))
