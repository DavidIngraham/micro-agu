"""Select on four predeclared training cases; never inspect held-out scores."""
import json
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
TRAIN = {0, 1, 6, 9}
baseline = {r['id']: r for r in json.loads((HERE/'fixed/analysis.json').read_text())
            if r['variant'] == 'sensor' and r['id'] in TRAIN}
assert len(baseline) == 4 and all(r['complete'] for r in baseline.values())
result = {}
for name in ['efficient', 'damping']:
    rows = [r for r in json.loads((HERE/name/'analysis.json').read_text()) if r['id'] in TRAIN]
    assert len(rows) == 4
    reasons = []; ratios = []
    for r in rows:
        b = baseline[r['id']]
        if not r['complete']:
            reasons.append(f'case {r["id"]}: incomplete'); continue
        if r['late']['pitch_ac_deg'] > max(.2, 1.25*b['late']['pitch_ac_deg']):
            reasons.append(f'case {r["id"]}: persistent recovery motion')
        if r['upper_brake_fraction'] > .05:
            reasons.append(f'case {r["id"]}: upper brake saturation')
        if r['disturbed']['altitude_rms_m'] > 1.2*b['disturbed']['altitude_rms_m']:
            reasons.append(f'case {r["id"]}: altitude regression >20%')
        if r['mode'] == 'AUTO' and r['disturbed']['xtrack_rms_m'] > max(1., 1.2*b['disturbed']['xtrack_rms_m']):
            reasons.append(f'case {r["id"]}: cross-track regression >20%')
        ratios.extend([r['shaft_W']/b['shaft_W'], r['disturbed']['pitch_ac_deg']/b['disturbed']['pitch_ac_deg'],
                       r['disturbed']['altitude_rms_m']/b['disturbed']['altitude_rms_m']])
    score = float(np.exp(np.mean(np.log(ratios)))) if ratios else None
    result[name] = dict(score=score, rejected_reasons=reasons)
eligible = [name for name, r in result.items() if not r['rejected_reasons'] and r['score'] < 1]
selected = min(eligible, key=lambda name: result[name]['score']) if eligible else 'fixed'
out = dict(training_cases=sorted(TRAIN), candidates=result, selected=selected,
           rule='Require completion, recovery and saturation checks; no >20% altitude/AUTO cross-track regression. Rank geometric mean of paired shaft power, disturbed pitch RMS and altitude RMS ratios; require improvement over fixed gains.')
(HERE/'selection.json').write_text(json.dumps(out, indent=2)+'\n')
(HERE/'selected.txt').write_text(selected+'\n')
print(json.dumps(out, indent=2))
