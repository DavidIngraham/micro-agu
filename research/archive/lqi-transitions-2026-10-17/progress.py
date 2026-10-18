import json
from pathlib import Path
import numpy as np
import sys
here=Path(__file__).resolve().parent
if '--refresh' in sys.argv:
    from score import analyze_phase
    for phase in ['validation','ablation']:
        if (here/phase).exists():analyze_phase(here/phase)
for phase in ['training','validation','ablation']:
    p=here/phase/'metrics.json'
    if not p.exists():continue
    rows=json.loads(p.read_text())
    for name in sorted({r['variant'] for r in rows}):
        rr=[r for r in rows if r['variant']==name]
        good=[r for r in rr if r['score'] is not None]
        print(phase,name,'gates',len(good),'/',len(rr),
          'score',round(float(np.median([r['score'] for r in good])),3) if good else None,
          'peaks',np.median([r['command_peak_rates_deg_s'] for r in good],axis=0).round(1).tolist() if good else None,
          'rise',round(float(np.median([r['steps']['climb']['rise_10_90_s'] for r in good])),2) if good else None,
          'updraft',round(float(np.median([r['gusts']['vertical']['recovery_after_off_s'] for r in good])),2) if good else None)
