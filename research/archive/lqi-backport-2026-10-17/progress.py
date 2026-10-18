from pathlib import Path
import numpy as np
from score import analyze_phase
here=Path(__file__).resolve().parent
for phase in ['training','validation']:
    if not (here/phase).exists():continue
    rows=analyze_phase(here/phase)
    for name in sorted({r['variant'] for r in rows}):
        rr=[r for r in rows if r['variant']==name];good=[r for r in rr if r['score'] is not None]
        print(phase,name,'qualified',len(good),'/',len(rr),'reasons',[r['gates'] for r in rr if r['gates']],
          'score',float(np.median([r['score'] for r in good])) if good else None,
          'peaks',np.median([r['command_peak_rates_deg_s'] for r in good],axis=0).tolist() if good else None)
