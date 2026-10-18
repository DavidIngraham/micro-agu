import json
from pathlib import Path
import numpy as np
from score import analyze_phase
from evaluate import compare,m
HERE=Path(__file__).resolve().parent
for phase in ['training','validation']:
    if not (HERE/phase).exists():continue
    metrics=HERE/phase/'metrics.json'
    raw=[p for p in (HERE/phase).glob('*/*.json') if p.stem.isdigit()]
    rows=json.loads(metrics.read_text()) if metrics.exists() and all(p.stat().st_mtime_ns<metrics.stat().st_mtime_ns for p in raw) else analyze_phase(HERE/phase)
    for name in sorted({r['variant'] for r in rows}):
        rr=[r for r in rows if r['variant']==name];good=[r for r in rr if r['score'] is not None]
        print(phase,name,len(good),'/',len(rr),'score',round(float(np.median([r['score'] for r in good])),4) if good else None,
              'gates',[(r['case'],r.get('gates')) for r in rr if r['score'] is None],flush=True)
        if phase=='training' and len(rr)==6 and name!='published':
            comparison=compare(rows,name,'published',6)
            print('  paired', {k:round(v['median'],2) for k,v in comparison['changes'].items()},'rejections',comparison['reasons'],flush=True)
for directory in sorted((HERE/'missions').glob('*')):
    if not directory.is_dir():continue
    rr=[m.mission_metrics(p) for p in directory.glob('*.json') if p.stem.isdigit()]
    print('missions',directory.name,sum(r['complete'] and not r.get('screen_failures') for r in rr),'/',len(rr),
          'failures',[(r['case'],r.get('screen_failures'),r['failure']) for r in rr if not r['complete'] or r.get('screen_failures')])
