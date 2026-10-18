import json
from pathlib import Path
HERE=Path(__file__).resolve().parent
rows=json.loads((HERE/'training/metrics.json').read_text())
for r in rows:
    if r['variant'] in ['published','responsive_brake3k'] and r['complete']:
        print(r['variant'],r['case'],[(k,round(v['mean_speed_m_s'],3)) for k,v in r['steps'].items()])
config=json.loads((HERE.parents[1]/'viewer/assets/runtime/highdof.json').read_text())
print('\n'.join(x for x in config['defaults'].splitlines() if 'SPD' in x or 'AIRSPEED' in x))
