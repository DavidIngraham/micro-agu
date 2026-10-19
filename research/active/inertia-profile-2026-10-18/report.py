import json
from pathlib import Path
import numpy as np
OUT=Path(__file__).resolve().parent;PREV=OUT.parent/'power-transition-fit-2026-10-18'
rows=json.loads((OUT/'profile.json').read_text());transfer=json.loads((OUT/'transfer.json').read_text());chosen=json.loads((OUT/'selection.json').read_text());baseline={r['name']:r for r in json.loads((PREV/'results.json').read_text()) if r['variant']=='baseline'}
s=[]
for r in rows:
 t=[x for x in transfer if x['payload']==r['payload'] and x['rotor']==r['rotor']]
 train=next(x for x in t if x['name']=='trout_lake-3350');val=next(x for x in t if x['name']=='trout_lake-3443')
 gate=all(train['metrics'][phase]['rmse'][axis]<=baseline[train['name']]['metrics'][phase]['rmse'][axis] for phase in ['powered','off'] for axis in [1,2])
 s.append(dict(payload=r['payload'],rotor=r['rotor'],cost=r['selected']['cost'],parameters=np.exp(r['selected']['x'][:4]).tolist(),training_pitch_yaw_nonregression=gate,training_rmse=train['metrics'],validation_rmse=val['metrics'],start_cost_spread=abs(r['fits'][0]['cost']-r['fits'][1]['cost'])))
summary=dict(selected_payload=chosen['payload'],selected_rotor=chosen['rotor'],cells=s,total_optimizer_calls=sum(f['calls'] for r in rows for f in r['fits']),optimizer_successes=sum(f['success'] for r in rows for f in r['fits']),training_nonregression_count=sum(x['training_pitch_yaw_nonregression'] for x in s))
(OUT/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps({k:v for k,v in summary.items() if k!='cells'},indent=2))
for x in s:print(x['payload'],x['rotor'],round(x['cost'],3),'train yaw',*[round(x['training_rmse'][k]['rmse'][2],2) for k in ['powered','off']],'val yaw',*[round(x['validation_rmse'][k]['rmse'][2],2) for k in ['powered','off']])
