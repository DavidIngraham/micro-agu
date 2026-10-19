import concurrent.futures as cf,json
from run import P,run
cases=[]
winds={'calm':(0,0),'head':(-1.5,0),'tail':(1.5,0),'cross':(0,1.5)}
for name,(n,e) in winds.items():
 for strategy in ['gate','zero','weighted']:
  for seed in [17,73]:
   cases.append(dict(id=f'natural-{name}-{strategy}-{seed}',strategy=strategy,weather='default',north=n,east=e,seed=seed,group='natural'))
  cases.append(dict(id=f'delayed-{name}-{strategy}',strategy=strategy,weather='default',north=n,east=e,seed=73,windDelay=12,group='delayed'))
for name,n,e in [('correct',-1.5,0),('reversed',1.5,0),('rotated',0,3),('too-strong',-4,0)]:
 cases.append(dict(id=f'prior-{name}',strategy='weighted',weather='default',north=-1.5,east=0,seed=73,priorNorth=n,priorEast=e,group='prior'))
for strategy in ['gate','zero','weighted']:
 cases.append(dict(id=f'timeout-{strategy}',strategy=strategy,weather='default',north=0,east=0,seed=73,windDelay=60,group='timeout'))
(P/'protocol.json').write_text(json.dumps(dict(cases=cases,criteria={'completion':True,'no_ground_recontact':True,'no_premature_engagement':True,'max_true_roll_deg':45,'max_true_pitch_deg':45,'note':'Attitude bounds apply after free-flight release; descriptive screening, not certification.'}),indent=2))
with cf.ThreadPoolExecutor(max_workers=4) as pool:rows=list(pool.map(run,cases))
(P/'execution.json').write_text(json.dumps(rows,indent=2))
