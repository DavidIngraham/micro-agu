import json,concurrent.futures as cf
from run import P,run
primary=json.loads((P/'protocol.json').read_text())['cases']
cases=[]
for s in primary:
 if s['strategy']=='weighted':
  s=s.copy();s['id']='smooth-'+s['id'];s['revision']='smooth';cases.append(s)
for strategy in ['gate','zero','weighted']:
 cases.append(dict(id=f'late-valid-{strategy}',strategy=strategy,weather='default',north=0,east=0,seed=73,windDelay=44,group='late-valid',revision='smooth'))
(P/'smooth-protocol.json').write_text(json.dumps(cases,indent=2))
with cf.ThreadPoolExecutor(max_workers=4) as pool:rows=list(pool.map(run,cases))
(P/'smooth-execution.json').write_text(json.dumps(rows,indent=2))
