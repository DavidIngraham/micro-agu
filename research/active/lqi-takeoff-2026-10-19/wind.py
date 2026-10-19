import concurrent.futures as cf,json
from run import P,run
cases=[]
for variant in ['deployed','candidate']:
 for name,north,east in [('headwind',-1.5,0),('crosswind',0,1.5)]:
  cases.append(dict(id=f'{variant}-{name}',variant=variant,weather='default',seed=73,north=north,east=east))
(P/'wind-protocol.json').write_text(json.dumps(cases,indent=2))
with cf.ThreadPoolExecutor(max_workers=2) as pool:rows=list(pool.map(run,cases))
(P/'wind-execution.json').write_text(json.dumps(rows,indent=2))
