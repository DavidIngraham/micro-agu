import json,concurrent.futures as cf
from run import P,run
cases=[]
for repeat in [1,2]:
 for strategy in ['gate','zero','weighted']:
  cases.append(dict(id=f'repeat-tail-{strategy}-{repeat}',strategy=strategy,weather='default',north=1.5,east=0,seed=17,group='repeat'))
(P/'confirm-protocol.json').write_text(json.dumps(cases,indent=2))
with cf.ThreadPoolExecutor(max_workers=3) as pool:rows=list(pool.map(run,cases))
(P/'confirm-execution.json').write_text(json.dumps(rows,indent=2))
