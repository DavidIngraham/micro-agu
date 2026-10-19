import json,concurrent.futures as cf
from followup import P,run
cases=[]
for name in ['boundary--0.18-baseline','boundary--0.18-lqi','mode-2.1-0.3-baseline','mode-2.1-0.3-lqi']:
 s=json.loads((P/'cases'/f'{name}.json').read_text());s['id']='confirm-'+name;cases.append(s)
(P/'confirm-protocol.json').write_text(json.dumps(dict(reason='Repeat responsive cases with divergent or sustained closed-loop motion, including a fixed-control stable case.',cases=cases),indent=2))
with cf.ThreadPoolExecutor(max_workers=2) as pool:rows=list(pool.map(run,cases))
(P/'confirm-execution.json').write_text(json.dumps(rows,indent=2))
