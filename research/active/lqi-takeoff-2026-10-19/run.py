import concurrent.futures as cf,json,os,subprocess,time
from pathlib import Path
P=Path(__file__).resolve().parent;ROOT=P.parents[2]
def run(spec):
 (P/'cases').mkdir(exist_ok=True);(P/'runs').mkdir(exist_ok=True)
 case=P/'cases'/f"{spec['id']}.json";out=P/'runs'/f"{spec['id']}.json"
 case.write_text(json.dumps(spec,indent=2));start=time.time()
 with out.with_suffix('.log').open('w') as log:
  r=subprocess.run([str(ROOT/'viewer/node_modules/.bin/tsx'),str(P/'case.mts'),str(case),str(out)],cwd=ROOT,env=os.environ|{'PATH':'/home/david/emsdk/node/24.19.0_64bit/bin:/usr/bin:/bin'},stdout=log,stderr=subprocess.STDOUT,timeout=260)
 d=json.loads(out.read_text()) if out.exists() else {}
 row=dict(id=spec['id'],exit=r.returncode,complete=d.get('complete'),failure=d.get('failure'),wall_s=time.time()-start)
 print(row,flush=True);return row
if __name__=='__main__':
 cases=[dict(id=f'{variant}-calm',variant=variant,weather='calm',seed=17) for variant in ['deployed','candidate']]
 (P/'protocol.json').write_text(json.dumps(cases,indent=2))
 with cf.ThreadPoolExecutor(max_workers=2) as pool:rows=list(pool.map(run,cases))
 (P/'execution.json').write_text(json.dumps(rows,indent=2))
