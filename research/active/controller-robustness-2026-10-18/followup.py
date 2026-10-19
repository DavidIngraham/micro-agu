import json,subprocess,os,time,concurrent.futures as cf
from pathlib import Path
P=Path(__file__).resolve().parent;ROOT=P.parents[2];NODE='/home/david/emsdk/node/24.19.0_64bit/bin'
cases=[]
for gain in [-.12,-.18]:
 for scheme in ['baseline','lqi']:cases.append(dict(id=f'boundary-{gain}-{scheme}',scheme=scheme,experiment=dict(kind='mode',hz=1.6,amplitude=.005,ratio=.4,phase=0.,damping=.05,gain=gain)))
for scheme in ['baseline','lqi']:
 s=json.loads((P/'cases'/f'force-2-{scheme}.json').read_text());s['id']=f'repeat-force-2-{scheme}';cases.append(s)
def run(spec):
 file=P/'cases'/(spec['id']+'.json');file.write_text(json.dumps(spec,indent=2));out=P/'runs'/(spec['id']+'.json');start=time.time()
 with out.with_suffix('.log').open('w') as f:
  try:code=subprocess.run([str(ROOT/'viewer/node_modules/.bin/tsx'),str(P/'case.mts'),str(file),str(out)],cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,env=os.environ|{'PATH':NODE+':/usr/bin:/bin'},timeout=330).returncode
  except subprocess.TimeoutExpired:code=124
 r=json.loads(out.read_text()) if out.exists() else {};row=dict(id=spec['id'],exit=code,complete=r.get('complete',False),failure=r.get('failure','Missing output'),wall_s=time.time()-start);print(row,flush=True);return row
if __name__=='__main__':
 (P/'followup-protocol.json').write_text(json.dumps(dict(reason='Bracket fixed-control stability difference between gain -0.08 and -0.3 at 1.6 Hz; repeat the observed-phase forcing pair.',cases=cases),indent=2))
 with cf.ThreadPoolExecutor(max_workers=2) as pool:rows=list(pool.map(run,cases))
 (P/'followup-execution.json').write_text(json.dumps(rows,indent=2))
