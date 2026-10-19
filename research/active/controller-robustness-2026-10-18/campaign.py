import json,subprocess,os,time,concurrent.futures as cf
from pathlib import Path
P=Path(__file__).resolve().parent;ROOT=P.parents[2];NODE='/home/david/emsdk/node/24.19.0_64bit/bin'
def main():
 forcing=json.loads((P/'forcing-calibration.json').read_text());base=dict(kind='forcing',hz=1.6,amplitude=0.,ratio=.4,phase=-1.57079632679,damping=.1,gain=0.)
 cases=[dict(id='quiet',experiment=base),dict(id='quiet-maneuver',experiment=base,maneuver=True)]
 for i,e in enumerate(forcing):cases.append(dict(id=f'force-{i}',experiment=e))
 for i in [2,4]:
  e=forcing[i].copy();e['amplitude']*=2;cases.append(dict(id=f'force-{i}-double',experiment=e))
 cases.append(dict(id='force-maneuver',experiment=forcing[2],maneuver=True))
 for hz in [1.6,2.1]:
  for gain in [-.3,-.08,.08,.3]:
   e=dict(kind='mode',hz=hz,amplitude=.005,ratio=.4,phase=0.,damping=.05,gain=gain);cases.append(dict(id=f'mode-{hz}-{gain}',experiment=e))
 for gain in [-.3,.3]:cases.append(dict(id=f'mode-damped-{gain}',experiment=dict(kind='mode',hz=1.6,amplitude=.005,ratio=.4,phase=0.,damping=.2,gain=gain),maneuver=True))
 (P/'cases').mkdir(exist_ok=True);(P/'runs').mkdir(exist_ok=True);(P/'protocol.json').write_text(json.dumps(dict(cases=cases,schemes=['baseline','lqi'],duration_s=85,forcing_interval=[10,50],recovery_interval=[60,85],guards=dict(min_height_m=15,roll_deg=60,pitch_deg=45),note='Shared forcing calibrated at fixed trim; responsive phenomenological mode remains enabled after its two-second seed torque, with uncertain signed feedback. No controller tuning.'),indent=2))
 jobs=[]
 for case in cases:
  for scheme in ['baseline','lqi']:
   spec=case|dict(id=case['id']+'-'+scheme,scheme=scheme);file=P/'cases'/(spec['id']+'.json');file.write_text(json.dumps(spec,indent=2));jobs.append((spec,file))
 def run(job):
  spec,file=job;out=P/'runs'/(spec['id']+'.json');log=out.with_suffix('.log');start=time.time()
  with log.open('w') as f:
   try:r=subprocess.run([str(ROOT/'viewer/node_modules/.bin/tsx'),str(P/'case.mts'),str(file),str(out)],cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,env=os.environ|{'PATH':NODE+':/usr/bin:/bin'},timeout=330).returncode
   except subprocess.TimeoutExpired:r=124
  result=json.loads(out.read_text()) if out.exists() else {};row=dict(id=spec['id'],exit=r,complete=result.get('complete',False),failure=result.get('failure','missing output'),wall_s=time.time()-start);print(row,flush=True);return row
 rows=[]
 with cf.ThreadPoolExecutor(max_workers=4) as pool:
  for r in pool.map(run,jobs):rows.append(r);(P/'execution.json').write_text(json.dumps(rows,indent=2))
if __name__=='__main__':main()
