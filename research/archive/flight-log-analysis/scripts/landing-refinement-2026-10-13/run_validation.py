import os,sys,json,subprocess,time
from pathlib import Path
r=Path(__file__).parent;repo=Path('/home/david/repos/ardupilot-high-dof')
source=r.parent/'landing-iteration-2026-10-13/heldout-candidate'
for i in range(12):
 for hz in [1]:
  d=r/f'validation-{i:02d}';d.mkdir(exist_ok=True)
  if (d/'completed.json').exists():continue
  extra=dict(PG_CONTROLLER_CONFIG=str(r/f'controller-{hz}hz.json'),PG_LANDING_CONFIG=str(r/'validation-configs'/f'{i:02d}.json'),PG_GAINS='mission')
  env=dict(os.environ,BUILDLOGS=str(d),**extra);env['PATH']=str(Path(sys.executable).parent)+':'+env['PATH']
  start=time.time()
  with (d/'run.log').open('w') as log:
   run=subprocess.run([sys.executable,'Tools/simulation/paraglider/landing_autotest.py','--no-check-parameter-leaks','test.Plane.ParagliderLandingStudy'],cwd=repo,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=1200)
  (d/'completed.json').write_text(json.dumps(dict(returncode=run.returncode,seconds=time.time()-start,environment=extra)))
  print(d.name,run.returncode,flush=True)
  if run.returncode:sys.exit(run.returncode)

# Complete the missing current-profile crosswind baseline, using native AUTO.
d=r/'mission-4hz-crosswind'
env=dict(os.environ);env['PATH']=str(Path(sys.executable).parent)+':'+env['PATH']
run=subprocess.run(['bash','/home/david/repos/ArduPilot-tinkering/research/archive/legacy-tools/run_gust_native.sh',str(repo),str(d),str(r/'controller-4hz.json'),'crosswind'],env=env,timeout=1200)
(d/'completed.json').write_text(json.dumps(dict(returncode=run.returncode)))
print('mission-4hz-crosswind',run.returncode,flush=True)
