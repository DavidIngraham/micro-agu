import os,sys,json,subprocess,time
from pathlib import Path
r=Path(__file__).parent;repo=Path('/home/david/repos/ardupilot-high-dof')
source=r.parent/'landing-iteration-2026-10-13/heldout-candidate'
for i in range(12):
 for hz in [4,1]:
  d=r/f'landing-{hz}hz-{i:02d}';d.mkdir(exist_ok=True)
  if (d/'completed.json').exists():continue
  extra=dict(PG_CONTROLLER_CONFIG=str(r/f'controller-{hz}hz.json'),PG_LANDING_CONFIG=str(source/f'moderate-{i:02d}'/'config.json'),PG_GAINS='mission')
  env=dict(os.environ,BUILDLOGS=str(d),**extra);env['PATH']=str(Path(sys.executable).parent)+':'+env['PATH']
  start=time.time()
  with (d/'run.log').open('w') as log:
   run=subprocess.run([sys.executable,'Tools/simulation/paraglider/landing_autotest.py','--no-check-parameter-leaks','test.Plane.ParagliderLandingStudy'],cwd=repo,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=1200)
  (d/'completed.json').write_text(json.dumps(dict(returncode=run.returncode,seconds=time.time()-start,environment=extra)))
  print(d.name,run.returncode,flush=True)
  if run.returncode:sys.exit(run.returncode)
