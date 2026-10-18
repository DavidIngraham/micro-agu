import os,sys,json,subprocess,time
from pathlib import Path
r=Path(__file__).parent
while not (r/'default-controller-check/completed.json').exists():time.sleep(5)
assert json.loads((r/'default-controller-check/completed.json').read_text())['returncode']==0
for hz in [1]:
 d=r/'preview-calm';d.mkdir(exist_ok=True)
 env=dict(os.environ,BUILDLOGS=str(d),PG_CONTROLLER_CONFIG=str(r/f'controller-{hz}hz.json'),PG_LANDING_CONFIG=str(r/'preview-calm-config.json'),PG_GAINS='mission')
 env['PATH']=str(Path(sys.executable).parent)+':'+env['PATH']
 with (d/'run.log').open('w') as log:
  run=subprocess.run([sys.executable,'Tools/simulation/paraglider/landing_autotest.py','--no-check-parameter-leaks','test.Plane.ParagliderLandingStudy'],cwd='/home/david/repos/ardupilot-high-dof',env=env,stdout=log,stderr=subprocess.STDOUT,timeout=1200)
 (d/'completed.json').write_text(json.dumps(dict(returncode=run.returncode)))
 print(d.name,run.returncode,flush=True)
 if run.returncode:sys.exit(run.returncode)
