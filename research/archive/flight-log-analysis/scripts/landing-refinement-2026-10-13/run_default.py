import os,sys,json,subprocess,time
from pathlib import Path
r=Path(__file__).parent
while not (r/'mission-4hz-crosswind/completed.json').exists():time.sleep(5)
assert json.loads((r/'mission-4hz-crosswind/completed.json').read_text())['returncode']==0
d=r/'default-controller-check';d.mkdir(exist_ok=True)
env=dict(os.environ,BUILDLOGS=str(d),PG_LANDING_CONFIG=str(r.parent/'landing-iteration-2026-10-13/heldout-candidate/moderate-01/config.json'))
for k in ['PG_GAINS','PG_CONTROLLER_CONFIG']:env.pop(k,None)
env['PATH']=str(Path(sys.executable).parent)+':'+env['PATH']
with (d/'run.log').open('w') as log:
 run=subprocess.run([sys.executable,'Tools/simulation/paraglider/landing_autotest.py','--no-check-parameter-leaks','test.Plane.ParagliderLandingStudy'],cwd='/home/david/repos/ardupilot-high-dof',env=env,stdout=log,stderr=subprocess.STDOUT,timeout=1200)
(d/'completed.json').write_text(json.dumps(dict(returncode=run.returncode)))
print('default-controller-check',run.returncode,flush=True)
