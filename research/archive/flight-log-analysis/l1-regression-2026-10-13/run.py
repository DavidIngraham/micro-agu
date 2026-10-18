import os

import os,sys,json,subprocess
from pathlib import Path
r=Path(os.environ.get('PG_RESEARCH_ROOT', '/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/l1-regression-2026-10-13'))
env=dict(os.environ);env['PATH']=str(Path(sys.executable).parent)+':'+env['PATH']
for case in json.loads((r/'plan.json').read_text())['mission_cases']:
 d=r/case['name']
 if (d/'completed.json').exists():continue
 run=subprocess.run(['bash','/home/david/repos/ArduPilot-tinkering/research/archive/legacy-tools/run_gust_native.sh','/home/david/repos/ardupilot-high-dof',str(d),case['config'],case['weather']],env=env,timeout=2400)
 (d/'completed.json').write_text(json.dumps(dict(returncode=run.returncode)))
 print(d.name,run.returncode,flush=True)
# Retain failures and continue the planned suite.
for i in range(12):
 d=r/f'landing-{i:02d}';d.mkdir(exist_ok=True)
 if (d/'completed.json').exists():continue
 e=dict(env,BUILDLOGS=str(d),PG_GAINS='mission',PG_CONTROLLER_CONFIG=str(r/'standard-calm-12.json'),PG_LANDING_CONFIG=str(r.parent/'landing-refinement-2026-10-13/validation-configs'/f'{i:02d}.json'))
 with (d/'run.log').open('w') as f:
  run=subprocess.run([sys.executable,'Tools/simulation/paraglider/landing_autotest.py','--no-check-parameter-leaks','test.Plane.ParagliderLandingStudy'],cwd='/home/david/repos/ardupilot-high-dof',env=e,stdout=f,stderr=subprocess.STDOUT,timeout=1200)
 (d/'completed.json').write_text(json.dumps(dict(returncode=run.returncode)))
 print(d.name,run.returncode,flush=True)
