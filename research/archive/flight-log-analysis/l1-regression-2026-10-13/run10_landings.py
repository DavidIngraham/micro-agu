import os

import os,sys,json,subprocess
from pathlib import Path
r=Path(os.environ.get('PG_RESEARCH_ROOT', '/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/l1-regression-2026-10-13'))
env=dict(os.environ);env['PATH']=str(Path(sys.executable).parent)+':'+env['PATH']
import time
while not (r/'case08-10-repeat/completed.json').exists():time.sleep(2)
for i in range(12):
 d=r/f'landing10-{i:02d}';d.mkdir(exist_ok=True)
 if (d/'completed.json').exists():continue
 e=dict(env,BUILDLOGS=str(d),PG_GAINS='mission',PG_CONTROLLER_CONFIG=str(r/'candidate-10.json'),PG_LANDING_CONFIG=str(r.parent/'landing-refinement-2026-10-13/validation-configs'/f'{i:02d}.json'))
 with (d/'run.log').open('w') as f:
  run=subprocess.run([sys.executable,'Tools/simulation/paraglider/landing_autotest.py','--no-check-parameter-leaks','test.Plane.ParagliderLandingStudy'],cwd='/home/david/repos/ardupilot-high-dof',env=e,stdout=f,stderr=subprocess.STDOUT,timeout=1200)
 (d/'completed.json').write_text(json.dumps(dict(returncode=run.returncode)))
 print(d.name,run.returncode,flush=True)
