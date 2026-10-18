import os

import os,sys,json,subprocess
from pathlib import Path
r=Path(os.environ.get('PG_RESEARCH_ROOT', '/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/l1-regression-2026-10-13'))
import time
while not (r/'case08-20/completed.json').exists():time.sleep(2)
for name,configpath in [('calm10','preview-calm-config.json'),('case08-10','validation-configs/08.json'),('case08-10-repeat','validation-configs/08.json')]:
 period=10
 d=r/name;d.mkdir(exist_ok=True)
 config=r/('standard-calm-12.json' if period==12 else f'candidate-{period}.json')
 env=dict(os.environ,BUILDLOGS=str(d),PG_CONTROLLER_CONFIG=str(config),PG_GAINS='mission',PG_LANDING_CONFIG=str(r.parent/'landing-refinement-2026-10-13'/configpath))
 env['PATH']=str(Path(sys.executable).parent)+':'+env['PATH']
 with (d/'run.log').open('w') as f:
  run=subprocess.run([sys.executable,'Tools/simulation/paraglider/landing_autotest.py','--no-check-parameter-leaks','test.Plane.ParagliderLandingStudy'],cwd='/home/david/repos/ardupilot-high-dof',env=env,stdout=f,stderr=subprocess.STDOUT,timeout=1200)
 (d/'completed.json').write_text(json.dumps(dict(returncode=run.returncode)));print(d.name,run.returncode,flush=True)
