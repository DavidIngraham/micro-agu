import os

import os,sys,json,subprocess
from pathlib import Path
r=Path(os.environ.get('PG_RESEARCH_ROOT', '/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/landing-guidance-2026-10-13'))
old=r.parent/'landing-refinement-2026-10-13'
for period in [12,16,20]:
 d=r/f'calm-{period}';d.mkdir(exist_ok=True)
 env=dict(os.environ,BUILDLOGS=str(d),PG_CONTROLLER_CONFIG=str(r/f'controller-{period}.json'),PG_LANDING_CONFIG=str(old/'preview-calm-config.json'),PG_GAINS='mission')
 env['PATH']=str(Path(sys.executable).parent)+':'+env['PATH']
 with (d/'run.log').open('w') as log:
  run=subprocess.run([sys.executable,'Tools/simulation/paraglider/landing_autotest.py','--no-check-parameter-leaks','test.Plane.ParagliderLandingStudy'],cwd='/home/david/repos/ardupilot-high-dof',env=env,stdout=log,stderr=subprocess.STDOUT,timeout=1200)
 (d/'completed.json').write_text(json.dumps(dict(returncode=run.returncode)))
 print(d.name,run.returncode,flush=True)
 if run.returncode:sys.exit(run.returncode)
