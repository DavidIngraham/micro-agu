import os

import os,sys,json,subprocess,time
from pathlib import Path
r=Path(os.environ.get('PG_RESEARCH_ROOT', '/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/landing-guidance-2026-10-13'))
while not (r/'calm-20/completed.json').exists():time.sleep(2)
for name,period in [['steady', 12], ['gust', 12], ['stress06', 12], ['stress02', 12], ['steady', 8]]:
 d=r/f'check-{name}-{period}';d.mkdir(exist_ok=True)
 controller=r/f'controller-{period}.json' if period!=8 else r.parent/'landing-refinement-2026-10-13/controller-1hz.json'
 env=dict(os.environ,BUILDLOGS=str(d),PG_CONTROLLER_CONFIG=str(controller),PG_LANDING_CONFIG=str(r/f'{name}.json'),PG_GAINS='mission')
 env['PATH']=str(Path(sys.executable).parent)+':'+env['PATH']
 with (d/'run.log').open('w') as log:
  run=subprocess.run([sys.executable,'Tools/simulation/paraglider/landing_autotest.py','--no-check-parameter-leaks','test.Plane.ParagliderLandingStudy'],cwd='/home/david/repos/ardupilot-high-dof',env=env,stdout=log,stderr=subprocess.STDOUT,timeout=1200)
 (d/'completed.json').write_text(json.dumps(dict(returncode=run.returncode)))
 print(d.name,run.returncode,flush=True)
 if run.returncode:sys.exit(run.returncode)
