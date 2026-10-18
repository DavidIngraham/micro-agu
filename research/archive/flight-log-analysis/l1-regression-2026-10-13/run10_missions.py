import os

import os,sys,json,subprocess
from pathlib import Path
r=Path(os.environ.get('PG_RESEARCH_ROOT', '/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/l1-regression-2026-10-13'))
env=dict(os.environ);env['PATH']=str(Path(sys.executable).parent)+':'+env['PATH']
for case in json.loads((r/'plan.json').read_text())['mission_cases']:
 if not case['name'].endswith('-10'):continue
 d=r/case['name']
 if (d/'completed.json').exists():continue
 run=subprocess.run(['bash','/home/david/repos/ArduPilot-tinkering/research/archive/legacy-tools/run_gust_native.sh','/home/david/repos/ardupilot-high-dof',str(d),case['config'],case['weather']],env=env,timeout=2400)
 (d/'completed.json').write_text(json.dumps(dict(returncode=run.returncode)))
 print(d.name,run.returncode,flush=True)
