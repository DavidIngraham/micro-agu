import os, subprocess, json, sys
from pathlib import Path
r=Path('/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/lateral-tuning-2026-10-13')
repo=Path('/home/david/repos/ardupilot-high-dof')
source=Path('/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/landing-iteration-2026-10-13/heldout-candidate')
for i in [0,2,3]:
 d=r/f'landing-mission-lateral-{i:02d}';d.mkdir(exist_ok=True)
 env=dict(os.environ,PG_CONTROLLER_CONFIG=str(r/'mission-lateral-only.json'),PG_LANDING_CONFIG=str(source/f'moderate-{i:02d}'/'config.json'),BUILDLOGS=str(d))
 with (d/'run.log').open('w') as log:
  run=subprocess.run([sys.executable,'Tools/simulation/paraglider/landing_autotest.py','--no-check-parameter-leaks','test.Plane.ParagliderLandingStudy'],cwd=repo,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=360)
 print(d.name,run.returncode,flush=True)
 if run.returncode:break
