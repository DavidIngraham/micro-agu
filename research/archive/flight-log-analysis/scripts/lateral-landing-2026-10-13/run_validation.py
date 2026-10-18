import os,sys,json,subprocess,time
from pathlib import Path
r=Path('/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/lateral-tuning-2026-10-13');repo=Path('/home/david/repos/ardupilot-high-dof')
source=r.parent/'landing-iteration-2026-10-13/heldout-candidate'
jobs=[]
for i in range(12):
 if i==2:continue
 jobs.append((f'landing-1hz-{i:02d}','Tools/simulation/paraglider/landing_autotest.py','test.Plane.ParagliderLandingStudy',dict(PG_CONTROLLER_CONFIG=str(r/'mission-lateral-1hz.json'),PG_LANDING_CONFIG=str(source/f'moderate-{i:02d}'/'config.json'))))
jobs.append(('mission-1hz','Tools/simulation/paraglider/compare_autotest.py','test.Plane.ParagliderAutoMission',dict(PG_CONTROLLER_CONFIG=str(r/'mission-full-1hz.json'),PG_GAINS='original')))
for name,script,test,extra in jobs:
 d=r/name;d.mkdir(exist_ok=True)
 if (d/'completed.json').exists():continue
 env=dict(os.environ,BUILDLOGS=str(d),**extra)
 started=time.time()
 with (d/'run.log').open('w') as log:
  try:run=subprocess.run([sys.executable,script,'--no-check-parameter-leaks',test],cwd=repo,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=1200);code=run.returncode
  except subprocess.TimeoutExpired:code=-1
 (d/'completed.json').write_text(json.dumps(dict(returncode=code,seconds=time.time()-started,environment=extra),indent=2))
 print(name,code,flush=True)
 if code:break
