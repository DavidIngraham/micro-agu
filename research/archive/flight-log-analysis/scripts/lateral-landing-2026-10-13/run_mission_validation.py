import os,sys,subprocess,json,time
from pathlib import Path
r=Path('/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/lateral-tuning-2026-10-13');repo=Path('/home/david/repos/ardupilot-high-dof');tk=Path('/home/david/repos/ArduPilot-tinkering')
env=dict(os.environ);env['PATH']=str(Path(sys.executable).parent)+':'+env['PATH']
for condition in ['gust','crosswind']:
 d=r/f'mission-1hz-{condition}'
 start=time.time()
 run=subprocess.run(['bash',str(tk/'research/archive/legacy-tools/run_gust_native.sh'),str(repo),str(d),str(r/'mission-full-1hz.json'),condition],env=env,timeout=1200)
 (d/'completed.json').write_text(json.dumps(dict(returncode=run.returncode,seconds=time.time()-start)))
 print(condition,run.returncode,flush=True)
 if run.returncode:break
