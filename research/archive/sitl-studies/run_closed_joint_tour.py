"""Generate the settled native AUTO 9-degree comparison suite; retain raw logs outside Git."""
import argparse, os, subprocess, sys, json, time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--repo',type=Path,required=True);p.add_argument('--raw',type=Path,required=True);p.add_argument('--model-dir',type=Path,default=Path(__file__).resolve().parents[3]/'simulation');a=p.parse_args()
script=Path(__file__).with_name('make_closed_joint_tour.py')
for profile in ['mission','dynamic_roll']:
 for body in ['canopy','payload']:
  for axis in ['roll','pitch','yaw']:
   directory=a.raw/f'{profile}-{body}-{axis}-9deg';directory.mkdir(parents=True,exist_ok=True)
   result=directory/'result.json'
   if result.exists():
    existing=json.loads(result.read_text())
    assert existing.get('mode')=='AUTO' and existing.get('settle_s')==100, f'Use a new output directory: {directory}'
    continue
   env=dict(os.environ,BUILDLOGS=str(directory.resolve()),ARDUPILOT_ROOT=str(a.repo.resolve()))
   with (directory/'run.log').open('w') as log:
    cmd=[sys.executable,str(script.resolve()),'--model-dir',str(a.model_dir.resolve()),'--output',str(result.resolve()),'--profile',profile,'--body',body,'--axis',axis,'--amplitude','9','--','--no-check-parameter-leaks','--speedup','10','test.Plane.ParagliderAutoMission']
    run=subprocess.run(cmd,cwd=a.repo,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=600)
   if run.returncode or not result.exists(): raise RuntimeError(f'Failed {directory}; inspect run.log')
   print('Completed',directory.name,flush=True)
