"""Run one named native swoop case; preserve completed results."""
import os,json,sys,subprocess
from pathlib import Path
root=Path('/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/swoop-2026-10-13')
source=Path(__file__).parent
configpath=Path(sys.argv[1]);config=json.loads(configpath.read_text());out=root/config['name']
assert not (out/'completed.json').exists(),'Case already completed'
out.mkdir(exist_ok=True)
env=dict(os.environ,BUILDLOGS=str(out),PG_LANDING_CONFIG=str(configpath.resolve()),PG_CONTROLLER_CONFIG='/home/david/repos/ardupilot-high-dof/Tools/simulation/paraglider/mission-controller-candidate.json')
env['PATH']=str(Path(sys.executable).parent)+':'+env['PATH']
with (out/'run.log').open('w') as f:
 p=subprocess.run([sys.executable,str(source/'native.py'),'--no-check-parameter-leaks','test.Plane.ParagliderSwoopStudy'],cwd='/home/david/repos/ardupilot-high-dof',env=env,stdout=f,stderr=subprocess.STDOUT,timeout=1200)
(out/'completed.json').write_text(json.dumps({'returncode':p.returncode}))
print(config['name'],p.returncode,flush=True)
if (out/'landing.json').exists():
 r=json.loads((out/'landing.json').read_text());print('miss',r.get('radial_error_m'),'contact',r['contact'],flush=True)
else:print((out/'run.log').read_text()[-2500:])