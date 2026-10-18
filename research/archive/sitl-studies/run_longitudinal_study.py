"""Sequential native experiments; each case records its full configuration."""
from _sim_path import SIMULATION
import os,json,subprocess,sys
from pathlib import Path
from microagu_sim.compat.firmware import checkout
root=checkout()
out=Path(os.environ.get('MICROAGU_STUDY_OUTPUT','pg-longitudinal-design')).resolve();out.mkdir(exist_ok=True)
cases=json.loads(Path(sys.argv[1]).read_text())
for name,config in cases.items():
 folder=out/name;folder.mkdir(exist_ok=True);p=folder/'config.json';p.write_text(json.dumps(config,indent=2))
 env=os.environ|{'BUILDLOGS':str(folder),'PG_STUDY_CONFIG':str(p),'PG_PROFILE':config.get('profile','mission')}
 with (out/(name+'.log')).open('w') as log:
  result=subprocess.run([sys.executable,'-u',str(Path(__file__).with_name('longitudinal_autotest.py')),'--no-check-parameter-leaks','--speedup','10','test.Plane.ParagliderLongitudinalStudy'],cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT)
 print(name,result.returncode,flush=True)
 if result.returncode:sys.exit(result.returncode)
