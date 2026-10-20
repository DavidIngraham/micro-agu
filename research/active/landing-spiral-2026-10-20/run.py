import concurrent.futures,json,os,subprocess
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
def run(name):
    env=dict(os.environ,PATH="/home/david/emsdk/node/24.19.0_64bit/bin:/usr/bin:/bin")
    spec={"id":"spiral-"+name,"fixedMission":True}
    if name=="crosswind": spec.update(across=1.5,turbulence=.2)
    (HERE/"runs").mkdir(exist_ok=True)
    config=HERE/"runs"/(name+".json");config.write_text(json.dumps(spec))
    with (HERE/"runs"/(name+".log")).open("w") as log:
      result=subprocess.run([str(ROOT/"viewer/node_modules/.bin/tsx"),str(HERE.parent/"native-landing-2026-10-20/case.mts"),str(config),str(HERE/"runs"/(name+"-result.json"))],cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=400)
    print(name,result.returncode,flush=True)
    return result.returncode
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
    raise SystemExit(any(list(pool.map(run,["calm","crosswind"]))))
