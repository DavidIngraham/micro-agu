from pathlib import Path
import json,os,subprocess,hashlib
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2]
cases=[
 dict(id="calm-feedforward",variant="feedforward"),
 dict(id="calm-feedback",variant="feedback"),
 dict(id="head-feedback",variant="feedback",along=-1.5,turbulence=.2),
 dict(id="cross-feedback",variant="feedback",across=1.5,turbulence=.2),
 dict(id="dropout-feedback",variant="feedback",lidar={"dropout_probability":.2}),
 dict(id="failed-sensor",variant="feedback",lidar={"dropout_probability":1}),
]
for d in ["cases","runs"]:(HERE/d).mkdir(exist_ok=True)
paths=["viewer/generated/ardupilot/arduplane.wasm","viewer/generated/physics/sim_wasm_bg.wasm",
 "aircraft/microagu/controllers/lqi-gains.bin","aircraft/microagu/configurations/browser-physics.json",
 "viewer/packages/sim-browser/src/physics/physics.ts","viewer/packages/sim-browser/src/physics/lidar.ts"]
(HERE/"protocol.json").write_text(json.dumps(dict(cases=cases,hashes={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in paths}),indent=2)+"\n")
env=dict(os.environ,PATH="/home/david/emsdk/node/24.19.0_64bit/bin:/usr/bin:/bin")
execution=[]
for case in cases:
    path=HERE/"cases"/(case["id"]+".json");path.write_text(json.dumps(case))
    r=subprocess.run([str(ROOT/"viewer/node_modules/.bin/tsx"),str(HERE/"case.mts"),
                      str(path),str(HERE/"runs"/(case["id"]+".json"))],
                     cwd=ROOT,env=env,capture_output=True,text=True,timeout=390)
    execution.append(dict(id=case["id"],code=r.returncode,stdout=r.stdout,stderr=r.stderr))
    (HERE/"execution.json").write_text(json.dumps(execution,indent=2)+"\n")
    print(case["id"],r.returncode,r.stdout[-500:],r.stderr[-700:],flush=True)
