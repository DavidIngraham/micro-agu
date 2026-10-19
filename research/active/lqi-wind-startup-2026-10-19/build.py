import hashlib,json,os,shutil,subprocess,sys
from pathlib import Path
P=Path(__file__).resolve().parent
AP=Path('/home/david/repos/ardupilot-lqi-pipeline')
patch=P/(sys.argv[1] if len(sys.argv)>1 else 'startup.patch')
names=['ArduPlane/Plane.cpp','libraries/AP_CustomControl/AP_CustomControl_MIMO.h','libraries/AP_CustomControl/AP_CustomControl_LQI.h','libraries/AP_CustomControl/AP_CustomControl_LQI.cpp']
original={n:(AP/n).read_bytes() for n in names}
assert not subprocess.check_output(['git','status','--porcelain','--untracked-files=no'],cwd=AP).strip()
try:
 subprocess.run(['git','apply',str(patch)],cwd=AP,check=True)
 env=os.environ|{'PATH':'/home/david/envs/venv-ardupilot/bin:/usr/bin:/bin'}
 cmd='source /home/david/emsdk/emsdk_env.sh && export EMCC_CFLAGS="-sEXPORTED_RUNTIME_METHODS=cwrap,HEAPU8,FS,SOCKFS,PThread" && ./waf configure --board wasm && ./waf plane -j4'
 with (P/'build.log').open('w') as log:subprocess.run(['bash','-c',cmd],cwd=AP,env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
 out=P/'runtime';out.mkdir(exist_ok=True)
 for ext in ['js','wasm']:shutil.copyfile(AP/f'build/wasm/bin/arduplane.{ext}',out/f'arduplane.{ext}')
 (out/'package.json').write_text('{"type":"module"}')
 sha=lambda f:hashlib.sha256(f.read_bytes()).hexdigest()
 (P/'build-manifest.json').write_text(json.dumps(dict(base=subprocess.check_output(['git','rev-parse','HEAD'],cwd=AP,text=True).strip(),patch_sha256=sha(patch),patched_source_sha256={n:sha(AP/n) for n in names},firmware_sha256=sha(out/'arduplane.wasm'),ai_assisted=True),indent=2))
finally:
 for n,data in original.items():(AP/n).write_bytes(data)
