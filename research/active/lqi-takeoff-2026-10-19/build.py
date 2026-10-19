import hashlib,json,os,shutil,subprocess
from pathlib import Path
P=Path(__file__).resolve().parent
AP=Path('/home/david/repos/ardupilot-lqi-pipeline')
source=AP/'ArduPlane/Plane.cpp'
original=source.read_bytes()
assert not subprocess.check_output(['git','status','--porcelain','--untracked-files=no'],cwd=AP).strip()
try:
 subprocess.run(['git','apply',str(P/'takeoff.patch')],cwd=AP,check=True)
 env=os.environ|{'PATH':'/home/david/envs/venv-ardupilot/bin:/usr/bin:/bin'}
 cmd='source /home/david/emsdk/emsdk_env.sh && export EMCC_CFLAGS="-sEXPORTED_RUNTIME_METHODS=cwrap,HEAPU8,FS,SOCKFS,PThread" && ./waf configure --board wasm && ./waf plane -j4'
 with (P/'build.log').open('w') as log:subprocess.run(['bash','-c',cmd],cwd=AP,env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
 out=P/'runtime';out.mkdir(exist_ok=True)
 for ext in ['js','wasm']:shutil.copyfile(AP/f'build/wasm/bin/arduplane.{ext}',out/f'arduplane.{ext}')
 (out/'package.json').write_text('{"type":"module"}')
 sha=lambda f:hashlib.sha256(f.read_bytes()).hexdigest()
 (P/'build-manifest.json').write_text(json.dumps(dict(base=subprocess.check_output(['git','rev-parse','HEAD'],cwd=AP,text=True).strip(),patch_sha256=sha(P/'takeoff.patch'),patched_source_sha256=sha(source),firmware_sha256=sha(out/'arduplane.wasm'),ai_assisted=True),indent=2))
finally:
 source.write_bytes(original)
