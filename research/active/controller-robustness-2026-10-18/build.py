import subprocess,shutil,json,hashlib
from pathlib import Path
P=Path(__file__).resolve().parent;ROOT=P.parents[2]
subprocess.run(['/home/david/.cargo/bin/cargo','build','--release','--target','wasm32-unknown-unknown'],cwd=P,check=True)
subprocess.run(['/home/david/.cargo/bin/wasm-bindgen','target/wasm32-unknown-unknown/release/robustness.wasm','--target','web','--out-dir','physics','--out-name','sim_wasm_extra'],cwd=P,check=True)
(P/'runtime').mkdir(exist_ok=True)
for suffix in ['js','wasm']:shutil.copyfile(ROOT/f'viewer/generated/ardupilot/arduplane.{suffix}',P/f'runtime/arduplane.{suffix}')
for folder in ['runtime','physics']:(P/folder/'package.json').write_text('{"type":"module"}')
shutil.copyfile(P/'wrapper.mjs',P/'physics/sim_wasm.js');shutil.copyfile(P/'physics/sim_wasm_extra_bg.wasm',P/'physics/sim_wasm_bg.wasm')
manifest=json.loads((ROOT/'aircraft/microagu/current.json').read_text());sha=lambda f:hashlib.sha256(f.read_bytes()).hexdigest()
assert sha(P/'runtime/arduplane.wasm')==manifest['published']['firmware_sha256']
assert sha(ROOT/'aircraft/microagu/controllers/lqi-gains.bin')==manifest['sha256']['controllers/lqi-gains.bin']
print('Built isolated physics; firmware and gains match selected release.')
