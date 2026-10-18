"""Record exact Rust source/artifact hashes and dependency license texts."""
from pathlib import Path
import hashlib
import json
import subprocess

root = Path(__file__).resolve().parents[1]
destination = root.parent / 'viewer/assets/runtime'
manifest_path = destination / 'build.json'
manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
sources = [root/'Cargo.toml', root/'Cargo.lock']
sources += sorted((root/'crates').rglob('*.rs'))
sources += sorted((root/'crates').rglob('Cargo.toml'))
hashes = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
revision = subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip()
dirty = bool(subprocess.check_output(['git','status','--porcelain','--','Cargo.toml','Cargo.lock','crates'],cwd=root,text=True).strip())
manifest['model'] = 'JSON:127.0.0.1 + Rust full 9-DOF external model'
manifest['physics_source'] = dict(repository='https://github.com/DavidIngraham/micro-agu', revision=revision,
    working_tree_dirty=dirty, language='Rust', source_sha256=hashes,
    rustc=subprocess.check_output(['rustc','--version'],text=True).strip(), wasm_bindgen='0.2.114')
manifest['sha256'] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in [*destination.iterdir(), *(root.parent/'viewer/generated/physics').iterdir(), *(root.parent/'viewer/generated/ardupilot').iterdir()]
    if p.name.startswith(('arduplane.','sim_wasm.','sim_wasm_bg.','highdof.json','lqi-gains.bin'))}
manifest_path.write_text(json.dumps(manifest,indent=2)+'\n')
metadata = json.loads(subprocess.check_output(['cargo','metadata','--locked','--format-version','1','--filter-platform','wasm32-unknown-unknown'],cwd=root,text=True))
packages = {p['id']:p for p in metadata['packages']}
edges = {n['id']:[d['pkg'] for d in n['deps']] for n in metadata['resolve']['nodes']}
pending = [next(k for k,p in packages.items() if p['name']=='sim-wasm')]
visited = set()
notices = ['Rust simulator dependency notices. Project source is GPL-3.0-or-later.\n']
while pending:
    key=pending.pop()
    if key in visited:continue
    visited.add(key);pending.extend(edges.get(key,[]))
    package=packages[key]
    if package['source'] is None:continue
    directory=Path(package['manifest_path']).parent
    notices.append(f"\n{package['name']} {package['version']} — {package['license']}\n{package.get('repository') or ''}\n")
    files=sorted({*directory.glob('LICENSE*'), *directory.glob('COPYING*')})
    for file in files:
        if file.is_file(): notices.append(file.read_text(errors='replace')+'\n')
(destination/'RUST-NOTICES.txt').write_text(''.join(notices).rstrip()+'\n')
