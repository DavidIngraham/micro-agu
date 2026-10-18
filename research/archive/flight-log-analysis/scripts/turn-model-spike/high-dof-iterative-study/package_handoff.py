"""Package a current Rust wheel and explicit configs without rewriting old results."""
import argparse
import hashlib
import json
import subprocess
from pathlib import Path
import zipfile

from prepare_package import ROOT, DEFAULT_OUTPUT, build


def package(output):
    wheel = build(output)
    files = {wheel.name: wheel.read_bytes()}
    for path in sorted((ROOT/'simulation/data/presets').glob('*.json')):
        files['presets/'+path.name] = path.read_bytes()
    files['README.txt'] = (
        'MicroAGU Rust simulator handoff.\n'
        'Install on the wheel platform: pip install "./microagu_sim-*.whl[analysis]"\n'
        '(Replace the wildcard with the exact wheel filename.)\n'
        'Python: from microagu_sim import Simulator\n'
        'sim = Simulator(open("presets/as-built-trout-lake.json").read())\n'
        'sim.step([0.4, 0.0, 0.0], [0.0, 0.0, 0.0], 0.005)\n'
        'Legacy mission/dynamic_roll helpers: microagu_sim.compat.profiles.make_simulator.\n'
        'Historical study outputs are evidence from their recorded source versions;\n'
        'this package does not claim to regenerate those results unchanged.\n'
    ).encode()
    files['COPYING.txt'] = (ROOT/'COPYING.txt').read_bytes()
    files['RUST-NOTICES.txt'] = (ROOT/'viewer/assets/runtime/RUST-NOTICES.txt').read_bytes()
    manifest = dict(source_revision=subprocess.check_output(
        ['git','rev-parse','HEAD'], cwd=ROOT, text=True).strip(),
        files={name:hashlib.sha256(data).hexdigest() for name,data in files.items()})
    files['manifest.json'] = (json.dumps(manifest,indent=2)+'\n').encode()
    dest = Path(output)/'microagu-rust-handoff.zip'
    with zipfile.ZipFile(dest,'w',zipfile.ZIP_DEFLATED) as archive:
        for name,data in files.items():
            archive.writestr(name,data)
    return dest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=DEFAULT_OUTPUT)
    print(package(parser.parse_args().output))
