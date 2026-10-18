"""Restore campaign runtimes from Git and validate frozen input hashes."""
import argparse
import hashlib
import json
import subprocess
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--fixed-ref", default="HEAD", help="microAGU release containing the fixed WASM")
args = parser.parse_args()
manifest = json.loads((HERE / "runtime-manifest.json").read_text())
for folder, hashes in manifest["runtime_sha256"].items():
    ref = manifest["original_microagu_commit"] if folder == "runtime-original" else args.fixed_ref
    (HERE / folder).mkdir(exist_ok=True)
    for name, expected in hashes.items():
        target = HERE / folder / name
        data = target.read_bytes() if target.exists() else subprocess.check_output(
            ["git", "show", f"{ref}:viewer/generated/ardupilot/{name}"], cwd=ROOT)
        assert hashlib.sha256(data).hexdigest() == expected, f"Wrong runtime: {folder}/{name}"
        target.write_bytes(data)
for path, key in [(HERE / "physics-config.json", "physics_config_sha256"),
                  (ROOT / "viewer/generated/physics/sim_wasm_bg.wasm", "physics_wasm_sha256")]:
    assert hashlib.sha256(path.read_bytes()).hexdigest() == manifest[key], f"Wrong physics: {path}"
print("Frozen runtimes and physics verified.")
