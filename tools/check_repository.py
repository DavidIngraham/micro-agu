"""Check package direction and the selected aircraft/release integrity."""
import ast
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

def check():
    errors = []
    aircraft = ROOT / "aircraft/microagu"
    current = json.loads((aircraft / "current.json").read_text())
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    for name, expected in current["sha256"].items():
        path = aircraft / name
        if not path.is_file() or digest(path) != expected:
            errors.append(f"Selected artifact changed: {name}")
    for source, staged in current["staged_runtime"].items():
        if not (ROOT / staged).is_file() or digest(aircraft / source) != digest(ROOT / staged):
            errors.append(f"Staged runtime differs from selected aircraft: {staged}")
    build = json.loads((ROOT / "viewer/assets/runtime/build.json").read_text())
    if build["revision"] != current["controller"]["firmware_revision"]:
        errors.append("Firmware revision differs from selected release")
    if build["sha256"]["lqi-gains.bin"] != digest(aircraft / current["controller"]["gain_file"]):
        errors.append("Runtime manifest declares a different gain file")
    if digest(ROOT / "viewer/generated/ardupilot/arduplane.wasm") != current["published"]["firmware_sha256"]:
        errors.append("Firmware binary differs from selected release")
    for package, prohibited in [("simulation/crates/sim-python/python/microagu_sim", ("microagu_control", "research")), ("control/src/microagu_control", ("research", "viewer"))]:
        for path in (ROOT / package).rglob("*.py"):
            tree = ast.parse(path.read_text(), filename=str(path))
            for node in ast.walk(tree):
                names = [n.name for n in node.names] if isinstance(node, ast.Import) else [node.module or ""] if isinstance(node, ast.ImportFrom) and node.level == 0 else []
                if any(name.split(".")[0] in prohibited for name in names):
                    errors.append(f"Forbidden package dependency: {path.relative_to(ROOT)}:{node.lineno}")
    catalog = json.loads((ROOT / "research/catalog.json").read_text())
    for study in catalog["studies"]:
        if not (ROOT / study["path"]).is_dir():
            errors.append(f"Missing catalog study: {study['path']}")
    for manifest_path in (ROOT / "research/archive").glob("*/evidence-manifest.json"):
        evidence = json.loads(manifest_path.read_text())
        archive = manifest_path.parent / evidence["archive"]
        if not archive.is_file() or digest(archive) != evidence["sha256"]:
            errors.append(f"Historical evidence checksum mismatch: {manifest_path.parent.name}")
    if errors:
        raise SystemExit("\n".join(errors))
    print("Package boundaries, study catalog and selected release integrity passed")

if __name__ == "__main__":
    check()
