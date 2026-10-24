"""Record completed checks and exact inputs for this staged simulator release."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


markers = {
    "firmware-build": "'plane' finished successfully",
    "unit": "pass 56",
    "viewer-build": "built in",
    "integration": "AUTOLAND PASS",
    "sitl": "9-DOF TRUTH OK",
    "mimo": "MIMO PASS",
    "browser": "2 passed",
}
logs = {}
for name, marker in markers.items():
    source = Path(f"/tmp/autoland-{name}.log")
    assert marker in source.read_text(), f"Missing completion: {source}"
    shutil.copyfile(source, HERE / source.name)
    logs[name] = {"path": source.name, "sha256": digest(source)}
previous = json.loads((HERE / "previous-build.json").read_text())
build = json.loads((ROOT / "viewer/assets/runtime/build.json").read_text())
result = json.loads((HERE / "integration.json").read_text())
assert result["firmware_sha256"] == build["sha256"]["arduplane.wasm"]
assert result["configuration_sha256"] == build["sha256"]["highdof.json"]
assert all(result["checks"].values())
for key, paths in build.items():
    if key.endswith("_source_sha256"):
        for name, expected in paths.items():
            assert digest(ROOT.parent / "ardupilot-lqi-pipeline" / name) == expected, name
for name in ("sim_wasm.js", "sim_wasm_bg.wasm", "lqi-gains.bin"):
    assert previous["sha256"][name] == build["sha256"][name], name
subprocess.run(["git", "apply", "--reverse", "--check", str(ROOT / "viewer/assets/runtime/lqi-file-pipeline.patch")],
               cwd=ROOT.parent / "ardupilot-lqi-pipeline", check=True)
sources = (
    "viewer/tests/integration/autoland.ts", "viewer/tests/browser/autoland.spec.ts",
    "viewer/tests/browser/rtl.spec.ts", "viewer/tests/integration/sitl.ts",
    "viewer/tests/integration/mimo.ts", "viewer/packages/sim-browser/src/session/runtime.ts",
    "viewer/packages/sim-browser/src/session/landing-target.ts",
    "viewer/apps/simulator/main.ts", "viewer/apps/simulator/index.html",
)
verification = {
    "checks": {name: True for name in markers} | {
        "physics_and_gain_hashes_unchanged": True, "reconstructable_firmware_patch": True},
    "logs": logs,
    "source_sha256": {name: digest(ROOT / name) for name in sources},
    "firmware_revision": build["revision"],
    "firmware_sha256": result["firmware_sha256"],
    "configuration_sha256": result["configuration_sha256"],
    "qualification": "Current workspace SITL/MIMO tests include pre-existing local test edits; fingerprints are retained.",
}
(HERE / "verification.json").write_text(json.dumps(verification, indent=2) + "\n")
print(json.dumps(verification["checks"]))
