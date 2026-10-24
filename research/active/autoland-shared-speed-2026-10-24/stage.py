"""Stage the shared-speed/AUTOLAND runtime without modifying historical evidence."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
FIRMWARE = ROOT.parent / "ardupilot-lqi-pipeline"
AIRCRAFT = ROOT / "aircraft/microagu"
RUNTIME = ROOT / "viewer/assets/runtime"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path, data):
    path.write_text(json.dumps(data, indent=2) + "\n")


revision = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=FIRMWARE, text=True).strip()
assert not subprocess.check_output(["git", "status", "--porcelain"], cwd=FIRMWARE, text=True).strip()
manifest = json.loads((RUNTIME / "build.json").read_text())
if not (HERE / "previous-build.json").exists():
    shutil.copyfile(RUNTIME / "build.json", HERE / "previous-build.json")
for name in ("arduplane.js", "arduplane.wasm"):
    source = FIRMWARE / "build/wasm/bin" / name
    destination = ROOT / "viewer/generated/ardupilot" / name
    shutil.copyfile(source, destination)
    manifest["sha256"][name] = digest(destination)
shutil.copyfile(AIRCRAFT / "configurations/browser-physics.json", RUNTIME / "highdof.json")
manifest["sha256"]["highdof.json"] = digest(RUNTIME / "highdof.json")
manifest.update(revision=revision, source=f"https://github.com/DavidIngraham/ardupilot/tree/{revision}", working_tree_dirty=False)
base = manifest["controller_source_patch"]["base_revision"]
(RUNTIME / "lqi-file-pipeline.patch").write_bytes(subprocess.check_output(
    ["git", "diff", "--binary", base, revision], cwd=FIRMWARE))
manifest["controller_source_patch"].update(
    sha256=digest(RUNTIME / "lqi-file-pipeline.patch"),
    description="File-loaded LQI, native paraglider AUTO/AUTOLAND and shared AHRS_MIN_SPEED gate.")
manifest["wind_estimator_source_sha256"] = {
    name: digest(FIRMWARE / name) for name in (
        "libraries/AP_AHRS/AP_AHRS.cpp", "libraries/AP_AHRS/AP_AHRS.h",
        "libraries/AP_DAL/AP_DAL.cpp", "libraries/AP_DAL/AP_DAL.h", "libraries/AP_DAL/LogStructure.h",
        "libraries/AP_NavEKF2/AP_NavEKF2_AirDataFusion.cpp",
        "libraries/AP_NavEKF3/AP_NavEKF3_AirDataFusion.cpp")}
manifest["autoland_source_sha256"] = {
    name: digest(FIRMWARE / name) for name in (
        "ArduPlane/mode_autoland.cpp", "ArduPlane/mode_takeoff.cpp",
        "ArduPlane/mode_cruise.cpp", "ArduPlane/commands_logic.cpp", "ArduPlane/defines.h")}
for key, paths in manifest.items():
    if key.endswith("_source_sha256"):
        for name in paths:
            paths[name] = digest(FIRMWARE / name)
manifest["autoland"] = {
    "mode": 26, "target": "AHRS home", "AHRS_MIN_SPEED": 3,
    "firmware_default_min_speed_m_s": 5,
    "AUTOLAND_WP_DIST": 200, "AUTOLAND_WP_ALT": 30,
    "validation": "autoland-validation.json"}
evidence = {
    "scope": "Experimental browser simulation only; no physical flight acceptance",
    "study": "research/active/autoland-shared-speed-2026-10-24/RESULTS.md",
    "firmware_revision": revision,
    "firmware_sha256": manifest["sha256"]["arduplane.wasm"],
    "configuration_sha256": manifest["sha256"]["highdof.json"],
    "settings": manifest["autoland"],
    "previous_landing_accuracy_study": "research/active/landing-release-2026-10-24/RESULTS.md",
    "status": "staged; awaiting tests",
}
if (HERE / "integration.json").exists():
    evidence["integration"] = json.loads((HERE / "integration.json").read_text())
    assert evidence["integration"]["firmware_sha256"] == evidence["firmware_sha256"]
    assert evidence["integration"]["configuration_sha256"] == evidence["configuration_sha256"]
    assert all(evidence["integration"]["checks"].values())
    evidence["status"] = "native integration tested; see report for browser and regression checks"
if (HERE / "verification.json").exists():
    evidence["verification"] = json.loads((HERE / "verification.json").read_text())
write(AIRCRAFT / "validation/autoland-acceptance.json", evidence)
shutil.copyfile(AIRCRAFT / "validation/autoland-acceptance.json", RUNTIME / "autoland-validation.json")
write(RUNTIME / "build.json", manifest)
current = json.loads((AIRCRAFT / "current.json").read_text())
current["controller"].update(firmware_revision=revision, firmware_source=manifest["source"])
current["published"].update(site_commit=None, firmware_sha256=manifest["sha256"]["arduplane.wasm"], status="staged; shared speed and native AUTOLAND")
current["autoland_validation"] = "validation/autoland-acceptance.json"
current["staged_runtime"]["validation/autoland-acceptance.json"] = "viewer/assets/runtime/autoland-validation.json"
for name in ("configurations/browser-physics.json", "validation/autoland-acceptance.json"):
    current["sha256"][name] = digest(AIRCRAFT / name)
write(AIRCRAFT / "current.json", current)
print(json.dumps({"revision": revision, "firmware_sha256": manifest["sha256"]["arduplane.wasm"]}))
