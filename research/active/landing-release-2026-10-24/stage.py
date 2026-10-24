"""Stage the explicitly reviewed landing release; does not publish or push."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
FIRMWARE = ROOT.parent / "ardupilot-lqi-pipeline"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2) + "\n")


decision = json.loads((HERE / "decision.json").read_text())
assert decision["deploy"] is True
summary = json.loads((HERE / "summary.json").read_text())
assert all(summary["checks"][key] for key in (
    "all_candidate_runs_complete", "candidate_modes_auto_only", "median_miss_reduced",
    "worst_miss_reduced", "existing_native_regression_sink_limit"))
revision = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=FIRMWARE, text=True).strip()
assert not subprocess.check_output(["git", "status", "--porcelain", "--untracked-files=no"], cwd=FIRMWARE, text=True).strip()
assert b"if (vel_rel_wind.x > 3.0f)" in (FIRMWARE / "libraries/AP_NavEKF3/AP_NavEKF3_AirDataFusion.cpp").read_bytes()

aircraft = ROOT / "aircraft/microagu"
runtime = ROOT / "viewer/assets/runtime"
generated = ROOT / "viewer/generated/ardupilot"
original = json.loads((HERE / "published-build.json").read_text())
for name, relative in {
    "sim_wasm_bg.wasm": "viewer/generated/physics/sim_wasm_bg.wasm",
    "sim_wasm.js": "viewer/generated/physics/sim_wasm.js",
    "lqi-gains.bin": "aircraft/microagu/controllers/lqi-gains.bin",
}.items():
    assert digest(ROOT / relative) == original["sha256"][name], name

config = json.loads((HERE / "configuration.json").read_text())
assert "LAND_FLARE_AIM" not in config["defaults"]
config["defaults"] = config["defaults"].replace(
    "LAND_FLARE_ALT 3\n",
    "# Native landing: retain approach sink in the path estimate.\nLAND_FLARE_AIM 0\nLAND_FLARE_ALT 3\n",
)
write_json(aircraft / "configurations/browser-physics.json", config)
shutil.copyfile(aircraft / "configurations/browser-physics.json", runtime / "highdof.json")
shutil.copyfile(aircraft / "configurations/browser-physics.json", HERE / "release-configuration.json")
release_firmware = HERE / "firmware/release"
release_firmware.mkdir(exist_ok=True)
for name in ("arduplane.js", "arduplane.wasm"):
    source = FIRMWARE / "build/wasm/bin" / name
    shutil.copyfile(source, generated / name)
    shutil.copyfile(source, release_firmware / name)

acceptance = {
    "scope": "Experimental browser landing calibration; no physical-flight acceptance",
    "study": "research/active/landing-release-2026-10-24/RESULTS.md",
    "firmware_revision": revision,
    "settings": {"LAND_FLARE_AIM": 0, "LAND_FLARE_ALT": 3, "PG_FLARE_HGT": 1.3, "sideslip_update_minimum_forward_tas_m_s": 3},
    "statistics": summary["statistics"], "checks": summary["checks"], "decision": decision,
    "result_sha256": {r["id"]: r["result_sha256"] for r in summary["rows"]},
}
write_json(aircraft / "validation/landing-acceptance.json", acceptance)
shutil.copyfile(aircraft / "validation/landing-acceptance.json", runtime / "landing-validation.json")

manifest = dict(original)
manifest.update(revision=revision, source=f"https://github.com/DavidIngraham/ardupilot/tree/{revision}", working_tree_dirty=False)
manifest["sha256"] = dict(original["sha256"])
for name, path in {"highdof.json": runtime / "highdof.json", "arduplane.js": generated / "arduplane.js", "arduplane.wasm": generated / "arduplane.wasm"}.items():
    manifest["sha256"][name] = digest(path)
patch = subprocess.check_output(["git", "diff", "--binary", "010ada6a4f0b26910fc18ecca176d2e65c5ed9d7", revision], cwd=FIRMWARE)
(runtime / "lqi-file-pipeline.patch").write_bytes(patch)
manifest["controller_source_patch"] = {
    "path": "lqi-file-pipeline.patch", "sha256": digest(runtime / "lqi-file-pipeline.patch"),
    "base_revision": "010ada6a4f0b26910fc18ecca176d2e65c5ed9d7",
    "description": "Published file-loaded LQI, native AUTO landing and 3 m/s sideslip wind-update gate.",
}
manifest["wind_estimator_source_sha256"] = {
    "libraries/AP_NavEKF3/AP_NavEKF3_AirDataFusion.cpp": digest(FIRMWARE / "libraries/AP_NavEKF3/AP_NavEKF3_AirDataFusion.cpp")}
manifest["landing_calibration"] = acceptance["settings"] | {"validation": "landing-validation.json"}
write_json(runtime / "build.json", manifest)

current = json.loads((aircraft / "current.json").read_text())
current["controller"].update(firmware_revision=revision, firmware_source=manifest["source"])
current["published"].update(site_commit=None, firmware_sha256=manifest["sha256"]["arduplane.wasm"], status="staged, awaiting packaged-runtime and browser checks")
current["landing_validation"] = "validation/landing-acceptance.json"
current["staged_runtime"]["validation/landing-acceptance.json"] = "viewer/assets/runtime/landing-validation.json"
current["sha256"]["configurations/browser-physics.json"] = manifest["sha256"]["highdof.json"]
current["sha256"]["validation/landing-acceptance.json"] = digest(aircraft / "validation/landing-acceptance.json")
write_json(aircraft / "current.json", current)

# Preserve the executed driver; make replay use its captured configuration even
# after the production operating defaults have been promoted.
if not (HERE / "case-tested.mts").exists():
    shutil.copyfile(HERE / "case.mts", HERE / "case-tested.mts")
driver = (HERE / "case-tested.mts").read_text(encoding="utf-8-sig").replace(
    "new URL('../../../aircraft/microagu/configurations/browser-physics.json',import.meta.url)",
    "new URL(spec.configuration??'./configuration.json',import.meta.url)",
)
(HERE / "case.mts").write_text(driver)
write_json(HERE / "staged-release.json", {
    "firmware_revision": revision, "sha256": manifest["sha256"],
    "tested_driver_sha256": digest(HERE / "case-tested.mts"),
    "replay_driver_sha256": digest(HERE / "case.mts"),
    "configuration_sha256": digest(HERE / "release-configuration.json"),
})
print(json.dumps({"revision": revision, "firmware_sha256": manifest["sha256"]["arduplane.wasm"], "configuration_sha256": manifest["sha256"]["highdof.json"]}, indent=2))
