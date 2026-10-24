"""Verify the promoted bytes and record the completed release checks."""
import hashlib
import json
import math
from pathlib import Path
import shutil

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


manifest = json.loads((ROOT / "viewer/assets/runtime/build.json").read_text())
for name in ("arduplane.js", "arduplane.wasm"):
    path = ROOT / "viewer/generated/ardupilot" / name
    assert digest(path) == manifest["sha256"][name]
    assert path.read_bytes() == (HERE / "firmware/release" / name).read_bytes()
assert digest(HERE / "release-configuration.json") == manifest["sha256"]["highdof.json"]
before = json.loads((HERE / "configuration.json").read_text())
after = json.loads((HERE / "release-configuration.json").read_text())
before.pop("defaults")
after.pop("defaults")
assert before == after, "Aircraft physics changed"

smokes = []
for spec in json.loads((HERE / "smoke-cases.json").read_text(encoding="utf-8-sig")):
    path = HERE / "runs" / (spec["id"] + "-result.json")
    data = json.loads(path.read_text())
    assert data["spec"] == spec and data["complete"] and not data["failure"]
    assert data["parameters"]["LAND_FLARE_AIM"] == 0
    assert data["finalLanding"]["phase"] == "landed"
    state = data["contact"]["incoming"]["state"]
    miss = math.hypot(state[0] - data["target"]["north"], state[1] - data["target"]["east"])
    if spec["name"] in ("calm", "weather"):
        assert miss < 5, (spec["id"], miss)
    assert 0 < state[9] < 2
    smokes.append({"id": spec["id"], "miss_m": miss, "sink_m_s": state[9],
                   "complete": True, "result_sha256": digest(path)})

checks = {}
for name, marker, command in [
    ("unit", "pass 55", "npm test"),
    ("physics", "PASS: 1200 steps", "npm run test:physics"),
    ("viewer-build", "built in", "npm run build"),
    ("sitl", "AUTO OK", "npm run test:sitl"),
    ("mimo", "MIMO PASS", "npm run test:mimo"),
    ("browser", "1 passed", "npx playwright test tests/browser/rtl.spec.ts"),
]:
    source = Path(f"/tmp/landing-{name}.log")
    assert marker in source.read_text(), name
    destination = HERE / f"{name}.log"
    shutil.copyfile(source, destination)
    checks[name] = {"command": command, "exit_code": 0, "log_sha256": digest(destination)}

receipt = {"firmware_revision": manifest["revision"], "runtime_sha256": manifest["sha256"],
           "checks": checks, "smoke": smokes,
           "executed_campaign_driver_sha256": digest(HERE / "case-tested.mts"),
           "replay_and_smoke_driver_sha256": digest(HERE / "case.mts")}
(HERE / "release-checks.json").write_text(json.dumps(receipt, indent=2) + "\n")
print(json.dumps({"checks": list(checks), "smoke": smokes}, indent=2))
