"""Verify the published simulator assets against the exact local release."""
import hashlib
import json
from pathlib import Path
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[2]
BASE = "https://davidingraham.github.io/assets/microagu-viewer/"
expected = json.loads((ROOT / "viewer/dist/simulator-manifest.json").read_text()) if (ROOT / "viewer/dist/simulator-manifest.json").exists() else json.loads((ROOT / "viewer/dist/applet-manifest.json").read_text())["applets"]["simulator"]
actual = json.loads(urlopen(BASE + "simulator-manifest.json?release=dad276c", timeout=60).read())
assert actual == expected, "Live applet manifest differs from release"
checks = {}
for name, sha in actual["files"].items():
    payload = urlopen(BASE + name + "?v=" + sha, timeout=60).read()
    checks[name] = hashlib.sha256(payload).hexdigest() == sha
assert all(checks.values()), checks
Path(__file__).with_name("live-hashes.json").write_text(json.dumps({"passed": True, "url": BASE, "files": checks}, indent=2) + "\n")
print(f"Verified {len(checks)} live assets")
