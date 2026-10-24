"""Prepare, push and verify the explicitly authorized experimental sim release."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import urllib.request

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
SITE = ROOT.parent / "microagu-pages-publish"
BASE_URL = "https://davidingraham.github.io/assets/microagu-viewer/"


def git(*args):
    return subprocess.check_output(["git", *args], cwd=SITE, text=True).strip()


def write(path, value):
    path.write_text(json.dumps(value, indent=2) + "\n")


def sha(data):
    return hashlib.sha256(data).hexdigest()


action = sys.argv[1]
receipt_path = HERE / "publication.json"
if action == "prepare":
    assert json.loads((HERE / "decision.json").read_text())["deploy"]
    assert (HERE / "release-checks.json").is_file()
    assert not git("diff", "--cached", "--name-only"), "Unrelated staged site changes"
    assert git("rev-parse", "HEAD") == git("rev-parse", "origin/master")
    base = git("rev-parse", "HEAD")
    manifest_path = SITE / "assets/microagu-viewer/simulator-manifest.json"
    manifest = json.loads(manifest_path.read_text())
    paths = ["assets/microagu-viewer/" + p for p in manifest["files"]]
    paths.append("assets/microagu-viewer/simulator-manifest.json")
    for name, expected in manifest["files"].items():
        assert sha((SITE / "assets/microagu-viewer" / name).read_bytes()) == expected, name
    subprocess.run(["git", "add", "--", *paths], cwd=SITE, check=True)
    assert set(git("diff", "--cached", "--name-only").splitlines()) <= set(paths)
    subprocess.run(["git", "commit", "-m", "Publish calibrated MicroAGU native landing",
                    "-m", "Use the tested 3 m/s wind-update gate and zero fixed-wing flare aim adjustment. Matched nine-condition spot miss improves from 9.61 to 2.79 m median; retain documented headwind, steep-approach and contact-speed limitations. AI-assisted implementation and validation."], cwd=SITE, check=True)
    receipt = {"status": "prepared", "site_base": base, "site_commit": git("rev-parse", "HEAD"),
               "url": BASE_URL + "simulator.html", "files": manifest["files"]}
    write(receipt_path, receipt)
    current_path = ROOT / "aircraft/microagu/current.json"
    current = json.loads(current_path.read_text())
    current["published"].update(site_commit=receipt["site_commit"], status="deployment prepared; awaiting public verification")
    write(current_path, current)
    print(receipt["site_commit"])
elif action == "push":
    receipt = json.loads(receipt_path.read_text())
    assert git("rev-parse", "HEAD") == receipt["site_commit"]
    subprocess.run(["git", "push", "origin", "HEAD:refs/heads/master"], cwd=SITE, check=True)
    receipt["status"] = "pushed; awaiting Pages verification"
    write(receipt_path, receipt)
elif action == "verify":
    receipt = json.loads(receipt_path.read_text())
    expected = json.loads((ROOT / "viewer/assets/runtime/build.json").read_text())
    verified = {}
    for name in ("wasm/build.json", "wasm/highdof.json", "wasm/arduplane.wasm", "wasm/landing-validation.json", "simulator.html"):
        request = urllib.request.Request(BASE_URL + name + "?release=" + receipt["site_commit"], headers={"Cache-Control": "no-cache"})
        with urllib.request.urlopen(request, timeout=45) as response:
            data = response.read()
        assert sha(data) == receipt["files"][name], f"Public bytes not updated: {name}"
        verified[name] = sha(data)
        if name == "wasm/build.json":
            assert json.loads(data) == expected
    receipt.update(status="published and public bytes verified", live_verified_sha256=verified)
    write(receipt_path, receipt)
    current_path = ROOT / "aircraft/microagu/current.json"
    current = json.loads(current_path.read_text())
    current["published"]["status"] = "published; firmware, configuration and landing evidence verified over HTTPS"
    write(current_path, current)
    print(json.dumps({"site_commit": receipt["site_commit"], "verified": list(verified)}, indent=2))
else:
    raise SystemExit("Use prepare, push or verify")
