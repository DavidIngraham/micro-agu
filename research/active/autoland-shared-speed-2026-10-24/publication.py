"""Publish and verify the tested shared-speed and AUTOLAND web release."""
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
current_path = ROOT / "aircraft/microagu/current.json"
current = json.loads(current_path.read_text())
if action == "prepare":
    assert all(json.loads((HERE / "verification.json").read_text())["checks"].values())
    assert not git("diff", "--cached", "--name-only"), "Unrelated staged site changes"
    assert git("rev-parse", "HEAD") == git("rev-parse", "origin/master")
    base = git("rev-parse", "HEAD")
    published = SITE / "assets/microagu-viewer"
    assert (published / "wasm/build.json").read_bytes() == (ROOT / "viewer/assets/runtime/build.json").read_bytes()
    assert (published / "wasm/autoland-validation.json").read_bytes() == (ROOT / "viewer/assets/runtime/autoland-validation.json").read_bytes()
    manifest = json.loads((published / "simulator-manifest.json").read_text())
    paths = ["assets/microagu-viewer/" + p for p in manifest["files"]]
    paths.append("assets/microagu-viewer/simulator-manifest.json")
    for name, expected in manifest["files"].items():
        assert sha((published / name).read_bytes()) == expected, name
    subprocess.run(["git", "add", "--", *paths], cwd=SITE, check=True)
    assert set(git("diff", "--cached", "--name-only").splitlines()) <= set(paths)
    subprocess.run(["git", "commit", "-m", "Expose native MicroAGU AUTOLAND with shared speed gate",
                    "-m", "Publish tested AHRS_MIN_SPEED=3 and native home approach alongside the retained RTL mission landing. Native and both browser landing paths passed. AI-assisted."], cwd=SITE, check=True)
    receipt = {"status": "prepared", "site_base": base, "site_commit": git("rev-parse", "HEAD"),
               "url": BASE_URL + "simulator.html", "files": manifest["files"]}
    write(receipt_path, receipt)
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
    verified = {}
    for name in ("wasm/build.json", "wasm/highdof.json", "wasm/arduplane.wasm", "wasm/autoland-validation.json", "simulator.html"):
        request = urllib.request.Request(BASE_URL + name + "?release=" + receipt["site_commit"], headers={"Cache-Control": "no-cache"})
        with urllib.request.urlopen(request, timeout=45) as response:
            data = response.read()
        assert sha(data) == receipt["files"][name], f"Public bytes not updated: {name}"
        verified[name] = sha(data)
    receipt.update(status="published and public bytes verified", live_verified_sha256=verified)
    write(receipt_path, receipt)
    current["published"]["status"] = "published; shared speed, native AUTOLAND, configuration and evidence verified over HTTPS"
    write(current_path, current)
    print(json.dumps({"site_commit": receipt["site_commit"], "verified": list(verified)}))
else:
    raise SystemExit("Use prepare, push or verify")
