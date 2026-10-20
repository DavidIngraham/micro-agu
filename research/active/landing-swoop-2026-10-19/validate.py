"""Validate retained evidence, experiment boundaries, and installed-kernel parity."""
from pathlib import Path
import hashlib
import json
import tarfile
import numpy as np
from microagu_sim import _native
from assess import assess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
manifest = json.loads((HERE/"evidence-manifest.json").read_text())
sha = lambda b: hashlib.sha256(b).hexdigest()
assert sha((HERE/manifest["archive"]).read_bytes()) == manifest["sha256"]
with tarfile.open(HERE/manifest["archive"]) as archive:
    records = {}
    for name, checksum in manifest["files"].items():
        data = archive.extractfile(name).read()
        assert sha(data) == checksum
        records[name] = json.loads(data)
native = [r for name, r in records.items() if name.startswith("native-runs/")]
ap = [r for name, r in records.items() if name.startswith("ap-runs/")]
assert len(native) == 72 and len(ap) == 18, (len(native), len(ap))
summaries = {r["id"]: r for r in json.loads((HERE/"ap-summary.json").read_text())}
for r in ap:
    assert assess(r) == summaries[r["spec"]["id"]]
    assert r["complete"] and not r["failure"], r["spec"]
    assert len(r["samples"]) > 100
    if r["spec"]["kind"] == "landing":
        env = r["environment"]
        h = env["heading"]
        rotation = np.array([[np.cos(h), -np.sin(h)], [np.sin(h), np.cos(h)]])
        np.testing.assert_allclose(env["windNED"][:2], rotation@[env["along"], env["across"]])
        assert r["touchdown"]["incoming"]["state"][2] < -.12
        # Verify the last five seconds really belong to the classical controller.
        assert summaries[r["spec"]["id"]]["late_active_fraction"] == 0
    elif r["spec"]["scheme"] == "lqi":
        assert summaries[r["spec"]["id"]]["lqi_active_fraction"] > .95
    assert np.isfinite(np.array([s["state"] for s in r["samples"]])).all()
for r in native:
    assert np.isfinite(r["samples"]).all()
    assert not r["metrics"]["preapproach_contact"]
    if r["spec"]["kind"] == "landing":
        assert r["metrics"]["contact"] is not None
        assert -r["metrics"]["contact"]["position"][2] > .1
    else:
        assert r["metrics"]["contact"] is None
for name, checksum in json.loads((HERE/"ap-protocol.json").read_text())["artifacts"].items():
    assert sha((ROOT/name).read_bytes()) == checksum, name
c = json.loads((ROOT/"aircraft/microagu/configurations/browser-physics.json").read_text())
k = _native.Kernel(c["parameters"], c["mass"])
native_step = k.surface_step(c["state"], [0, .036, .04], [1, .5, 0], c["voltage"],
                            c["rpm"], .00125, [2, 0, 1], [.1, 1.2, 4.5, 3, .8, .5])
np.testing.assert_allclose(native_step, json.loads((HERE/"wasm-parity.json").read_text()),
                           rtol=1e-10, atol=1e-10)
print("PASS: 90 traces, metrics, ownership, wind direction, first-contact bounds, artifact hashes and native/WASM parity")
