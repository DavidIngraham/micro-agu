from pathlib import Path
import hashlib,json,tarfile
import numpy as np
from assess import assess
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
sha=lambda b:hashlib.sha256(b).hexdigest()
manifest=json.loads((HERE/"evidence-manifest.json").read_text())
assert sha((HERE/"evidence.tar.gz").read_bytes())==manifest["sha256"]
summary={r["id"]:r for r in json.loads((HERE/"summary.json").read_text())}
count=0
with tarfile.open(HERE/"evidence.tar.gz") as archive:
    for name,checksum in manifest["files"].items():
        data=archive.extractfile(name).read()
        assert sha(data)==checksum
        if name.startswith("preliminary"):continue
        count+=1;r=json.loads(data)
        assert assess(r)==summary[r["spec"]["id"]]
        assert np.isfinite([s["state"] for s in r["samples"]]).all()
        if r["contact"]:assert r["contact"]["incoming"]["state"][2]<-.12
        if r["spec"]["variant"]!="stock":
            assert all(x["command"][0]<=.001 for x in r["samples"]
                       if x["mode"]==0 and x["time"]>r["handoff"]+1),r["spec"]
            assert summary[r["spec"]["id"]]["approach_active_fraction"]>.95
            assert all(0<=s["guidance"]["common"]<=58 for s in r["samples"] if s["guidance"])
assert count==13,count
for name in ["pilot","campaign"]:
    p=json.loads((HERE/f"{name}-protocol.json").read_text())
    for path,checksum in p["artifacts"].items():assert sha((ROOT/path).read_bytes())==checksum
    for path,checksum in p["source"].items():assert sha((HERE/path).read_bytes())==checksum
print("PASS: 13 final traces, raw checksums, recomputed metrics, unchanged artifacts/source, approach ownership and flare limits")
