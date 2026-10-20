from pathlib import Path
import hashlib,json,tarfile
import numpy as np
from assess import assess
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2]
sha=lambda b:hashlib.sha256(b).hexdigest()
m=json.loads((HERE/"evidence-manifest.json").read_text())
assert sha((HERE/"evidence.tar.gz").read_bytes())==m["sha256"]
assert len(m["files"])==6
with tarfile.open(HERE/"evidence.tar.gz") as a:
    for name,digest in m.get("diagnostics",{}).items():
        data=a.extractfile(name).read();assert sha(data)==digest
        d=json.loads(data);assert d["contact"] and "divergent" in d["failure"]
    for name,digest in m["files"].items():
        data=a.extractfile(name).read();assert sha(data)==digest
        r=json.loads(data)
        assert len(r["lidarPackets"])>100 and len(r["haglPackets"])>100
        assert np.isfinite([x["state"] for x in r["samples"]]).all()
        if r["spec"]["id"]=="failed-sensor":
            assert r["failure"].startswith("EKF terrain height unavailable")
            assert not r["contact"]
            assert all(p["values"]["current_distance"]>p["values"]["max_distance"] for p in r["lidarPackets"])
        elif not r["complete"]:
            assert r["spec"]["id"]=="dropout-feedback" and r["failure"].startswith("EKF terrain height unavailable")
        else:
            assert r["contact"]["incoming"]["state"][2]<-.12
            assert r["finalLanding"]["phase"]=="landed"
            assert r["samples"][-1]["time"]>=r["contact"]["current"]["time"]
            assert assess(r)["approach_active_fraction"]>.95
            assert 0<assess(r)["sink"]<1.6
for name,digest in json.loads((HERE/"protocol.json").read_text())["hashes"].items():
    assert sha((ROOT/name).read_bytes())==digest,name
print("PASS: six AP/EKF traces, checksums, finite dynamics, LQI approach, physical contact before endpoint, and failed-sensor rejection")
