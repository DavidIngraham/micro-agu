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
    for name,digest in m["files"].items():
        data=a.extractfile(name).read();assert sha(data)==digest
        r=json.loads(data)
        assert len(r["lidarPackets"])>100
        assert np.isfinite([x["state"] for x in r["samples"]]).all()
        if r["spec"]["id"]=="failed-sensor":
            assert r["failure"]=="Lidar unavailable near ground"
            assert not r["contact"]
            assert all(p["values"]["current_distance"]>p["values"]["max_distance"] for p in r["lidarPackets"])
        elif not r["complete"]:
            assert r["spec"]["id"]=="dropout-feedback" and r["failure"]=="Lidar unavailable near ground"
        else:
            assert r["contact"]["incoming"]["state"][2]<-.12
            assert assess(r)["approach_active_fraction"]>.95
for name,digest in json.loads((HERE/"protocol.json").read_text())["hashes"].items():
    assert sha((ROOT/name).read_bytes())==digest,name
print("PASS: six AP telemetry traces, checksums, source/artifact hashes, finite dynamics and failed-sensor rejection")
