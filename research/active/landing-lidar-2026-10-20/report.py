from pathlib import Path
import json,numpy as np
from assess import assess
HERE=Path(__file__).resolve().parent
metrics=[]
lines=["# Simulated lidar landing results","",
       "| Case | Result | Spot error (m) | Incoming sink (m/s) | Ground speed (m/s) | AP range packets |",
       "|---|---|---:|---:|---:|---:|"]
for p in sorted((HERE/"runs").glob("*.json")):
    r=json.loads(p.read_text());m=assess(r);m["range_packets"]=len(r["lidarPackets"])
    errors=[]
    for s in r["samples"]:
        l=s["lidar"];x=l["range"];t=s["telemetry"]
        if x is not None and 0.2<-s["state"][2]<10:
            errors.append(x*np.cos(t["roll"])*np.cos(t["pitch"])+s["state"][2])
    m["median_height_error_m"]=float(np.median(errors)) if errors else None
    metrics.append(m)
    cells=[f'{m[k]:.2f}' if k in m else "—" for k in ["spot_error","sink","ground_speed"]]
    lines.append("| "+m["id"]+" | "+("Finished" if m["complete"] else m["failure"])+" | "+
                 " | ".join(cells)+f' | {m["range_packets"]} |')
(HERE/"summary.json").write_text(json.dumps(metrics,indent=2)+"\n")
(HERE/"results.md").write_text("\n".join(lines)+"\n")
print("\n".join(lines))
