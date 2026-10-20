import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
HERE = Path(__file__).resolve().parent

def assess(r):
    rows=[x for x in r["samples"] if not r["contact"] or x["time"]<=r["contact"]["current"]["time"]]
    result=dict(id=r["spec"]["id"],complete=r["complete"],failure=r["failure"],final_phase=(r.get("finalLanding") or {}).get("phase"))
    if not rows:
        return result
    states=np.array([x["state"] for x in rows])
    angles=Rotation.from_quat(states[:,3:7]).as_euler("xyz",degrees=True)
    approach=([x for x in rows if x["mode"]==7]
              if r["spec"]["variant"]!="stock" else [])
    manual=[x for x in rows if x["mode"]==0]
    flare=[x for x in rows if (x["guidance"] or {}).get("flare")]
    result.update(peak_roll=float(max(abs(angles[:,0]))),peak_pitch=float(max(abs(angles[:,1]))),
        approach_active_fraction=float(np.mean([x["active"] for x in approach])) if approach else None,
        handoff_height=float(-manual[0]["state"][2]) if manual else None,
        flare_height=float(-flare[0]["state"][2]) if flare else None,
        duration=rows[-1]["time"],
        peak_brake_mm=float(max(max(x["brakes"]) for x in rows)))
    if r["contact"]:
        y=np.array(r["contact"]["incoming"]["state"])
        t=r["target"];dn,de=np.cos(t["heading"]),np.sin(t["heading"])
        delta=y[:2]-[t["north"],t["east"]]
        result.update(spot_error=float(np.linalg.norm(delta)),along_error=float(delta@[dn,de]),
            cross_error=float(delta@[-de,dn]),sink=float(y[9]),ground_speed=float(np.linalg.norm(y[7:9])),
            pitch=float(Rotation.from_quat(y[3:7]).as_euler("xyz",degrees=True)[1]),
            incoming_height=float(-y[2]))
    return result

if __name__=="__main__":
    rows=[assess(json.loads(p.read_text())) for p in sorted((HERE/"runs").glob("*.json"))]
    (HERE/"summary.json").write_text(json.dumps(rows,indent=2)+"\n")
    for row in rows:print(json.dumps(row))
