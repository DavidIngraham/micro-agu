import json
from pathlib import Path
import numpy as np
from microagu_control.sysid.data import load_session
from microagu_control.sysid.model import replay
OUT=Path(__file__).resolve().parent
_,segments,_=load_session(OUT.parent/"missing-physics-2026-10-18/session.json")
rows=[]
for s in segments:
    _,_,pred=replay(s)
    day="2026-01-03" if s.flight=="hood_river" else "2026-02-15"
    base=Path("/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/longitudinal-2026-10-09")
    folder="2026-01-03 15-41-42" if s.flight=="hood_river" else "2026-02-15 15-38-50"
    esc=json.loads((base/folder/"ESC.json").read_text())
    t=np.array([x["TimeUS"]/1e6 for x in esc])
    # Evaluate model at native ESC timestamps, no interpolation of measured RPM.
    mask=(t>=s.time[0]+2)&(t<=s.time[-1])
    pp=np.interp(t[mask],s.time,pred)
    row=dict(name=s.name,flight=s.flight,voltage=s.voltage)
    for key in ["RPM","RawRPM"]:
        obs=np.array([x[key] for x in esc])[mask];valid=obs>1500
        e=pp[valid]-obs[valid]
        row[key]=dict(n=int(valid.sum()),rmse=float(np.sqrt(np.mean(e**2))),bias=float(e.mean()),median_abs_pct=float(np.median(abs(e)/obs[valid])*100))
    rows.append(row)
(OUT/"replay-check.json").write_text(json.dumps(rows,indent=2))
for f in ["hood_river","trout_lake"]:
 a=[r for r in rows if r["flight"]==f]
 print(f,"mean window raw RMSE",np.mean([r["RawRPM"]["rmse"] for r in a]),"mean window median pct",np.mean([r["RawRPM"]["median_abs_pct"] for r in a]))
