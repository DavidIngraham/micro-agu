
"""Resolution checks and grouping without selecting a tune."""
import json,numpy as np
from scipy.spatial.transform import Rotation
from microagu_control.sysid.metrics import metrics
from run import OUT,execute

rows=[]
for name in ["hood_river-438","trout_lake-2100"]:
 for label in ["prop-0.6","drag-z--1","risers-500-2","risers-2000-2"]:
    req=json.loads((OUT/"inputs"/f"{name}-{label}.json").read_text())
    old=np.load(OUT/"traces"/f"{name}-{label}.npz")
    new=execute(req|{"substeps":64})
    rates=np.rad2deg(Rotation.from_quat(new[:,3:7]).inv().apply(new[:,10:13]))
    m=metrics(old["observed"],rates)
    rows.append(dict(name=name,variant=label,
        max_state_difference_first_2s=float(abs(new[:51,:26]-old["rows"][:51,:26]).max()),
        max_state_difference_full=float(abs(new[:,:26]-old["rows"][:,:26]).max()),
        yaw_band_rms_64=m["model_band_rms"][2],
        yaw_band_rms_32=metrics(old["observed"],old["rates"])["model_band_rms"][2]))
(OUT/"resolution.json").write_text(json.dumps(rows,indent=2)+chr(10))

data=json.loads((OUT/"results.json").read_text())
summary={}
for flight in ["hood_river","trout_lake"]:
 baseline={r["name"]:r for r in data if r["flight"]==flight and r["variant"]=="baseline"}
 summary[flight]={}
 for label in json.loads((OUT/"protocol.json").read_text())["variants"]:
    selected=[r for r in data if r["flight"]==flight and r["variant"]==label and "error" not in r]
    summary[flight][label]=dict(
        mean_rate_rmse_ratio=np.mean([np.array(r["uncentered_rate_rmse"])/baseline[r["name"]]["uncentered_rate_rmse"] for r in selected],axis=0).tolist(),
        mean_model_band_rms=np.mean([r["metrics"]["model_band_rms"] for r in selected],axis=0).tolist(),
        mean_observed_band_rms=np.mean([r["metrics"]["observed_band_rms"] for r in selected],axis=0).tolist(),
        mean_abs_phase_error_deg=float(np.mean([abs(r["metrics"]["phase_error_deg"]) for r in selected])),
        cases=len(selected))
(OUT/"summary.json").write_text(json.dumps(summary,indent=2)+chr(10))
