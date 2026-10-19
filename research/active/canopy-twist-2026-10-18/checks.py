import json,numpy as np
from scipy.spatial.transform import Rotation
from microagu_control.sysid.data import load_session
from microagu_control.sysid.metrics import metrics
from run import OUT,variants,prepare,execute
_,segments,_=load_session(OUT/"session.json")
labels=json.loads((OUT/"selected-diagnostics.json").read_text())["labels"]
rows=[]
for s in segments:
    for label in labels:
        ref=np.load(OUT/"traces"/f"{s.name}-{label}.npz")
        for steps,twist in [(64,0.),(32,np.deg2rad(5)),(32,np.deg2rad(-5))]:
            if label=="baseline" and twist!=0:continue
            try:
                y=execute(prepare(s,variants()[label],steps,twist))
                rates=np.rad2deg(Rotation.from_quat(y[:,3:7]).inv().apply(y[:,10:13]))
                m=metrics(np.rad2deg(s.observation[:,3:6]),rates)
                rows.append(dict(name=s.name,variant=label,substeps=steps,initial_twist=float(twist),max_rate_difference=float(abs(rates-ref["rates"]).max()),metrics=m))
            except Exception as e:rows.append(dict(name=s.name,variant=label,substeps=steps,initial_twist=float(twist),error=str(e)))
(OUT/"checks.json").write_text(json.dumps(rows,indent=2))
print("checks",len(rows),"errors",sum("error" in r for r in rows),flush=True)
