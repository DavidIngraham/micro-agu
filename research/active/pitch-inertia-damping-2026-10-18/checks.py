import json
import numpy as np
from scipy.spatial.transform import Rotation
from run import OUT,apply,variants
from microagu_control.sysid.data import load_session
from microagu_control.sysid.model import configured_simulator,initialize,replay
from microagu_control.sysid.metrics import metrics
_,segments,_=load_session(OUT/"session.json")
rows=[]
for s in segments:
    for label in ["baseline","J4.0-C0.25-D0.1","coupled-offset0.0-D0.1"]:
        ref=np.load(OUT/"traces"/f"{s.name}-{label}.npz")
        for steps,alpha in [(64,.35),(32,.25),(32,.45)]:
            sim=apply(configured_simulator(s,steps),variants()[label])
            y=initialize(sim,s,alpha,s.wind)
            states,rpm=sim.replay(y,s.commands,s.voltage,s.rpm,s.wind)
            rates=np.rad2deg(Rotation.from_quat(states[:,3:7]).inv().apply(states[:,10:13]))
            m=metrics(np.rad2deg(s.observation[:,3:6]),rates)
            rows.append(dict(name=s.name,variant=label,substeps=steps,alpha=alpha,max_rate_difference=float(abs(rates-ref["rates"]).max()),metrics=m))
        if label=="baseline":
            rates,y,rpm=replay(s)
            assert np.allclose(y,ref["state"],rtol=0,atol=1e-12)
# Identity candidate must be identical to baseline.
for s in segments:
    a=np.load(OUT/"traces"/f"{s.name}-baseline.npz")["state"]
    b=np.load(OUT/"traces"/f"{s.name}-J1.0-C1.0-D1.0.npz")["state"]
    assert np.array_equal(a,b)
(OUT/"checks.json").write_text(json.dumps(rows,indent=2))
print("baseline parity and identity passed; sensitivity runs",len(rows),flush=True)
