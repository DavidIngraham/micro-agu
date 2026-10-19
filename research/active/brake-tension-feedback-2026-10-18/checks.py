
"""Resolution and fitted-Hood-River sensitivity; no coefficient selection."""
import json
import numpy as np
from scipy.spatial.transform import Rotation
from microagu_control.sysid.data import load_session
from microagu_control.sysid.metrics import metrics
from microagu_sim.compat import dynamics
from run import OUT,prepare,execute

_,segments,_=load_session(OUT/"session.json")
results=[]
for name in ["hood_river-438","trout_lake-2100"]:
    seg=next(s for s in segments if s.name==name)
    for k in [0,300]:
        a=execute(prepare(seg)|{"stiffness":k,"substeps":32})
        b=execute(prepare(seg)|{"stiffness":k,"substeps":64})
        results.append(dict(name=name,stiffness=k,max_state_difference=float(abs(a[:,:26]-b[:,:26]).max())))
(OUT/"resolution.json").write_text(json.dumps(results,indent=2)+chr(10))

theta=json.loads((OUT.parent/"hood-river-pipeline-2026-10-18/fit/fit.json").read_text())["selection"]["theta"]
results=[]
for seg in [s for s in segments if s.flight=="hood_river"]:
    req=prepare(seg)
    p=dict(zip(dynamics.KEYS,req["parameters"]))
    for key,factor in zip(["rotor_spin_inertia","joint_k_yaw","joint_c_yaw","joint_c_pitch"],theta[:4]):
        p[key]*=factor
    req["parameters"]=[p[key] for key in dynamics.KEYS]
    commands=np.array(req["commands"]);commands[:,1:]*=theta[4]
    decay=np.exp(-.04/theta[5])
    for i in range(1,len(commands)):commands[i,1:]=decay*commands[i-1,1:]+(1-decay)*commands[i,1:]
    req["commands"]=commands.tolist()
    for label,opts in json.loads((OUT/"protocol.json").read_text())["variants"].items():
        a=execute(req|opts)
        rates=np.rad2deg(Rotation.from_quat(a[:,3:7]).inv().apply(a[:,10:13]))
        obs=np.rad2deg(seg.observation[:,3:6])
        results.append(dict(name=seg.name,variant=label,metrics=metrics(obs,rates)))
(OUT/"fitted-sensitivity.json").write_text(json.dumps(results,indent=2)+chr(10))
