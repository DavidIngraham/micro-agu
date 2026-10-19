"""Joint diagnostic sweep; no selected model or gain changes."""
import json,itertools,concurrent.futures as cf
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.signal import welch
from microagu_control.sysid.data import load_session
from microagu_control.sysid.model import configured_simulator,initialize
from microagu_control.sysid.metrics import metrics
OUT=Path(__file__).resolve().parent
def variants():
    v={"baseline":{}}
    for j,c,d in itertools.product([.25,1.,4.],[.25,1.,4.],[.1,1.,3.]):
        v[f"J{j}-C{c}-D{d}"]=dict(added_Jy=j,Cmq=c,joint_c_pitch=d)
    for z in [0.,.25,.75]:
        v[f"offset{z}"]={"added_z2_value":z}
    for scale in [.7,1.3]:
        v[f"payload{scale}"]={"payload_scale":scale}
    for z in [0.,.25,.75]:
        for d in [.1,1.]:
            v[f"coupled-offset{z}-D{d}"]=dict(added_Jy=4.,Cmq=.25,joint_c_pitch=d,added_z2_value=z)
    for z in [-.5,.25,.5]:
        v[f"pitch-offset{z}"]={"added_z1_value":z}
        v[f"coupled-pitch-offset{z}"]=dict(added_Jy=4.,Cmq=.25,joint_c_pitch=.1,added_z1_value=z)
    return v
def apply(sim,options):
    for key,value in options.items():
        if key.endswith("_value"):sim.parameters[key[:-6]]=value
        elif key=="payload_scale":
            # Congruence transform: scale pitch radius of gyration, preserve SPD.
            for k in ["Ixy","Iyz"]:sim.parameters[k]*=np.sqrt(value)
            sim.parameters["Iyy"]*=value
            sim.geometry["inertia"][1,:]*=np.sqrt(value)
            sim.geometry["inertia"][:,1]*=np.sqrt(value)
        else:sim.parameters[key]*=value
    return sim
def one(args):
    segment,label,options=args
    try:
        sim=apply(configured_simulator(segment),options)
        state=initialize(sim,segment,.35,segment.wind)
        y,rpm=sim.replay(state,segment.commands,segment.voltage,segment.rpm,segment.wind)
        rates=np.rad2deg(Rotation.from_quat(y[:,3:7]).inv().apply(y[:,10:13]))
        obs=np.rad2deg(segment.observation[:,3:6])
        m=metrics(obs,rates)
        f,p=welch(rates[50:-10],fs=25,nperseg=256,axis=0)
        band=(f>=1.2)&(f<=1.9)
        m["model_peak_hz"]=float(f[np.flatnonzero(band)[np.argmax(p[band,2])]])
        m["uncentered_rmse"]=np.sqrt(np.mean((rates[50:-10]-obs[50:-10])**2,axis=0)).tolist()
        np.savez_compressed(OUT/"traces"/f"{segment.name}-{label}.npz",rates=rates,state=y,rpm=rpm)
        return dict(flight=segment.flight,name=segment.name,variant=label,metrics=m)
    except Exception as e:return dict(name=segment.name,variant=label,error=str(e))
if __name__=="__main__":
    (OUT/"traces").mkdir(exist_ok=True)
    _,segments,hashes=load_session(OUT/"session.json")
    v=variants()
    (OUT/"protocol.json").write_text(json.dumps(dict(variants=v,selection="Exploratory common-parameter sweep across both flights; all windows previously inspected; no held-out flight.",initialization_alpha=.35,substeps=32),indent=2))
    (OUT/"input-hashes.json").write_text(json.dumps(hashes,indent=2))
    results=[]
    with cf.ProcessPoolExecutor(max_workers=4) as pool:
        for r in pool.map(one,[(s,k,o) for k,o in v.items() for s in segments]):
            results.append(r)
            if len(results)%10==0:print(len(results),r["variant"],flush=True)
    (OUT/"results.json").write_text(json.dumps(results,indent=2))
