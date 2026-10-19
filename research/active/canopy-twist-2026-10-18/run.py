import concurrent.futures as cf,hashlib,itertools,json,subprocess
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.signal import welch
from microagu_sim.compat import dynamics
from microagu_control.sysid.data import load_session
from microagu_control.sysid.model import configured_simulator,initialize,replay
from microagu_control.sysid.metrics import metrics
OUT=Path(__file__).resolve().parent
BIN=OUT/"target/release/canopy-twist-study"
def variants():
    v={"baseline":dict(enabled=False,frequency=1.6,damping=.3,arm_fraction=0.,beta_gain=0.)}
    for f,d,a,b in itertools.product([1.,1.6,2.5],[.15,.5],[-.08,-.03,.03,.08],[-.2,0.,.2]):
        v[f"f{f}-d{d}-a{a}-b{b}"]=dict(enabled=True,frequency=f,damping=d,arm_fraction=a,beta_gain=b)
    for f,d,a in itertools.product([2.5,4.],[.3,.7],[-.01,-.005,.005,.01]):
        v[f"f{f}-d{d}-a{a}-b0.0"]=dict(enabled=True,frequency=f,damping=d,arm_fraction=a,beta_gain=0.)
    return v
def execute(req):
    p=subprocess.run([str(BIN)],input=json.dumps(req)+chr(10),text=True,capture_output=True,check=True)
    r=json.loads(p.stdout)
    if "error" in r:raise ValueError(r["error"])
    return np.array(r["rows"])
def prepare(s,e,substeps=32,twist=0.):
    sim=configured_simulator(s,substeps)
    return dict(parameters=dynamics.params(sim.parameters).tolist(),mass=sim.geometry["mass"],state=initialize(sim,s,.35,s.wind).tolist(),
        commands=[sim.controls(*u).tolist() for u in s.commands],wind=s.wind.tolist(),voltage=s.voltage,rpm=s.rpm,
        experiment=e,substeps=substeps,twist=[twist,0.])
def one(args):
    s,label,e=args
    try:
        req=prepare(s,e);y=execute(req)
        rates=np.rad2deg(Rotation.from_quat(y[:,3:7]).inv().apply(y[:,10:13]))
        obs=np.rad2deg(s.observation[:,3:6]);m=metrics(obs,rates)
        m["uncentered_rmse"]=np.sqrt(np.mean((rates[50:-10]-obs[50:-10])**2,axis=0)).tolist()
        f,p=welch(rates[50:-10],fs=25,nperseg=256,axis=0);mask=(f>=.8)&(f<=5)
        m["model_peak_hz"]=float(f[np.flatnonzero(mask)[np.argmax(p[mask,2])]])
        row=dict(name=s.name,flight=s.flight,variant=label,metrics=m,peak_twist_deg=float(np.rad2deg(abs(y[:,26]).max())),rms_twist_deg=float(np.rad2deg(np.sqrt(np.mean(y[50:,26]**2)))))
        if label=="baseline":
            _,ref,_=replay(s);row["baseline_parity"]=float(abs(y[:,:26]-ref).max())
        np.savez_compressed(OUT/"traces"/f"{s.name}-{label}.npz",rows=y,rates=rates,time=s.time)
        return row
    except Exception as e:return dict(name=s.name,flight=s.flight,variant=label,error=str(e))
if __name__=="__main__":
    (OUT/"traces").mkdir(exist_ok=True)
    _,segments,hashes=load_session(OUT/"session.json")
    v=variants()
    (OUT/"protocol.json").write_text(json.dumps(dict(variants=v,modal_inertia=.001,soft_angle=.35,selection="Shared canopy parameters across both flights; exploratory reused data; no deployment."),indent=2))
    (OUT/"input-hashes.json").write_text(json.dumps(hashes,indent=2))
    rows=[]
    with cf.ProcessPoolExecutor(max_workers=4) as pool:
        for row in pool.map(one,[(s,k,e) for k,e in v.items() for s in segments]):
            rows.append(row)
            if len(rows)%50==0:print(len(rows),row["variant"],flush=True)
    (OUT/"results.json").write_text(json.dumps(rows,indent=2))
    print("replays",len(rows),"failed",sum("error" in x for x in rows),flush=True)
