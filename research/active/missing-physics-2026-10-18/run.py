
import concurrent.futures as cf
import hashlib,json,subprocess
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from microagu_sim import compat
from microagu_sim.compat import dynamics
from microagu_control.sysid.data import load_session
from microagu_control.sysid.model import configured_simulator,initialize,replay
from microagu_control.sysid.metrics import metrics

OUT=Path(__file__).resolve().parent
BIN=OUT/"target/release/missing-physics-study"
def execute(request,field="rows"):
    proc=subprocess.run([str(BIN)],input=json.dumps(request)+chr(10),text=True,capture_output=True,check=True)
    result=json.loads(proc.stdout)
    if "error" in result:raise RuntimeError(result["error"])
    return np.array(result[field])
def endpoints(y,p):
    rp=Rotation.from_quat(y[3:7]);rc=Rotation.from_quat(y[16:20])
    ap=rp.apply([[p["hinge_x"],p["hinge_y"]+side*p["riser_halfspacing"],p["hinge_z"]] for side in [-1,1] for fore in [-1,1]])
    ac=rc.apply([[fore*.15,side*p["support_halfspacing"],0] for side in [-1,1] for fore in [-1,1]])
    return y[:3]+ap,y[13:16]+ac,ap,ac
def rest_lengths(sim,k):
    reference=np.load(Path(compat.__file__).parent/f"equilibrium-trim-{sim.flight}-0.0-0.0.npz")["state"]
    a,b,_,_=endpoints(reference,sim.parameters);length=np.linalg.norm(b-a,axis=1)
    vertical=-(b-a)[:,2]/length
    tension=sim.geometry["payload_mass"]*9.80665/(4*vertical)
    return (length-tension/k).tolist()
def initialize_risers(y,sim,e):
    # Use one fixed rigging-reference shape, rotated with the observed payload.
    # All four lengths are geometrically compatible, with the same fixed preload.
    # This changes unmeasured canopy state only; log-observed payload state remains.
    y=y.copy()
    ref=np.load(Path(compat.__file__).parent/f"equilibrium-trim-{sim.flight}-0.0-0.0.npz")["state"]
    delta=Rotation.from_quat(y[3:7])*Rotation.from_quat(ref[3:7]).inv()
    y[16:20]=(delta*Rotation.from_quat(ref[16:20])).as_quat()
    separation=delta.apply(ref[13:16]-ref[:3])
    y[13:16]=y[:3]+separation
    y[23:26]=y[10:13]
    y[20:23]=y[7:10]+np.cross(y[10:13],separation)
    return y
def prepare(segment,options):
    sim=configured_simulator(segment)
    y=initialize(sim,segment,.35,segment.wind)
    e=options.copy()
    reference_init=e.pop("reference_init",False)
    if e.get("suspension_k",0):
        e["rest_lengths"]=rest_lengths(sim,e["suspension_k"])
        y=initialize_risers(y,sim,e)
    elif reference_init:y=initialize_risers(y,sim,e)
    return dict(parameters=dynamics.params(sim.parameters).tolist(),mass=sim.geometry["mass"],
        state=y.tolist(),commands=[sim.controls(*u).tolist() for u in segment.commands],
        wind=segment.wind.tolist(),voltage=segment.voltage,rpm=segment.rpm,substeps=32,experiment=e)
def one(segment,label,options):
    try:
        req=prepare(segment,options)
        path=OUT/"inputs"/f"{segment.name}-{label}.json";path.write_text(json.dumps(req))
        rows=execute(req)
        rates=np.rad2deg(Rotation.from_quat(rows[:,3:7]).inv().apply(rows[:,10:13]))
        obs=np.rad2deg(segment.observation[:,3:6])
        result=dict(name=segment.name,flight=segment.flight,variant=label,
            metrics=metrics(obs,rates),
            uncentered_rate_rmse=np.sqrt(np.mean((rates[50:]-obs[50:])**2,axis=0)).tolist(),
            tension_min_N=rows[50:,27:31].min(0).tolist(),tension_max_N=rows[50:,27:31].max(0).tolist(),
            slack_fraction=np.mean(rows[50:,27:31]<=0,axis=0).tolist(),
            moment_peak_Nm=np.max(abs(rows[50:,31:34]),axis=0).tolist(),
            input_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        if segment.name=="hood_river-438":
            m=(segment.time>=447)&(segment.time<=456)
            result["powered_turn"]=dict(mean=rates[m].mean(0).tolist(),rmse=np.sqrt(np.mean((rates[m]-obs[m])**2,axis=0)).tolist())
        if label=="baseline":
            _,state,_=replay(segment);result["parity_max"]=float(abs(rows[:,:26]-state).max())
        np.savez_compressed(OUT/"traces"/f"{segment.name}-{label}.npz",time=segment.time,rows=rows,rates=rates,observed=obs)
        return result
    except Exception as error:return dict(name=segment.name,flight=segment.flight,variant=label,error=str(error))
if __name__=="__main__":
    for d in ["inputs","traces"]:(OUT/d).mkdir(exist_ok=True)
    _,segments,hashes=load_session(OUT/"session.json")
    variants=json.loads((OUT/"protocol.json").read_text())["variants"]
    with cf.ThreadPoolExecutor(max_workers=4) as pool:
        results=list(pool.map(lambda args:one(*args),[(s,k,v) for s in segments for k,v in variants.items()]))
    (OUT/"results.json").write_text(json.dumps(results,indent=2)+chr(10))
    (OUT/"input-hashes.json").write_text(json.dumps(hashes,indent=2)+chr(10))
    print("replays",len(results),"errors",sum("error" in r for r in results))
