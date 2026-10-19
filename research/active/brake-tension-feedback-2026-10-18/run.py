
"""Frozen diagnostic sweep; no optimizer and no production changes."""
import concurrent.futures as cf
import hashlib
import json
from pathlib import Path
import subprocess
import numpy as np
from scipy.spatial.transform import Rotation
from microagu_sim.compat import dynamics
from microagu_control.sysid.data import load_session
from microagu_control.sysid.model import configured_simulator, initialize, replay
from microagu_control.sysid.metrics import metrics

OUT=Path(__file__).resolve().parent
BIN=OUT/"target/release/brake-tension-study"
def execute(request):
    process=subprocess.run([str(BIN)],input=json.dumps(request)+"\n",text=True,capture_output=True,check=True)
    response=json.loads(process.stdout)
    if "error" in response: raise RuntimeError(response["error"])
    return np.array(response["rows"])
def prepare(segment):
    sim=configured_simulator(segment)
    state=initialize(sim,segment,.35,segment.wind)
    return dict(parameters=dynamics.params(sim.parameters).tolist(),mass=sim.geometry["mass"],
        state=state.tolist(),commands=[sim.controls(*u).tolist() for u in segment.commands],
        wind=segment.wind.tolist(),voltage=segment.voltage,rpm=segment.rpm,substeps=32)
def one(segment,label,options):
    request=prepare(segment)|options
    path=OUT/"inputs"/f"{segment.name}-{label}.json"
    path.write_text(json.dumps(request))
    try:
        rows=execute(request)
        rates=np.rad2deg(Rotation.from_quat(rows[:,3:7]).inv().apply(rows[:,10:13]))
        obs=np.rad2deg(segment.observation[:,3:6])
        result=dict(name=segment.name,variant=label,flight=segment.flight,metrics=metrics(obs,rates),
            uncentered_rate_rmse=np.sqrt(np.mean((rates[50:]-obs[50:])**2,axis=0)).tolist(),
            effective_pull_mm_min=rows[50:,27:29].min(0).tolist(),
            effective_pull_mm_max=rows[50:,27:29].max(0).tolist(),
            moment_peak_Nm=np.max(np.abs(rows[50:,29:32]),axis=0).tolist(),
            input_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        for key in ["effective_pull_mm_min","effective_pull_mm_max"]:
            result[key]=(np.array(result[key])*1000).tolist()
        if segment.name=="hood_river-438":
            mask=(segment.time>=447)&(segment.time<=456)
            result["powered_turn"]={"observed_mean":obs[mask].mean(0).tolist(),"model_mean":rates[mask].mean(0).tolist(),
                "rmse":np.sqrt(np.mean((rates[mask]-obs[mask])**2,axis=0)).tolist()}
        if label=="baseline":
            expected,states,rpm=replay(segment)
            result["baseline_parity_max"]=float(np.max(np.abs(rows[:,:26]-states)))
        np.savez_compressed(OUT/"traces"/f"{segment.name}-{label}.npz",time=segment.time,rows=rows,observed=obs,rates=rates)
        return result
    except Exception as error:return dict(name=segment.name,variant=label,error=str(error))
if __name__=="__main__":
    for d in ["inputs","traces"]:(OUT/d).mkdir(exist_ok=True)
    _,segments,hashes=load_session(OUT/"session.json")
    variants=json.loads((OUT/"protocol.json").read_text())["variants"]
    jobs=[(s,k,v) for s in segments for k,v in variants.items()]
    with cf.ThreadPoolExecutor(max_workers=4) as pool:
        results=list(pool.map(lambda a:one(*a),jobs))
    (OUT/"results.json").write_text(json.dumps(results,indent=2)+"\n")
    sources=[OUT/"src/main.rs",OUT/"run.py",OUT/"protocol.json",OUT/"session.json"]
    sources+=list((OUT.parents[2]/"simulation/crates/sim-core/src").glob("*.rs"))
    (OUT/"manifest.json").write_text(json.dumps({"input_hashes":hashes,"source_hashes":{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},"binary_sha256":hashlib.sha256(BIN.read_bytes()).hexdigest()},indent=2)+"\n")
    print("finished",len(results),"errors",sum("error" in r for r in results),flush=True)
