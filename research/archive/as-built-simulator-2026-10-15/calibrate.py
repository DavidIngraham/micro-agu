"""Repeatable as-built damping fit on existing logs; no launch parameter fitting.

MICROAGU_REPLAY_DATA locates the four preprocessed inputs. Historical alternating
windows are reused, so checks are held out from this fit, not independent flights.
"""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[3]/'simulation'
HIST=ROOT.parent/'research/archive/rigging-refinement-2026-10-15'
# Reuse the published observation/initialization method, with the migrated kernel.
from microagu_sim.compat import profiles, dynamics, routing
DATA = Path(dynamics.__file__).parent
spec=importlib.util.spec_from_file_location('study',HIST/'study.py')
study=importlib.util.module_from_spec(spec);sys.modules['study']=study;spec.loader.exec_module(study)
def migrated_model(flight,variant='legacy',tuning=(1.,1.,1.),height=.06,substeps=32):
    sim=profiles.make_simulator(flight,substeps=substeps)
    trim=dict(np.load(DATA/f'equilibrium-trim-{flight}-0.0-0.0.npz'))
    if variant=='routed':
        sim.parameters['joint_k_yaw']*=.066/.0743
        routing.configure(sim,trim['state'],ring_height=height)
    damping,brake,yaw=tuning
    for key in ['joint_c_pitch','joint_c_yaw']:sim.parameters[key]*=damping
    for key in ['CLbrake','CDbrake','Cldelta','Cndelta','brake_tilt_rad_per_m']:sim.parameters[key]*=brake
    sim.parameters['joint_k_yaw']*=yaw
    return sim,trim
study.make=migrated_model
OUT=ROOT.parent/'research/archive/as-built-simulator-2026-10-15'
OUT.mkdir(exist_ok=True)

def run():
    hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for day,_ in study.FLIGHTS.values()
            for p in [study.DATA/f'{day}-causal-input.npz',study.DATA/f'{day}-propulsion-9x45.npz']}
    expected=json.loads((HIST/'input-hashes.json').read_text())
    if hashes!=expected:raise ValueError('Flight inputs differ from the published dataset')
    (OUT/'input-hashes.json').write_text(json.dumps(hashes,indent=2)+'\n')
    candidates=[1.,1.5,2.,2.5,3.]
    rows=[]
    for damping in candidates:
        for flight,(_,starts) in study.FLIGHTS.items():
            for start in starts[::2]:
                rows.append(study.replay((flight,start,'routed',(damping,1.,.0743/.066),10,.35,.06,32)))
        print('fit damping',damping,flush=True)
    baseline=[study.replay((f,t,'legacy',(1.,1.,1.),10,.35,.06,32))
              for f,(_,starts) in study.FLIGHTS.items() for t in starts[::2]]
    refs={(r['flight'],r['start']):r['metrics'] for r in baseline}
    def score(d):
        group=[r for r in rows if r['tuning'][0]==d]
        if not all(r['finite'] for r in group):return 1e6
        return float(np.mean([sum(w*r['metrics'][k]/refs[r['flight'],r['start']][k]
            for k,w in [('roll_rate',1),('pitch_rate',1),('yaw_rate',1),('bank',.25),('climb',.25)])/3.5 for r in group]))
    ranked=sorted([dict(damping=d,score=score(d)) for d in candidates],key=lambda x:x['score'])
    selected=ranked[0]['damping']
    checks=[]
    for variant,tuning in [('legacy',(1.,1.,1.)),('routed',(selected,1.,.0743/.066))]:
        for f,(_,starts) in study.FLIGHTS.items():
            for t in starts[1::2]:
                for alpha in [.25,.35,.45]:
                    checks.append(study.replay((f,t,variant,tuning,30,alpha,.06,32)))
        print('checked',variant,flush=True)
    result=dict(schema='microagu.as-built-fit.v1',spacing_m=.132,ring_height_m=.06,
        damping_scale=selected,brake_scale=1.,yaw_scale=.0743/.066,ranked=ranked,
        bound_hit=selected in (min(candidates),max(candidates)),
        limitations='Same two historical flights; canopy initial attitude inferred, not measured. Launch/contact settings are assumptions, not fitted.',
        check_mean={v:{k:float(np.mean([r['metrics'][k] for r in checks if r['variant']==v and r['finite']]))
            for k in ['roll_rate','pitch_rate','yaw_rate','bank','course','climb']} for v in ['legacy','routed']})
    (OUT/'fit.json').write_text(json.dumps(rows+baseline,indent=2)+'\n')
    (OUT/'checks.json').write_text(json.dumps(checks,indent=2)+'\n')
    (OUT/'calibration.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))

if __name__=='__main__':run()
