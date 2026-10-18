"""Numerical/provenance checks for the wind-calibration experiment."""
import hashlib
import json
from pathlib import Path
import numpy as np
import run
from microagu_sim import _native
from microagu_sim.compat import dynamics

HERE = Path(__file__).resolve().parent
def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

runtime = Path(dynamics.__file__).parent / 'as-built-calibration.json'
before = digest(runtime)
checks = sum([json.loads((HERE/f).read_text()) for f in ['checks.json','checks-extended.json']], [])
fits = sum([json.loads((HERE/f).read_text()) for f in ['fit.json','fit-extended.json']], [])
assert len(checks)==576 and len(fits)==768
assert all(r['finite'] and r['valid_fraction']==1 for r in checks+fits)
prior = json.loads((HERE.parent/'as-built-simulator-2026-10-15/checks.json').read_text())
prior = {(r['flight'], r['start'], r['alpha']):r for r in prior if r['variant']=='routed'}
reproduction = max(abs(r['metrics'][k]-prior[r['flight'],r['start'],r['alpha']]['metrics'][k])
                   for r in checks if r['variant']=='original' and r['damping']==3 for k in r['metrics'])
assert reproduction < 1e-6, reproduction
bulk = next(r for r in checks if r['variant']=='original' and r['alpha']==.35)
job = (bulk['flight'], bulk['start'], 'original', 3., 30, .35, False, 32)
step = run.replay((*job, True))
parity = max(abs(step['metrics'][k]-bulk['metrics'][k]) for k in bulk['metrics'])
assert parity < 1e-8, parity
ref = json.loads((HERE/'references.json').read_text())[0]
legacy = run.study.replay((ref['flight'],ref['start'],'legacy',(1.,1.,1.),10,.35,.06,32))
legacy_error = max(abs(legacy['metrics'][k]-ref['metrics'][k]) for k in ref['metrics'])
assert legacy_error < 1e-8
refinement = []
for variant in ['original','smoothed','down_minus1','down_plus1']:
    candidates = [r for r in checks if r['variant']==variant and r['damping']>3]
    worst = max(candidates,key=lambda r:r['metrics']['pitch_rate'])
    refined = run.replay((worst['flight'],worst['start'],variant,worst['damping'],30,worst['alpha'],False,64))
    assert refined['finite']
    relative = {k:abs(refined['metrics'][k]/worst['metrics'][k]-1) for k in worst['metrics']}
    assert max(relative.values()) < .001, relative
    refinement.append(dict(variant=variant, flight=worst['flight'], start=worst['start'], alpha=worst['alpha'],
                           damping=worst['damping'], relative_metric_change=relative))
assert digest(runtime)==before
paths = [runtime, Path(_native.__file__), HERE/'run.py', HERE.parent/'as-built-simulator-2026-10-15/calibrate.py',
         HERE.parent/'rigging-refinement-2026-10-15/study.py', HERE.parent/'wind-smoother-2026-10-16/run.py',
         HERE.parent/'wind-smoother-2026-10-16/smoother.py']
for flight,(day,_) in run.study.FLIGHTS.items():
    paths += [run.study.DATA/f'{day}-causal-input.npz',run.study.DATA/f'{day}-propulsion-9x45.npz',
              run.WIND/f'{flight}-0.npz']
result = dict(finite_fit_replays=len(fits), finite_check_replays=len(checks),
              historical_check_reproduction_max_absolute_error=reproduction,
              constant_wind_bulk_vs_step_max_absolute_error=parity,
              legacy_reference_max_absolute_error=legacy_error,
              integration_refinement=refinement, runtime_calibration_unchanged=True,
              hashes={str(p):digest(p) for p in paths})
(HERE/'audit.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k!='hashes'},indent=2))
