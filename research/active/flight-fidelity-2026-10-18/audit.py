"""Render the recorded-input benchmark and audit the selected diagnostic."""
from pathlib import Path
import importlib.util,json,hashlib,subprocess
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[3];OUT=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('audit_run',OUT/'run.py');run=importlib.util.module_from_spec(spec);spec.loader.exec_module(run)
from microagu_sim.compat import profiles,dynamics
import microagu_sim._native as native
hashes=json.loads((OUT/'input-hashes.json').read_text());assert hashes==json.loads((ROOT/'research/archive/as-built-simulator-2026-10-15/input-hashes.json').read_text())
lag=json.loads((OUT/'lag-training.json').read_text())
rows=json.loads((OUT/'training.json').read_text());checks=json.loads((OUT/'checks.json').read_text());turns=json.loads((OUT/'turn.json').read_text())
# Verify current as-built configuration reproduces historical check errors.
old=json.loads((ROOT/'research/archive/as-built-simulator-2026-10-15/checks.json').read_text());errors=[]
for row in checks:
 if row['candidate']!='baseline':continue
 ref=next(r for r in old if r['variant']=='routed' and r['flight']==row['flight'] and r['start']==row['start'] and r['alpha']==row['alpha'])
 errors.extend(abs(row['metrics']['all'][k]-ref['metrics'][k]) for k in run.METRICS)
reproduction=max(errors);assert reproduction<1e-7,reproduction
# Keep mechanism probe trajectories explicitly distinct from a selected candidate.
probe=run.one(('hood_river',438,'selected','Cndelta_0.03',.35))
(OUT/'selected-hood-river-turn-0.35.npz').rename(OUT/'yaw-probe-hood-river-turn-0.35.npz')
run.one(('hood_river',438,'selected','CDbrake_x1.5',.35))
original=run.make_simulator
def refined(*args,**kwargs):kwargs['substeps']=64;return original(*args,**kwargs)
run.make_simulator=refined
fine=run.one(('hood_river',438,'numerical-check','baseline',.35));coarse=next(r for r in turns if r['candidate']=='baseline' and r['alpha']==.35)
numeric=max(abs(fine['metrics']['turn'][k]-coarse['metrics']['turn'][k]) for k in run.METRICS)
audit={'historical_baseline_max_absolute_metric_difference':reproduction,'32_vs_64_substeps_max_turn_metric_difference':numeric,'input_hashes_match':True,'all_finite':all(r['finite'] for r in rows+checks+turns+lag),'replays':len(rows)+len(checks)+len(turns)+len(lag)+3,'git_revision':subprocess.check_output(['git','-C',str(ROOT),'rev-parse','HEAD'],text=True).strip(),'code_hashes':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(native.__file__),Path(profiles.__file__),Path(dynamics.__file__),OUT/'run.py']}}
(OUT/'audit.json').write_text(json.dumps(audit,indent=2)+'\n');(OUT/'yaw-probe.json').write_text(json.dumps(probe,indent=2)+'\n')
print(json.dumps(audit,indent=2))
