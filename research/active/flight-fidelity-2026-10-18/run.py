"""Existing-flight mechanism screen; does not modify runtime physics or gains.

Run with the simulation Python environment. MICROAGU_REPLAY_DATA selects the
existing causal-input and propulsion NPZ directory. All inputs are hashed.
"""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
import hashlib, json, importlib.util, sys
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[3]
OUT=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('historical_replay',ROOT/'research/archive/rigging-refinement-2026-10-15/study.py')
study=importlib.util.module_from_spec(spec);spec.loader.exec_module(study)
from microagu_sim.compat.profiles import make_simulator
CANDIDATES={'baseline':{}}
for key,values in {'CLbrake':[.5,1.5],'CDbrake':[.5,1.5],'joint_k_roll':[.2,.5,2], 'joint_k_yaw':[.5,2], 'Cnbeta':[0,2], 'Cnr':[.5,2], 'canopy_half_arc':[.8,1.2]}.items():
 for value in values:CANDIDATES[f'{key}_x{value}']={'scale':{key:value}}
for key,values in {'Cldelta':[-.03,.03],'Cndelta':[-.03,.03],'CYbeta':[-.2]}.items():
 for value in values:CANDIDATES[f'{key}_{value}']={'set':{key:value}}
METRICS=['roll_rate','pitch_rate','yaw_rate','bank','course','climb']
def one(job):
 flight,start,stage,name,alpha=job;cfg=CANDIDATES[name]
 day=study.FLIGHTS[flight][0];a=np.load(study.DATA/f'{day}-causal-input.npz');b=np.load(study.DATA/f'{day}-propulsion-9x45.npz')
 i=int(np.argmin(abs(a['t']-start)));sl=slice(i,i+751);t=a['t'][sl];assert len(t)==751 and np.max(abs(np.diff(t)-.04))<1e-5
 assert a['valid'][sl].all() and np.max(abs(a['t']-b['t']))<1e-6
 sim=make_simulator(flight,profile='as_built',substeps=32)
 for k,v in cfg.get('scale',{}).items():sim.parameters[k]*=v
 sim.parameters.update(cfg.get('set',{}));initial=study.init(sim,a,b,i,alpha)
 u=np.column_stack([a['U'][sl,0],a['brake_mm'][sl]])
 if cfg.get('response_tau_s',0)>0:
  # Diagnostic lumped brake response, not a measured servo transfer function.
  gain=1-np.exp(-.04/cfg['response_tau_s'])
  for n in range(1,len(u)):u[n,1:]=u[n-1,1:]+gain*(u[n,1:]-u[n-1,1:])
 y,rpm=sim.replay(initial,u,voltage=float(b['voltage'][i]),rpm=float(b['rpm'][i]),wind=a['wind'][i])
 rp=Rotation.from_quat(y[:,3:7]);rates=rp.inv().apply(y[:,10:13]);nav=y[:,7:10]+np.cross(y[:,10:13],rp.apply(sim.geometry['imu']));obs=a['Y'][sl];bank=rp.as_euler('xyz')[:,0]
 wrap=lambda x:np.arctan2(np.sin(x),np.cos(x));errors=np.column_stack([np.rad2deg(rates-obs[:,3:6]),np.rad2deg(wrap(bank-obs[:,0])),np.rad2deg(wrap(np.arctan2(nav[:,1],nav[:,0])-np.arctan2(obs[:,7],obs[:,6]))),nav[:,2]-obs[:,8]])
 masks={'all':np.ones(len(t),bool),'settled':t>=t[0]+2,'late':t>=t[0]+10}
 if start==438:masks['turn']=(t>=447)&(t<=456)
 metrics={phase:dict(zip(METRICS,np.sqrt(np.mean(errors[mask]**2,axis=0)).tolist())) for phase,mask in masks.items()}
 row={'flight':flight,'start':start,'stage':stage,'candidate':name,'alpha':alpha,'finite':bool(np.isfinite(y).all()),'metrics':metrics}
 if start==438:
  mask=masks['turn']; summary=lambda bk,v:{'peak_abs_bank_deg':float(np.max(abs(np.rad2deg(bk[mask])))),'course_rotation_deg':float(np.rad2deg(np.unwrap(np.arctan2(v[mask,1],v[mask,0]))[-1]-np.unwrap(np.arctan2(v[mask,1],v[mask,0]))[0])),'mean_climb_mps':float(-np.mean(v[mask,2]))}
  row['turn_summary']={'model':summary(bank,nav),'flight':summary(obs[:,0],obs[:,6:9])}
  if stage in ['selected','baseline']:
   np.savez_compressed(OUT/f'{stage}-hood-river-turn-{alpha}.npz',time=t,modeled_state=y,modeled_body_rates=rates,modeled_imu_velocity=nav,observed=obs,commands=u,rpm=rpm,wind=a['wind'][i])
 return row
def batch(jobs):
 with ProcessPoolExecutor(max_workers=4) as pool:return list(pool.map(one,jobs))
def main():
 train=[(f,t) for f,(_,starts) in study.FLIGHTS.items() for t in starts[::2]]+[('hood_river',438)]
 check=[(f,t) for f,(_,starts) in study.FLIGHTS.items() for t in starts[1::2]]
 protocol={'purpose':'Identify coupled-response mechanisms with existing 132 mm flights; no runtime promotion','candidates':CANDIDATES,'training':train,'checks':check,'initial_alpha_rad':[.25,.35,.45], 'selection':'Mean six-metric normalized settled RMSE across development windows; bank/course/velocity included. Select before checking eight historical windows. These are not new independent flights.', 'promotion_gate':'At least 5% mean improvement on checks, at least 6/8 windows improved, and no check-window composite worsened more than 10%; Hood River turn course rotation must also improve. Passing this screen is not sufficient for firmware promotion.'}
 (OUT/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
 hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for day,_ in study.FLIGHTS.values() for p in [study.DATA/f'{day}-causal-input.npz',study.DATA/f'{day}-propulsion-9x45.npz']}
 (OUT/'input-hashes.json').write_text(json.dumps(hashes,indent=2)+'\n')
 rows=batch([(f,t,'training',name,.35) for name in CANDIDATES for f,t in train]);(OUT/'training.json').write_text(json.dumps(rows,indent=2)+'\n')
 refs={(r['flight'],r['start']):r['metrics']['settled'] for r in rows if r['candidate']=='baseline'}
 def score(row):return float(np.mean([row['metrics']['settled'][k]/max(refs[row['flight'],row['start']][k],1e-6) for k in METRICS]))
 ranking=sorted([{'candidate':name,'score':float(np.mean([score(r) for r in rows if r['candidate']==name]))} for name in CANDIDATES],key=lambda r:r['score'])
 selected=ranking[0]['candidate'];(OUT/'selection.json').write_text(json.dumps({'selected':selected,'ranking':ranking},indent=2)+'\n');print('Frozen selection',selected,ranking[:5],flush=True)
 checks=batch([(f,t,'check',name,alpha) for name in ['baseline',selected] for f,t in check for alpha in [.25,.35,.45]])
 (OUT/'checks.json').write_text(json.dumps(checks,indent=2)+'\n')
 turns=batch([('hood_river',438,'baseline' if name=='baseline' else 'selected',name,alpha) for name in ['baseline',selected] for alpha in [.25,.35,.45]])
 (OUT/'turn.json').write_text(json.dumps(turns,indent=2)+'\n')
 base={(r['flight'],r['start'],r['alpha']):r for r in checks if r['candidate']=='baseline'}
 ratios=[]
 for f,t in check:
  ratios.append(float(np.mean([np.mean([r['metrics']['settled'][k]/base[f,t,r['alpha']]['metrics']['settled'][k] for k in METRICS]) for r in checks if r['flight']==f and r['start']==t and r['candidate']==selected])))
 result={'selected':selected,'mean_check_ratio':float(np.mean(ratios)),'per_window_ratios':ratios,'improved_windows':sum(v<1 for v in ratios),'worst_window_ratio':max(ratios),'aggregate_gate':bool(np.mean(ratios)<=.95 and sum(v<1 for v in ratios)>=6 and max(ratios)<=1.1),'runtime_changed':False}
 (OUT/'summary.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
if __name__=='__main__':main()
