# AP_FLAKE8_CLEAN
"""Matched commanded pulses, individual level trim, no trajectory controller."""
from study import *
from scipy.optimize import brentq
from microagu_sim.compat.equilibrium import solve
OUT=HERE.parents[1]/'viewer/assets/recordings/rigging-comparison'

def setup(name):
 selected=json.loads((HERE/'selected-final.json').read_text())
 variant='legacy' if name=='legacy' else 'routed'
 tuning=selected[variant][0]['tuning']
 s,t=make('trout_lake',variant,tuning)
 oldphi=s.parameters['canopy_half_arc'];arc=s.parameters['b'];proj=arc*np.sin(oldphi)/oldphi
 if name.startswith('wide'):
  s.parameters['joint_k_yaw']*=.185/.132
  routing.configure(s,t['state'],spacing=.185)
  transfer={'wide':0,'wide_half':.5,'wide_flat':1,'wide_drag':1}[name]
  target=proj+transfer*(.185-.132)
  s.parameters['canopy_half_arc']=brentq(lambda x:arc*np.sin(x)/x-target,.01,oldphi) if transfer else oldphi
  if name=='wide_drag':s.parameters['k_drag']*=(proj/target)**2
 t=solve(s,t['state'],t['command'])
 if not t['success'] or np.max(abs(t['residual']))>1e-4:raise RuntimeError((name,t['message'],t['residual']))
 return s,t

def run(name,scenario):
 s,trim=setup(name);dt=.02;t=np.arange(4001)*dt;u=np.tile(trim['command'],(len(t),1))
 if scenario=='throttle':u[(t>=10)&(t<15),0]+=.03
 if scenario=='turns':
  u[(t>=10)&(t<20),2]+=5.;u[(t>=40)&(t<50),1]+=5.
 if scenario=='common':u[(t>=10)&(t<15),1:]+=3.
 y,r,applied=s.replay_actuated(trim['state'],u,rpm=float(trim['rpm']),dt=dt)
 y[:,:3]-=y[0,:3].copy();y[:,13:16]-=trim['state'][:3]
 pitch=R.from_quat(y[:,3:7]).inv().apply(y[:,10:13])[:,1]*180/np.pi
 course=np.unwrap(np.arctan2(y[:,8],y[:,7]))*180/np.pi
 pull=np.array([routing.diagnostic(s,a,c)['effective_m']*1000 if s.parameters['routing_enabled'] else s.controls(*c)[1:]*1000 for a,c in zip(y,applied)])
 trace=np.c_[t,-y[:,2],pitch,course-course[0],pull.mean(axis=1),pull[:,1]-pull[:,0],np.linalg.norm(y[:,7:10],axis=1)]
 rows=np.c_[t,y,r,np.full(len(t),22.2),applied]
 stride=5;rows=rows[::stride];trace=trace[::stride]
 metadata=dict(name=name,scenario=scenario,trim_command=trim['command'].tolist(),trim_speed=float(np.linalg.norm(trim['state'][7:10])),trim_residual=float(max(abs(trim['residual']))),half_arc=s.parameters['canopy_half_arc'],arc_span=s.parameters['b'],projected_span=float(s.parameters['b']*np.sin(s.parameters['canopy_half_arc'])/s.parameters['canopy_half_arc']),k_drag=s.parameters['k_drag'],yaw_stiffness=s.parameters['joint_k_yaw'],final_position=y[-1,:3].tolist(),height_range=[float(min(-y[:,2])),float(max(-y[:,2]))],max_heading_change=float(max(abs(course-course[0]))),pull_range_mm=[float(pull.min()),float(pull.max())])
 return rows,trace,metadata

if __name__=='__main__':
 OUT.mkdir(parents=True,exist_ok=True);(OUT/'poses').mkdir(exist_ok=True)
 runs=[];meta=[]
 for scenario in ['throttle','turns','common']:
  cache={}
  for name in ['legacy','current','wide','wide_half','wide_flat','wide_drag']:
   rows,trace,m=run(name,scenario);cache[name]=(rows,trace,m);meta.append(m);print(m,flush=True)
  for comparison,names in {'routing':['legacy','current'],'hardware':['current','wide'],'half':['current','wide_half'],'flattening':['current','wide_flat'],'drag':['current','wide_drag']}.items():
   for family,name in zip(['baseline','direct'],names):
    rows,trace,m=cache[name];case=f'{comparison}-{scenario}'
    (OUT/'poses'/f'{case}-truth-{family}.json').write_text(json.dumps(np.round(rows,7).tolist(),separators=(',',':')))
    runs.append(dict(case=case,family=family,observer=False,start=10,trace=np.round(trace,6).tolist(),metrics={k:float(np.sqrt(np.mean(trace[:,j]**2))) for j,k in [(1,'height_rms'),(2,'pitch_rms'),(3,'heading_rms')]}))
 (OUT/'data.json').write_text(json.dumps(dict(runs=runs,metadata=meta),separators=(',',':')))
 (HERE/'predictions.json').write_text(json.dumps(meta,indent=2))
