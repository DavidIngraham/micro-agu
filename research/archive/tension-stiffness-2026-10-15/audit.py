from run import *
import closed
import subprocess

def audit(checks_only=False):
 checks=[]
 for flight in FLIGHTS:
  s,a=make(flight,'fixed');u=np.tile(a['command'],(101,1));u[20:60,2]+=5
  y,r=s.replay(a['state'],u,rpm=float(a['rpm']))
  cmd=np.asarray([s.controls(*x) for x in u]);params=dynamics.params(s.parameters)[:70]
  # Isolated interpreter imports the installed production wheel, not this study's wheel.
  payload=dict(parameters=params.tolist(),mass=s.geometry['mass'],state=a['state'].tolist(),commands=cmd.ravel().tolist(),rpm=float(a['rpm']))
  code='import json,sys; from microagu_sim import Kernel; d=json.load(sys.stdin); print(json.dumps(Kernel(d["parameters"],d["mass"]).replay(d["state"],d["commands"],[0.,0.,0.],22.2,d["rpm"],.04,32,True)))'
  result=subprocess.run([sys.executable,'-I','-c',code],input=json.dumps(payload),capture_output=True,text=True,check=True)
  baseline=np.asarray(json.loads(result.stdout)).reshape(-1,27);out=baseline[:,:26];rpm=baseline[:,26]
  equivalence=float(np.max(abs(out-y)));assert equivalence<1e-12
  np.testing.assert_allclose(r,rpm,atol=1e-12,rtol=1e-12)
  for variant in VARIANTS:
   ss,_=make(flight,variant);diag=dynamics.tension_diagnostic(ss,a['state'],a['rpm'],a['command'])
   n1,r1=ss.step(a['state'],float(a['rpm']),a['command'],dt=.00001)
   n0,r0=s.step(a['state'],float(a['rpm']),a['command'],dt=.00001)
   trimdelta=float(np.max(abs(n1-n0)));assert trimdelta<1e-9
   # Independent frozen-coefficient check at a disturbed state.
   z=y[50];load=dynamics.tension_diagnostic(ss,z,r[50],u[50]);frozen,_=make(flight,'fixed')
   assert load[7]<1e-10
   if variant!='fixed':assert abs(load[6])<1e-10
   fr,fy=VARIANTS[variant]
   frozen.parameters['joint_k_roll']*=1+fr*(load[2]-1)
   frozen.parameters['joint_k_yaw']*=1+fy*(load[2]-1)
   ld=dynamics.tension_diagnostic(frozen,z,r[50],u[50]);loaderr=float(abs(ld[0]-load[0]));assert loaderr<1e-10
   aa,_=ss.step(z,r[50],u[50],dt=1e-7);bb,_=frozen.step(z,r[50],u[50],dt=1e-7)
   acceleration_difference=float(max(abs(aa-bb))/1e-7);assert acceleration_difference<1e-4
   checks.append(dict(flight=flight,variant=variant,baseline_state_max=equivalence,trim_step_max=trimdelta,frozen_load_error=loaderr,frozen_derivative_error=acceleration_difference,reference_load=float(diag[0])))
 if checks_only:
  print(json.dumps(checks,indent=2))
  return checks
 rows=json.loads((HERE/'flight-results.json').read_text())
 # Highest sampled load per flight, using primary initial condition.
 refined=[]
 for f in FLIGHTS:
  candidates=[x for x in rows if x['flight']==f and x['alpha']==.35 and x['variant']=='both']
  target=max(candidates,key=lambda x:x['load_quantiles'][-1])
  for variant in ['fixed','both']:
   refined.append(replay((f,target['start'],.35,variant,64)))
 cr=[]
 for c in closed.ex.scenarios():
  if c['name'] in ['strong_gust','limits_recovery']:
   for variant in ['fixed','both']:cr.append(closed.task((c,variant,32)))
 result=dict(checks=checks,flight_refinement=refined,controller_refinement=cr)
 (HERE/'audit-results.json').write_text(json.dumps(result,indent=2)+'\n')
 print(json.dumps(result,indent=2))
if __name__=='__main__':
 import argparse
 parser=argparse.ArgumentParser()
 parser.add_argument('--checks-only',action='store_true',help='Run force-law checks without redoing flight studies or replacing results')
 audit(parser.parse_args().checks_only)
