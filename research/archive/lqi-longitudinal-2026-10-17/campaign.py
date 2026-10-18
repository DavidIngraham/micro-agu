"""Frozen-gate metrics and bounded parallel runs against packaged firmware."""
import os,json,sys,subprocess
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import numpy as np
from scipy.spatial.transform import Rotation
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2];VIEW=ROOT/'viewer'
ENV=dict(os.environ,PATH='/home/david/emsdk/node/24.19.0_64bit/bin:'+os.environ.get('PATH','/usr/bin:/bin'))
import power
def rms(x):return float(np.sqrt(np.mean(x*x)))
def assess(p):
 d=json.loads(p.read_text());s=d['spec'];r=dict(spec=s,complete=d['complete'],failure=d['failure'],reasons=[])
 if not d['complete'] or not d['samples']:return dict(r,passed=False)
 a=d['samples'];t=np.array([x['time'] for x in a]);x=np.array([x['truth'] for x in a]);w=np.array([x['wind'] for x in a]);ph=np.array([x['phase'] for x in a])
 rot=Rotation.from_quat(x[:,3:7]);pitch=np.rad2deg(rot.inv().apply(x[:,23:26]-x[:,10:13])[:,1]);speed=np.linalg.norm(x[:,7:10]-w,axis=1)
 r['phases']={}
 disturbed=bool(s.get('turbulence',0) or s.get('wind',0) or s.get('thermal',0))
 for name,goal in [('climb',s.get('upDemand',s['demand'])),('descend',-s.get('downDemand',s['demand']))]:
  m=ph==name;m &= t>t[m][0]+10
  error=-x[m,9]-goal
  z=dict(vertical_mean=float(np.mean(-x[m,9])),vertical_bias=float(np.mean(error)),vertical_rms=rms(error),pitch_rms=rms(pitch[m]),pitch_peak=float(max(abs(pitch[m]))),throttle_mean=float(np.mean([a[i]['command'][0] for i in np.flatnonzero(m)])),airspeed_mean=float(np.mean(speed[m])))
  r['phases'][name]=z
  if abs(z['vertical_bias'])>(.35 if disturbed else .15):r['reasons'].append(name+' bias')
  if z['vertical_rms']>(.55 if disturbed else .25):r['reasons'].append(name+' RMS')
 quiet=t>145
 r['quiet']=dict(pitch_rms=rms(pitch[quiet]),height_rms=rms(-x[quiet,2]-80),vertical_rms=rms(x[quiet,9]))
 for key,limit in [('pitch_rms',3),('height_rms',1.5),('vertical_rms',.2)]:
  if r['quiet'][key]>limit:r['reasons'].append('quiet '+key)
 r['speed_min']=float(speed.min())
 if r['speed_min']<2:r['reasons'].append('airspeed minimum')
 watts=power.shaft_power(np.array([y['rpm'] for y in a]),rot.inv().apply(x[:,7:10]-w)[:,0])
 r['throttle_ceiling_fraction']=float(np.mean([y['command'][0]>=.99*.6*22.2/s.get('voltage',22.2) for y in a]))
 r['energy_Wh']=float(np.trapezoid(watts,t)/3600);r['shaft_W']=float(np.mean(watts))
 r['common_mm']=float(np.mean([min(y['brakes']) for y in a]));r['pitch_peak']=float(max(abs(pitch)));r['pitch_rms']=rms(pitch)
 r['passed']=not r['reasons'];return r
def run(s):
 s=dict(s)
 if 'runtime' in s:s['runtime']=str(HERE/Path(s['runtime']).name)
 out=HERE/'runs';out.mkdir(exist_ok=True);p=out/(s['id']+'.json');sp=out/(s['id']+'-spec.json')
 sp.write_text(json.dumps(s,indent=2))
 if not p.exists():
  with p.with_suffix('.log').open('w') as log:
   subprocess.run([str(VIEW/'node_modules/.bin/tsx'),'--tsconfig',str(VIEW/'tsconfig.json'),str(HERE/'case.mts'),str(sp),str(p)],cwd=VIEW,env=ENV,stdout=log,stderr=subprocess.STDOUT,timeout=270)
 if not p.exists():raise RuntimeError(p.with_suffix('.log').read_text())
 m=assess(p);p.with_name(p.stem+'-metrics.json').write_text(json.dumps(m,indent=2));print(json.dumps(m),flush=True);return m
if __name__=='__main__':
 phase=sys.argv[1] if len(sys.argv)>1 else 'screen'
 if phase=='screen':
  specs=[dict(id=f'nominal-{scheme}-{v}-{turn}',scheme=scheme,limit=v,demand=v,turn=turn,seed=73) for v in [.5,.75,1.] for turn in [0,35] for scheme in ['lqi','baseline']]
 else:specs=json.loads((HERE/(phase+'-specs.json')).read_text())
 (HERE/(phase+'-specs.json')).write_text(json.dumps(specs,indent=2))
 with ThreadPoolExecutor(max_workers=int(os.environ.get("CASE_WORKERS","3"))) as pool:results=list(pool.map(run,specs))
 (HERE/(phase+'-results.json')).write_text(json.dumps(results,indent=2))
