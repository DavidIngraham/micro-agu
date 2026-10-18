from pathlib import Path
import ast, re, json, hashlib, tarfile
import numpy as np
from scipy.linalg import solve_discrete_are
from scipy.spatial.transform import Rotation
r=Path(__file__).resolve().parents[4]
a=r/'viewer/apps/applets/lqi-explainer'
h=(a/'deployed-gains.h').read_text()
z={m[1]:np.array(ast.literal_eval(m[2].replace('{','[').replace('}',']').replace('f',''))) for m in re.finditer(r'static constexpr float (\w+)(?:\[\d+\])+ = ([^;]+);',h)}
A,B,K,L=z['A'],z['B'],z['K'],z['L'];xs=z['XS'];idx=[0,1,5,6,7,8,9,10,15]
# Reconstruct the selected discrete LQI from the release's normalized plant.
H=np.zeros((3,16));H[0,15]=1
roll,pitch,_=z['EULER'];H[1,9:11]=np.array([np.sin(roll),np.cos(roll)])/np.cos(pitch)*xs[9:11]/np.deg2rad(10)
H[2,5:8]=z['VELOCITY']/np.linalg.norm(z['VELOCITY'])*xs[5:8]/.5
aa=np.eye(19);aa[:16,:16]=A;aa[16:,:16]=.02*H
bb=np.zeros((19,3));bb[:16]=B
Q=np.diag(np.r_[np.full(16,.02),[12,3,.3]]);Q[:16,:16]+=H.T@np.diag([4,1,.3])@H;Q[9,9]+=90;Q[12,12]+=90
R=np.diag([500,3000,1000]);P=solve_discrete_are(aa,bb,Q,R)
kk=np.linalg.solve(R+bb.T@P@bb,bb.T@P@aa);err=float(np.max(abs(kk-K)));assert err<1e-7,err
names=['Payload roll','Payload pitch','Relative roll','Relative pitch','Relative yaw','Forward air velocity','Side air velocity','Down air velocity','Payload roll rate','Payload pitch rate','Payload yaw rate','Canopy roll rate','Canopy pitch rate','Canopy yaw rate','Rotor speed','Height','Height-error integral','Turn-error integral','Speed-error integral']
units=['deg']*5+['m/s']*3+['deg/s']*6+['rpm','m','m·s','deg','m']
# Pedagogical positive perturbations; not measured operating statistics.
scales=[5]*5+[.5]*3+[10]*6+[500,1,1,10,.5]
physical=xs.copy();physical[:5]*=180/np.pi;physical[8:14]*=180/np.pi;physical[14]*=1000
norm=np.r_[physical,[1,10,.5]]
gains=(-100*z['US'][:,None]*K/norm[None,:]).tolist()
coupling=[]
for control in range(3):
 x=np.zeros(16);rows=[]
 for k in range(251):
  t=k*.02;u=np.zeros(3);u[control]=.1 if 1<=t<1.5 else 0 # 1 percentage-point pulse
  x=A@x+B@u
  if k%5==0:rows.append([round(t,3),float(x[5]*xs[5]),float(-x[7]*xs[7]),float(x[10]*xs[10]*180/np.pi),float(x[3]*10)])
 coupling.append(rows)
# Same local plant and published fixed observer; hidden initial angle, deterministic
# synthetic measurement noise. No EKF, nonlinear plant or experimental accuracy claim.
x=np.zeros(16);x[3]=.5;estimate=np.zeros(16);observer=[]
for k in range(401):
 t=k*.02;x=A@x;prediction=A@estimate
 noise=.02*np.sin(k*.71+np.arange(9)*1.3);y=x[idx]+noise
 residual=y-prediction[idx];estimate=prediction+L@residual
 if k%5==0:observer.append([round(t,3),float(x[3]*10),float(prediction[3]*10),float(estimate[3]*10),float(residual[1]*10)])
replay={};sources={}
for scheme in ['lqi','baseline']:
 p=r/f'research/archive/lqi-longitudinal-2026-10-17/runs/heldout-0-{scheme}.json'
 if p.exists():
  raw=p.read_bytes()
 else:
  with tarfile.open(p.parent.parent/'raw-runs.tar.gz','r:gz') as archive:
   member=archive.extractfile('runs/'+p.name)
   if member is None:raise FileNotFoundError(p.name)
   raw=member.read()
 d=json.loads(raw);sources[scheme]={'file':str(p.relative_to(r)),'sha256':hashlib.sha256(raw).hexdigest(),'spec':d['spec']}
 rows=[]
 for s in d['samples'][::5]:
  state=s['truth'];rot=Rotation.from_quat(state[3:7]);relrate=rot.inv().apply(np.array(state[23:26])-state[10:13])[1]*180/np.pi
  rows.append([s['time'],s['alt']-s['targetAlt'],float(relrate),s['command'][0]*100,s['phase']])
 replay[scheme]=rows
data={'provenance':{'revision':'20a8fb59c8c050848a5dd0b2af24eaca062f5d2a','header_sha256':hashlib.sha256(h.encode()).hexdigest(),'gain_reconstruction_max_error':err,'recordings':sources},'names':names,'units':units,'scales':scales,'rawK':K.tolist(),'physicalGain':gains,'trim':z['TRIM'].tolist(),'coupling':coupling,'observer':observer,'replay':replay}
(a/'data.json').write_text(json.dumps(data,separators=(',',':'))+'\n')
print('Generated data; K reconstruction max error',err)
