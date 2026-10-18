from pathlib import Path
import sys,json,ctypes,numpy as np
from scipy.spatial.transform import Rotation as Rot
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent;O=R.parent;sys.path.insert(0,str(O));import fast_rotor_torque as f
from run_sustained_envelope import p
ptr=np.ctypeslib.ndpointer(dtype=np.float64,flags='C_CONTIGUOUS');f.lib.panel_loads.argtypes=[ptr,ptr,ctypes.c_double,ctypes.c_double,ptr,ptr]
fig,axs=plt.subplots(2,4,figsize=(15,8));rows=[]
for di,day in enumerate(['2026-01-03','2026-02-15']):
 y=np.load(R/f'lateral-{day}-reference.npz')['state'][900];rot=Rot.from_quat(y[16:20]);v=rot.inv().apply(y[20:23]);v[1]=0.;w=np.zeros(3)
 def loads(left,right):
  out=np.zeros(6);f.lib.panel_loads(np.ascontiguousarray(v),w,left/.072,right/.072,f.params(p),out);return out
 base=loads(0,0)
 for side in ['left','right']:
  mm=np.linspace(0,30,61);off=p[side+'_offset'];effective=np.maximum(mm/1000-off,0);values=np.array([loads(b,0) if side=='left' else loads(0,b) for b in effective])-base
  for j,k in enumerate([2,0,3,5]):axs[di,j].plot(mm,values[:,k],label=side)
  rows.append(dict(day=day,side=side,commanded_mm=mm.tolist(),effective_mm=(effective*1000).tolist(),delta_force_moment=values.tolist(),velocity=v.tolist()))
 for j,label in enumerate(['Vertical force delta, down+ (N)','Forward force delta (N)','Roll moment delta (N m)','Yaw moment delta (N m)']):axs[di,j].set(title=day+' '+label,xlabel='Commanded brake travel (mm)');axs[di,j].axhline(0,color='gray',lw=.5);axs[di,j].grid(alpha=.2);axs[di,j].legend()
fig.suptitle('Current curved-panel brake law, including empirical left slack offset; force/moment increments at fixed air velocity');fig.tight_layout();fig.savefig(R/'brake-moment-audit.png',dpi=160);(R/'brake-moment-audit.json').write_text(json.dumps(rows,indent=2))
report=[]
for day,name in [('2026-01-03','2026-01-03 15-41-42'),('2026-02-15','2026-02-15 15-38-50')]:
 raw=json.loads((O.parent/name/'PARM.json').read_text());fields=[f'SERVO{s}_{k}' for s in [1,3,4] for k in ['MIN','MAX','TRIM','REVERSED','FUNCTION']];values={k:sorted(set(float(r['Value']) for r in raw if r['Name']==k)) for k in fields};report.append(dict(day=day,parameters=values))
(R/'pwm-recheck.json').write_text(json.dumps(dict(flights=report,drum_diameter_mm=55,assumed_servo_deg_per_us=.1,mm_per_us=np.pi*55/360*.1,left_conversion='(PWM3-800)*mm_per_us',right_conversion='(2400-PWM4)*mm_per_us',left_empirical_slack_mm=p['left_offset']*1000,distinction='PWM calibration and real asymmetry are separate from the fitted aerodynamic slack offset.'),indent=2));print(json.dumps(report,indent=2))
