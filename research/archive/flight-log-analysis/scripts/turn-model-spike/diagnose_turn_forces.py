from pathlib import Path
import json,numpy as np
from scipy.ndimage import gaussian_filter1d
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
O=Path(__file__).parent;fig,axes=plt.subplots(2,3,figsize=(12,6),sharex=True);rows=[]
for row,(d,s) in enumerate([('2026-01-03',450),('2026-02-15',3387)]):
 a=np.load(O/f'{d}-{s}-panel-only-lateral.npz');inp=np.load(O/(d+'-causal-input.npz'));i=np.searchsorted(inp['t'],s);wind=inp['wind'][i];vals={}
 for name,y in [('logged',a['truth']),('model',a['model'])]:
  v=y[:,3:]-wind;speed=np.linalg.norm(v[:,:2],axis=1);acc=np.gradient(y[:,3:],.04,axis=0);right=np.column_stack([-v[:,1],v[:,0]])/speed[:,None];lateral=np.sum(acc[:,:2]*right,axis=1);support=9.80665-acc[:,2];vals[name]=[speed,gaussian_filter1d(lateral,2),gaussian_filter1d(support,2)]
  for ax,z in zip(axes[row],vals[name]):ax.plot(a['t'],z,label=name,color='black' if name=='logged' else '#2583a2')
 rows.append(dict(day=d,start=s,logged_mean=[float(np.mean(z[5:-5])) for z in vals['logged']],model_mean=[float(np.mean(z[5:-5])) for z in vals['model']]))
 for ax,title,unit in zip(axes[row],['Horizontal airspeed','Signed lateral acceleration','Upward specific support'],['m/s','m/s²','m/s²']):ax.set_title(('HR' if d.endswith('03') else 'TL')+f' {s}: '+title);ax.set_ylabel(unit);ax.grid(alpha=.2)
axes[0,0].legend()
for ax in axes[-1]:ax.set_xlabel('Time (s)')
fig.suptitle('Trajectory mismatch: speed and force components');fig.tight_layout();fig.savefig(O/'iteration-turn-force-components.png',dpi=150);(O/'turn-force-components.json').write_text(json.dumps(dict(scope='Diagnostic differentiated EKF/model velocity; centered 0.08s Gaussian smoothing for display only, assumed wind. Means omit first/last 0.2s.',components=['horizontal_speed','signed_lateral_acceleration','upward_specific_support'],results=rows),indent=2));print(json.dumps(rows))
p=O/'physics-development.html';h=p.read_text(encoding='utf-8');section='''<section id="turn-force-components"><h2>The heading error includes a trajectory error</h2><p>At HR 450, the panel-only model has final heading error −110.2° and air-relative course error −104.0°. Thus payload heading motion alone does not explain the failure. At TL 3387, the corresponding errors are −0.34° and +7.08°, showing that the mismatch varies strongly by maneuver.</p><img src="iteration-turn-force-components.png" alt="Horizontal speed, lateral acceleration, and upward specific support for two logged and simulated turns" style="max-width:100%"><p>These diagnostic accelerations come from differentiated EKF and model velocities; centered 0.08 s Gaussian smoothing is used only for display, with edge samples omitted from summary means. Specific upward support is g minus down acceleration. Horizontal airspeed uses the assumed wind. This comparison localizes errors in the flight path and does not measure canopy lift or canopy attitude directly.</p></section>'''
if 'id="turn-force-components"' not in h:h=h.replace('</body>',section+'\n</body>');p.write_text(h,encoding='utf-8')
