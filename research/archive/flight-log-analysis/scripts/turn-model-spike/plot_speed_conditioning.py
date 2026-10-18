from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
O=Path(__file__).parent;r=json.loads((O/'speed-conditioned-diagnostic.json').read_text());fig,axes=plt.subplots(2,3,figsize=(12,6),sharex=True)
for ax,row in zip(axes.flat,r['results']):
 d,s=row['day'],row['start'];a=np.load(O/(d+'-causal-input.npz'));wind=a['wind'][np.searchsorted(a['t'],s)]
 for suffix,label,color in [('panel-only-lateral','Baseline','#9aadb7'),('speed-conditioned','Logged speed imposed','#247d9e')]:
  z=np.load(O/f'{d}-{s}-{suffix}.npz');v=z['model'][:,3:]-wind;course=np.unwrap(np.arctan2(v[:,1],v[:,0]));ax.plot(z['t'],np.rad2deg(course-course[0]),label=label,color=color)
 v=z['truth'][:,3:]-wind;course=np.unwrap(np.arctan2(v[:,1],v[:,0]));ax.plot(z['t'],np.rad2deg(course-course[0]),color='black',label='Logged course');ax.set_title(('HR' if d.endswith('03') else 'TL')+f' {s} s');ax.set_ylabel('Air-course change (deg)');ax.grid(alpha=.2)
axes[0,0].legend(fontsize=7)
for ax in axes[-1]:ax.set_xlabel('Time (s)')
fig.suptitle('Imposing measured speed does not consistently recover turning');fig.tight_layout();fig.savefig(O/'iteration-speed-conditioning.png',dpi=150)
p=O/'physics-development.html';h=p.read_text(encoding='utf-8');section='''<section id="speed-conditioning"><h2>Separating speed error from turn-direction dynamics</h2><p>A diagnostic replay imposes logged horizontal airspeed magnitude after each 0.04 s step. Equal velocity increments are applied to both bodies, preserving their instantaneous relative joint velocity. Attitude, vertical velocity, and horizontal course direction remain model-predicted. This is an artificial intervention using future data, not a deployable controller or validated simulation.</p><img src="iteration-speed-conditioning.png" alt="Course prediction before and after imposing logged speed for six turns" style="max-width:100%"><p>At TL 326 and TL 402, final course errors improve from 34.6° and 44.6° to 1.6° and 9.3°. At HR 450, error remains −73.4° despite the imposed speed. Other turns worsen: HR 447 ends +70.2° and TL 3387 −66.7°. Thus excess deceleration contributes to some failures, but fixing speed alone does not consistently recover turning. The intervention requires mean absolute speed corrections equivalent to 2.0–3.4 m/s²; these are artificial velocity corrections, not measured missing thrust.</p></section>'''
if 'id="speed-conditioning"' not in h:h=h.replace('</body>',section+'\n</body>');p.write_text(h,encoding='utf-8')
