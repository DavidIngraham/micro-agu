import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path
O=Path(__file__).parent;rows=json.loads((O/'powered-trim-results.json').read_text());mapped=json.loads((O/'powered-trim-map-results.json').read_text());allrows=rows+mapped
checks=[]
for day in ['2026-01-03','2026-02-15']:
 label=f'neutral-{day}-0.4'
 for si in range(3):
  r={x['variant']:x for x in rows if x['case']['label']==label and x['start']==si};q=lambda v:r[v]['mean_speed_climb_turn_bank'][2]
  checks.append(dict(day=day,start=si,torque_off_turn_deg_s=q('torque_off'),reverse_symmetry_error_deg_s=abs(q('original')+q('torque_reversed')),offset_change_deg_s=abs(q('original')-q('offsets_off'))))
assert all(abs(r['torque_off_turn_deg_s'])<1e-8 and r['reverse_symmetry_error_deg_s']<1e-6 and r['offset_change_deg_s']<1e-8 for r in checks)
(O/'powered-trim-symmetry-checks.json').write_text(json.dumps(checks,indent=2))
fig,axes=plt.subplots(1,2,figsize=(13,5))
variants=['original','torque_off','torque_reversed','offsets_off','APC_torque_map'];colors=['#426b9f','#888888','#d7832a','#aa77aa','#448555']
for j,day in enumerate(['2026-01-03','2026-02-15']):
 vals=[]
 for v in variants:
  r=next(x for x in allrows if x['case']['label']==f'neutral-{day}-0.4' and x['start']==1 and x['variant']==v);vals.append(r['mean_speed_climb_turn_bank'][2])
 axes[0].bar(np.arange(5)+(j-.5)*.35,vals,width=.35,label=f'{1.385 if j==0 else 1.758:.3f}kg')
axes[0].set_xticks(range(5),['Original','Torque off','Torque reversed','Offsets off','APC torque'],rotation=20);axes[0].set(title='Released brakes,40% throttle,22.2V',ylabel='Settled turn rate (deg/s; right positive)');axes[0].legend();axes[0].axhline(0,color='k',lw=.8);axes[0].grid(axis='y',alpha=.2)
labels=[r['case']['label'] for r in rows if r['variant']=='original' and r['start']==1 and r['case']['label'].startswith('flight')]
for v,color in [('original',colors[0]),('torque_off',colors[1]),('offsets_off',colors[3]),('APC_torque_map',colors[4])]:
 vals=[]
 for label in labels:
  r=next(x for x in allrows if x['case']['label']==label and x['start']==1 and x['variant']==v);vals.append(r['mean_speed_climb_turn_bank'][2] if r['status']=='finite' else np.nan)
 axes[1].plot(range(4),vals,'o',linestyle='none',color=color,label=v)
obs=json.loads((O/'sustained-matched-comparison.json').read_text())['rows'];obs=[x['observed']['inferred_net_air_turn_deg_s'] for x in obs if x['model']=='baseline'];axes[1].plot(range(4),obs,'kx',ms=10,label='Flight average (inferred)');axes[1].set_xticks(range(4),['HR1020','HR840','TL1120','TL1020']);axes[1].set(title='Actual mean controls and voltage',ylabel='Turn rate (deg/s)');axes[1].legend(fontsize=8);axes[1].axhline(0,color='k',lw=.8);axes[1].grid(alpha=.2)
fig.suptitle('Powered trim ablations: causes isolated, not proposed physical corrections');fig.tight_layout();fig.savefig(O/'powered-trim-investigation.png',dpi=150)
print('Symmetry and zero-torque checks passed for both masses and all three starts.')
