from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
O=Path(__file__).parent;r=json.loads((O/'curved-panel-body-fit.json').read_text());old=json.loads((O/'balanced-body-fit.json').read_text());same=r['training_windows']==old['training_windows'];dev=[v for v in r['validation'] if v['group']=='development'];rem=[v for v in r['validation'] if v['group']=='remaining' and 'bank_rmse_deg' in v]
fig,axes=plt.subplots(2,3,figsize=(12,6),sharex=True)
for ax,row in zip(axes.flat,dev):
 d,s=row['day'],row['start'];a=np.load(O/f'{d}-{s}-curved-panel-body.npz');ax.plot(a['t'],np.rad2deg(a['truth'][:,0]),color='black',label='Logged');ax.plot(a['t'],np.rad2deg(a['model'][:,0]),color='#2687a3',label='Jointly refitted model');ax.set_title(('HR' if d.endswith('03') else 'TL')+f' {s} s');ax.set_ylabel('Bank (deg)');ax.grid(alpha=.2)
axes[0,0].legend(fontsize=8)
for ax in axes[-1]:ax.set_xlabel('Time (s)')
fig.suptitle('Curved-panel model after joint aerodynamic/suspension refit');fig.tight_layout();fig.savefig(O/'iteration-curved-panel-fit.png',dpi=150)
summary=dict(same_training_windows=same,remaining_median_bank=float(np.median([v['bank_rmse_deg'] for v in rem])),development_bank=[v['bank_rmse_deg'] for v in dev],success=r['success'],nfev=r['nfev']);(O/'curved-panel-summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary))
p=O/'physics-development.html';h=p.read_text(encoding='utf-8');section=f'''<section id="curved-panel-fit"><h2>Joint refit of the curved-panel model</h2><p>The new panel load implementation matches the Python reference to 2.85e-14 across 100 randomized cases. This run jointly refits 19 aerodynamic, suspension, engagement-offset, and brake-force and curvature parameters. Training-window identity against the earlier balanced fit: {same}. It uses latest-sample observations and controls. Target resampling differs from earlier interpolated-target checks, so small cross-run score differences are not attributed solely to physics.</p><img src="iteration-curved-panel-fit.png" alt="Logged and jointly refitted curved-panel model bank for six development turns" style="max-width:100%"><p>Median bank RMSE on the remaining internal comparison windows is {summary['remaining_median_bank']:.2f}°. Development bank RMSE ranges from {min(summary['development_bank']):.2f}° to {max(summary['development_bank']):.2f}°. Optimizer success: {r['success']}; function evaluations: {r['nfev']}. These curves determine whether sustained turns are recovered; optimizer termination alone is not validation. Thrust still comes from measured future RPM/inferred inflow, and no production simulator is changed.</p></section>'''
if 'id="curved-panel-fit"' not in h:
 h=h.replace('</body>',section+'\n</body>') if '</body>' in h else h.replace('</html>',section+'\n</html>');p.write_text(h,encoding='utf-8')

else:
 import re
 h=re.sub(r'<section id="curved-panel-fit">.*?</section>',lambda _:section,h,flags=re.S);p.write_text(h,encoding='utf-8')


