from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
O=Path(__file__).parent
rows=json.loads((O/'propulsion-body-ablation.json').read_text())['validation'];oracle=json.loads((O/'oracle-canopy-state-diagnostic.json').read_text())['results']
fig,axes=plt.subplots(2,3,figsize=(13,7),sharex=True)
for ax,row in zip(axes.flat,oracle):
 d,s=row['day'],row['start'];truth=None
 for tag,label,color in [('propulsion-linear-body','Old thrust','#9babb6'),('propulsion-rpm_9x45-body','Corrected thrust','#2086a3'),('oracle-canopy-state','Initial state fitted to future','#d58431')]:
  a=np.load(O/f'{d}-{s}-{tag}.npz');ax.plot(a['t'],np.rad2deg(a['model'][:,0]),label=label,color=color);truth=a['truth']
 ax.plot(a['t'],np.rad2deg(truth[:,0]),label='Logged bank',color='#15232e',linewidth=2)
 ax.set_title(('HR' if d.endswith('03') else 'TL')+f' {s} s');ax.grid(alpha=.2);ax.set_ylabel('Payload bank (deg)')
for ax in axes[-1]:ax.set_xlabel('Time (s)')
axes[0,0].legend(fontsize=7)
fig.suptitle('Propulsion correction alone does not recover the observed turns')
fig.text(.5,.015,'Orange uses future flight data to fit hidden initial states: diagnostic only, not predictive validation.',ha='center',fontsize=10)
fig.tight_layout(rect=(0,.045,1,.95));fig.savefig(O/'iteration-propulsion-state-diagnostic.png',dpi=150)
p=O/'physics-development.html';h=p.read_text(encoding='utf-8');section='''<section id="propulsion-state-ablation"><h2>Separating thrust error from hidden canopy-state error</h2><p>With the balanced aerodynamic and joint coefficients frozen, replacing linear throttle-to-thrust with the corrected 9×4.5 RPM-based thrust estimate changes three-second bank RMSE by at most 1.41°. Hood River at 450 s remains at 20.75°, versus 20.87° before. The large turn mismatch survives the propulsion correction.</p><img src="iteration-propulsion-state-diagnostic.png" alt="Measured bank and predictions with original thrust, corrected thrust, and future-fitted hidden initial canopy states" style="max-width:100%"><p>A second diagnostic gives the model an advantage unavailable in simulation: fitting six hidden initial canopy attitude/rate components to the complete future three-second trajectory. This reduces bank RMSE to 4.15–12.20°, but five of the six cases reach at least one orientation-adjustment bound (±0.5 rad per rotation-vector component). Several require large angular-rate changes. These adjustments are not estimates of the real canopy motion.</p><p>The results show sensitivity to hidden initial state, but this bounded local fit still does not reproduce the demanding turns well enough. They do not prove that every possible initialization fails. Both diagnostics prescribe future measured RPM and inferred inflow; neither is a free-running propulsion validation. Current geometry, joint torques, and aerodynamic structure still require investigation. No production simulator changes were made.</p></section>'''
if 'id="propulsion-state-ablation"' not in h:
 h=h.replace('</body>',section+'\n</body>') if '</body>' in h else h.replace('</html>',section+'\n</html>');p.write_text(h,encoding='utf-8')
