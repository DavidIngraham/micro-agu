from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
O=Path(__file__).parent;r=json.loads((O/'curved-added-grid.json').read_text())['rows'];scales=sorted(set(v['scale'] for v in r));fig,axes=plt.subplots(1,2,figsize=(10,4))
for ax,key,label in [(axes[0],'bank','Mean bank RMSE (deg)'),(axes[1],'heading','Mean absolute final heading error (deg)')]:
 values=[np.mean([abs(v[key]) for v in r if v['scale']==s]) for s in scales];ax.plot(scales,values,'o-',color='#267e9c');ax.set(xlabel='Apparent-inertia template scale',ylabel=label);ax.grid(alpha=.2)
fig.suptitle('Apparent inertia with curved-panel loads: frozen-coefficient diagnostic');fig.tight_layout();fig.savefig(O/'iteration-curved-added-inertia.png',dpi=150)
p=O/'physics-development.html';h=p.read_text(encoding='utf-8');section='''<section id="curved-added-inertia"><h2>Combining panel loads and spatial apparent inertia</h2><p>The curved-panel solver now includes the earlier spatial apparent-inertia formulation with the conservative suspension spring. Sixty randomized checks give zero-added-inertia derivative parity within 2.3×10⁻¹³ and uniform velocity/wind boost invariance within 1.2×10⁻¹³.</p><img src="iteration-curved-added-inertia.png" alt="Bank and heading errors versus apparent inertia scale" style="max-width:100%"><p>The provisional template uses translational added masses (0.01, 0.1, 0.5) kg, rotational terms (0.01, 0.005, 0.005) kg m², and axis-dependent centers at z = 0 and 0.5 m. A shared multiplier scales the six mass/inertia values. These are not measured canopy properties.</p><p>With coefficients frozen, scale 0.3 reduces mean development bank RMSE from 7.66° to 7.28°. Scale 1 reduces mean absolute final heading error from 41.62° to 32.07°, with mean bank RMSE 7.37°. Scale 2 performs poorly; step convergence there has not been checked. These results support a joint refit as the next test, not acceptance of the template or the full model. Propulsion remains conditioned on measured RPM/inflow.</p></section>'''
if 'id="curved-added-inertia"' not in h:h=h.replace('</body>',section+'\n</body>');p.write_text(h,encoding='utf-8')
