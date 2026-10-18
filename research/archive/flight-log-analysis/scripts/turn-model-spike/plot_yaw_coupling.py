from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
O=Path(__file__).parent;r=json.loads((O/'yaw-coupling-grid.json').read_text());rows=r['rows'];fig,ax=plt.subplots(figsize=(8,4))
for damp in [.1,1,10]:
 rr=sorted([v for v in rows if v['factors'][1]==damp],key=lambda v:v['factors'][0]);ax.semilogx([v['factors'][0]*.05 for v in rr],[v['mean_bank'] for v in rr],marker='o',label=f'Yaw damping {damp*.02:g} N m s/rad')
ax.set(xlabel='Yaw stiffness (N m/rad)',ylabel='Mean bank RMSE over six turns (deg)',title='Yaw coupling alone does not recover sustained bank');ax.legend();ax.grid(alpha=.2);fig.tight_layout();fig.savefig(O/'iteration-yaw-coupling.png',dpi=150)
p=O/'physics-development.html';h=p.read_text(encoding='utf-8');section='''<section id="yaw-coupling-sensitivity"><h2>Checking the previously fixed yaw stiffness</h2><p>The previous suspension grid did not vary yaw stiffness. This follow-up spans 0.005, 0.05, 0.5, and 5 N·m/rad, with yaw damping 0.002, 0.02, and 0.2 N·m·s/rad. Aerodynamic coefficients remain frozen and the conservative spring is used.</p><img src="iteration-yaw-coupling.png" alt="Mean bank error versus yaw stiffness at three yaw damping values" style="max-width:100%"><p>The best tested mean bank RMSE is 12.63°, versus 12.93° at the baseline; the worst turn still has 19.80° bank RMSE. This rules out a simple rescue by these yaw constants over the tested grid, not every possible coupled refit or suspension geometry. In particular, agreement of measured gyro and attitude oscillations does not by itself identify canopy-to-payload torsional motion: canopy orientation is unmeasured. The model remains unsuitable for controller validation.</p></section>'''
if 'id="yaw-coupling-sensitivity"' not in h:
 h=h.replace('</body>',section+'\n</body>') if '</body>' in h else h.replace('</html>',section+'\n</html>');p.write_text(h,encoding='utf-8')
