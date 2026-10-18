from pathlib import Path
import json,numpy as np
from scipy.spatial.transform import Rotation
from two_body import initial_state
from parafoil_two_body import Parafoil
import fast_conservative_joint as old,fast_split_brake as new
O=Path(__file__).parent;rng=np.random.default_rng(143);errors=[]
for k in range(40):
 Rp=Rotation.from_rotvec(rng.normal(size=3)*.2).as_matrix();Rc=Rotation.from_rotvec(rng.normal(size=3)*.2).as_matrix();y=initial_state(Parafoil(1.758),Rp,Rc,[6.,0.,0.],rng.normal(size=3)*.1,rng.normal(size=3)*.1);brake=rng.uniform(0,.05);u=np.array([.4,brake,brake]);p=dict(Cldelta=0.,Cndelta=0.,brake_lever_fraction=.25);errors.append(float(np.max(abs(old.derivative(y,u,p,1.758,[0,0,0])-new.derivative(y,u,p,1.758,[0,0,0])))))
assert max(errors)<1e-10
(O/'split-brake-mechanics-check.json').write_text(json.dumps(dict(symmetric_input_derivative_parity_max=max(errors),cases=40),indent=2))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
r=json.loads((O/'split-brake-fit.json').read_text());prev=json.loads((O/'brake-loading-fit.json').read_text());fig,ax=plt.subplots(figsize=(9,4));x=np.arange(6)
a=[v['bank_rmse_deg'] for v in prev['validation'] if v['group']=='development' and v['variant']=='fitted'];b=[v['bank_rmse_deg'] for v in r['validation'] if v['group']=='development' and v['variant']=='fitted']
ax.bar(x-.18,a,.36,label='Independent steering moments');ax.bar(x+.18,b,.36,label='Split brake forces');ax.set_xticks(x,['HR447','HR450','HR508','TL326','TL402','TL3387']);ax.set_ylabel('Bank RMSE (deg)');ax.set_title('Simple split-brake force model does not improve the turn fit');ax.legend();fig.tight_layout();fig.savefig(O/'iteration-split-brake.png',dpi=150)
p=O/'physics-development.html';h=p.read_text(encoding='utf-8');section='''<section id="split-brake-model"><h2>Steering moments from separated brake forces</h2><p>A new research variant places incremental brake forces at y = ±ℓ. Let bL and bR be engaged pull divided by 72 mm, m = (bL+bR)/2, and d = bR−bL. Total brake drag is proportional to (bL²+bR²)/2 = m²+d²/4, while the force difference is ΔF = qS/2 [−CDbrake(2md)e − CLbrake d f(α)n]. The steering moment is (0,ℓ,0) × ΔF, with f(α) the existing lift attenuation. Independent brake roll/yaw moment terms are zeroed to avoid double-counting.</p><p>This is our simplified planar force hypothesis, not a reproduction of a published curved-canopy model. <a href="https://digitalcommons.georgefox.edu/mece_fac/3/">Slegers and Costello (2003)</a> describe competing roll/skid responses and sensitivity to canopy curvature, incidence, and brake magnitude. That motivates checking force placement, but does not validate the present geometry.</p><img src="iteration-split-brake.png" alt="Bank error comparison for independent steering moments and separated brake forces" style="max-width:100%"><p>Forty symmetric-input derivative comparisons match the preceding solver to 10⁻¹⁰. Fitting brake lift, brake drag, and ℓ/b on the same 62 windows pushes brake lift essentially to zero and ℓ/b to 0.039. Bank RMSE on development turns is 5.95–22.56°, and median error on 254 excluded internal comparison windows is 3.25°, worse than the preceding 2.52°. No candidate is accepted. Boundary-seeking coefficients indicate this simplified force placement does not capture the observed steering with the other coefficients frozen; they are not measured canopy properties.</p></section>'''
if 'id="split-brake-model"' not in h:
 h=h.replace('</body>',section+'\n</body>') if '</body>' in h else h.replace('</html>',section+'\n</html>');p.write_text(h,encoding='utf-8')
print('symmetric parity',max(errors))
