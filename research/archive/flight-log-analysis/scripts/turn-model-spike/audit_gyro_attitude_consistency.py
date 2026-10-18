from pathlib import Path
import json,numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
O=Path(__file__).parent;cases=[('2026-01-03',447),('2026-02-15',3421)];fig,axes=plt.subplots(2,2,figsize=(12,6));rows=[]
for row,(d,start) in enumerate(cases):
 a=np.load(O/(d+'-causal-input.npz'));i=np.searchsorted(a['t'],start);y=a['Y'][i:i+76];t=a['t'][i:i+76];R=Rotation.from_euler('xyz',y[:,:3]);kin=(R[:-1].inv()*R[1:]).as_rotvec()/.04;gyro=y[:-1,3:6];pred=[R[0]]
 for w in gyro:pred.append(pred[-1]*Rotation.from_rotvec(w*.04))
 err=(Rotation.concatenate(pred).inv()*R).magnitude();rows.append(dict(day=d,start=start,gyro_peak_deg_s=np.rad2deg(np.max(abs(gyro),axis=0)).tolist(),attitude_increment_peak_deg_s=np.rad2deg(np.max(abs(kin),axis=0)).tolist(),three_second_integrated_gyro_attitude_error_deg=float(np.rad2deg(err[-1]))))
 ax=axes[row,0];ax.plot(t[:-1]-t[0],np.rad2deg(gyro[:,2]),label='Latest logged gyro Z',alpha=.7);ax.plot(t[:-1]-t[0],np.rad2deg(kin[:,2]),label='ATT relative rotation / dt',alpha=.8);ax.set_title(f'{d} at {start} s');ax.set_ylabel('Body yaw rate (deg/s)');ax.legend(fontsize=8)
 ax=axes[row,1];ax.plot(t-t[0],np.rad2deg(err));ax.set_ylabel('Orientation disagreement (deg)');ax.set_title('Integrating logged gyro from initial attitude')
for ax in axes.flat:ax.grid(alpha=.2);ax.set_xlabel('Time (s)')
fig.tight_layout();fig.savefig(O/'iteration-gyro-attitude-consistency.png',dpi=150);(O/'gyro-attitude-consistency.json').write_text(json.dumps(dict(scope='Sampling diagnostic: latest-sample 25Hz gyro integration versus EKF attitude, not sensor calibration or independent ground truth; ATT increments also affected by sample holding.',results=rows),indent=2));print(json.dumps(rows,indent=2))
p=O/'physics-development.html';h=p.read_text(encoding='utf-8');section='''<section id="input-support-and-gyro"><h2>Input coverage and angular-rate consistency</h2><p>The six development turns overlap the training set in bank/rate range; some combinations of brake input and inferred speed are sparse. A simple claim that all difficult turns are outside the training range is unsupported.</p><img src="iteration-turn-input-coverage.png" alt="Training and development control and motion coverage" style="max-width:100%"><p>The coverage audit also found large instantaneous gyro peaks in some training windows. The following check integrates the 25 Hz latest-sample gyro stream and compares it with EKF attitude, alongside rates inferred from adjacent attitude rotations. Neither is independent ground truth. Disagreement can reflect sampling, filtering, timing, and estimator correction; it is not automatically evidence of actual canopy oscillation.</p><img src="iteration-gyro-attitude-consistency.png" alt="Gyro versus attitude-derived yaw rates and integrated orientation disagreement" style="max-width:100%"><p>This identifies angular-rate preprocessing as an unresolved contributor to initial-state uncertainty. No flight-model acceptance follows from the coverage audit.</p></section>'''
if 'id="input-support-and-gyro"' not in h:
 h=h.replace('</body>',section+'\n</body>') if '</body>' in h else h.replace('</html>',section+'\n</html>');p.write_text(h,encoding='utf-8')
