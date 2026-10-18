from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter1d
O=Path(__file__).parent;dec=json.loads((O/'bounded-decision.json').read_text());fig,axes=plt.subplots(2,2,figsize=(12,7));rows=[]
for col,(day,mass) in enumerate([('2026-01-03',1.385),('2026-02-15',1.758)]):
 row=next(r for r in dec['groups'] if r['day']==day and r['case']=='nominal' and r['lead_s']==5);z=np.load(O/f"bounded-{day}-nominal-5-{row['selected_start']}.npz");sl=z['t']>=0;t=z['t'][sl];v=z['truth'][sl,6:9];vm=z['states'][sl,7:10];right=np.c_[-v[:,1],v[:,0],np.zeros(len(v))];right/=np.linalg.norm(right,axis=1)[:,None];am=np.gradient(vm,.04,axis=0);a=np.gradient(v,.04,axis=0);lateral=np.sum(a*right,axis=1);model=np.sum(am*right,axis=1);imu=np.sum(z['force'][sl]*right,axis=1)
 for val,label,color in [(lateral,'Logged velocity derivative','black'),(model,'Model, projected on logged track','tab:blue'),(imu,'Logged IMU specific force','tab:orange')]:axes[0,col].plot(t,gaussian_filter1d(val,2),label=label,color=color)
 axes[1,col].plot(t,gaussian_filter1d(mass*lateral,2),'k',label='AUW × logged acceleration');axes[1,col].plot(t,gaussian_filter1d(mass*model,2),color='tab:blue',label='AUW × model acceleration')
 interior=slice(5,-5);rows.append(dict(day=day,mean_signed_lateral_accel_logged=float(lateral[interior].mean()),mean_signed_lateral_accel_model=float(model[interior].mean()),lateral_acceleration_error_rms=float(np.sqrt(np.mean((model[interior]-lateral[interior])**2))),imu_vs_velocity_lateral_rms=float(np.sqrt(np.mean((imu[interior]-lateral[interior])**2))),mass_scaled_acceleration_gap_rms_N=float(mass*np.sqrt(np.mean((model[interior]-lateral[interior])**2)))))
 axes[0,col].set(title=day,ylabel='Ground-track lateral acceleration (m/s²)');axes[1,col].set(ylabel='Mass-scaled acceleration proxy (N)',xlabel='Forecast time (s)')
for ax in axes.ravel():ax.grid(alpha=.2);ax.legend(fontsize=7)
fig.suptitle('Two-turn force audit: payload-path acceleration, not measured canopy force');fig.tight_layout();fig.savefig(O/'turn-force-audit.png',dpi=150);(O/'turn-force-audit.json').write_text(json.dumps(dict(results=rows,method='Common logged ground-track right normal; no wind needed. Differentiate25Hz velocity, display Gaussian sigma .08s; summary excludes .2s edges. AUW times payload/EKF-reference acceleration is a proxy, not whole-system force or canopy lift.'),indent=2));print(json.dumps(rows,indent=2))
