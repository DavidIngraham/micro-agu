from pathlib import Path
import json,numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import fast_napkin_causal as fast
from napkin_geometry import configuration
O=Path(__file__).parent;p=json.loads((O/'curved-added-fit.json').read_text())['parameters'];rows=json.loads((O/'glide-mechanics-results.json').read_text());out=[];fig,axs=plt.subplots(2,2,figsize=(12,7))
for col,day in enumerate(['2026-01-03','2026-02-15']):
 r=min([r for r in rows if r['day']==day],key=lambda r:r['cost']);cut=r['cutoff_s'];z=np.load(O/f"glide-mechanics-{day}-{int(cut)}-nominal-5-{r['start']}.npz");g=configuration(day);pars=p|g['parameters'];pred=[];rates=[]
 for y,u,rpm in zip(z['states'],z['controls'],z['rpm']):
  rot=R.from_quat(y[3:7]);arm=rot.apply(g['imu']);w=y[10:13];ctrl=u.copy();ctrl[0]=fast.lib.mapped_thrust(rpm,float(rot.inv().apply(y[7:10]-np.array(r['wind']))[0]))/p['thrust_max'];dy=fast.derivative(y,ctrl,pars,g['mass'],np.array(r['wind']));pred.append(dy[7:10]+np.cross(dy[10:13],arm)+np.cross(w,np.cross(w,arm))-np.array([0,0,9.80665]));rates.append(rot.inv().apply(w))
 pred=np.array(pred);rates=np.array(rates);sl=z['t']>=0;forceerr=pred[sl]-z['force'][sl];rateerr=rates[sl]-z['truth'][sl,3:6]
 out.append(dict(day=day,specific_force_component_RMSE_NED_m_s2=np.sqrt(np.mean(forceerr**2,axis=0)).tolist(),body_rate_component_RMSE_deg_s=np.rad2deg(np.sqrt(np.mean(rateerr**2,axis=0))).tolist(),specific_force_mean_error_NED_m_s2=np.mean(forceerr,axis=0).tolist()))
 for k,label in enumerate(['North','East','Down']):axs[0,col].plot(z['t'][sl],forceerr[:,k],label=label)
 for k,label in enumerate(['Roll','Pitch','Yaw']):axs[1,col].plot(z['t'][sl],np.rad2deg(rateerr[:,k]),label=label)
 axs[0,col].set_title(day)
 for row in range(2):axs[row,col].grid(alpha=.2);axs[row,col].legend(fontsize=8);axs[row,col].set_xlabel('Forecast time (s)')
 axs[0,col].set_ylabel('Specific-force error (m/s²)');axs[1,col].set_ylabel('Body-rate error (deg/s)')
fig.suptitle('Low-power mechanics: sensor-point force and gyro residuals');fig.tight_layout();fig.savefig(O/'glide-force-gyro.png',dpi=160);(O/'glide-force-gyro.json').write_text(json.dumps(dict(scope='Forecast residuals, including IMU lever-arm tangential and centripetal acceleration. No model refit; logged force and gyro are observations.',results=out),indent=2));print(json.dumps(out,indent=2))
