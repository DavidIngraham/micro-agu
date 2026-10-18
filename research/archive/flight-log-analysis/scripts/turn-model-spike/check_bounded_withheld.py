from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import fast_bounded_causal as fast
from bounded_geometry import configuration,imu_specific_force
O=Path(__file__).parent;plan=json.loads((O/'bounded-diagnostic-plan.json').read_text());decision=json.loads((O/'bounded-decision.json').read_text());p=json.loads((O/'curved-added-fit.json').read_text())['parameters'];rows=[];fig,axes=plt.subplots(2,2,figsize=(12,7))
for row in decision['groups']:
 day=row['day'];lead=row['lead_s'];case=next(c for c in plan['geometry_cases'] if c['name']==row['case']);g=configuration(day,case);params=p|g['parameters'];z=np.load(O/f"bounded-{day}-{row['case']}-{lead}-{row['selected_start']}.npz");states=z['states'];truth=z['truth'];rot=R.from_quat(states[:,3:7]);gyro=rot.inv().apply(states[:,10:13]);observed_force=R.from_euler('xyz',truth[:,:3]).inv().apply(z['force']);force=[]
 wind=next(r['wind'] for r in json.loads((O/'bounded-results.json').read_text()) if r['day']==day and r['case']==row['case'] and r['lead_s']==lead)
 for y,u,rpm in zip(states,z['controls'],z['rpm']):
  u=u.copy();axial=float(R.from_quat(y[3:7]).inv().apply(y[7:10]-wind)[0]);u[0]=fast.lib.mapped_thrust(float(rpm),axial)/p['thrust_max'];dy=fast.derivative(y,u,params,g['mass'],wind);force.append(imu_specific_force(y,dy,g))
 force=np.array(force);n=round(lead/.04);record=dict(day=day,case=row['case'],lead_s=lead)
 for name,sl in [('lead_in',slice(0,n+1)),('forecast',slice(n,None))]:
  record[name]=dict(gyro_axis_rmse_deg_s=np.rad2deg(np.sqrt(np.mean((gyro[sl]-truth[sl,3:6])**2,axis=0))).tolist(),specific_force_axis_rmse_m_s2=np.sqrt(np.mean((force[sl]-observed_force[sl])**2,axis=0)).tolist(),attitude_rotation_rmse_deg=float(np.rad2deg(np.sqrt(np.mean((R.from_euler('xyz',truth[sl,:3]).inv()*rot[sl]).magnitude()**2)))),velocity_rmse_m_s=float(np.sqrt(np.mean((states[sl,7:10]-truth[sl,6:9])**2))))
 rows.append(record)
 if row['case']=='nominal' and lead==5:
  c=0 if day=='2026-01-03' else 1;t=z['t'];axes[0,c].plot(t,np.rad2deg(truth[:,5]),'k',label='logged');axes[0,c].plot(t,np.rad2deg(gyro[:,2]),label='predicted');axes[1,c].plot(t,observed_force[:,2],'k');axes[1,c].plot(t,force[:,2]);axes[0,c].set(title=day,ylabel='Body yaw rate (deg/s)');axes[1,c].set(ylabel='Body z specific force (m/s²)',xlabel='Time relative to forecast cutoff (s)')
for a in axes.ravel():a.axvline(0,color='gray',ls='--');a.grid(alpha=.2)
axes[0,0].legend();fig.suptitle('Withheld consistency channels — nominal geometry, 5-second lead-in');fig.tight_layout();fig.savefig(O/'bounded-withheld-channels.png',dpi=150);(O/'bounded-withheld-checks.json').write_text(json.dumps(rows,indent=2));print(json.dumps([r for r in rows if r['case']=='nominal'],indent=2))
