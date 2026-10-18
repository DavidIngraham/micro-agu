from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
O=Path(__file__).parent;rows=json.loads((O/'bounded-results.json').read_text());assert len(rows)==36
summary=[]
for key in sorted(set((r['day'],r['case'],r['lead_s']) for r in rows)):
 group=[r for r in rows if (r['day'],r['case'],r['lead_s'])==key];best=min(group,key=lambda r:r['cost']);near=[r for r in group if r['cost']<=1.1*best['cost']];b=best['baseline'];f=best['forecast'];posspread=max(np.linalg.norm(np.array(a['forecast']['endpoint_position'])-c['forecast']['endpoint_position']) for a in near for c in near);cs=max(abs(a['forecast']['endpoint_course_deg']-c['forecast']['endpoint_course_deg']) for a in near for c in near)
 gates=dict(position=f['position_rmse_m']<=.8*b['position_rmse_m'],course=f['course_rmse_deg']<=.8*b['course_rmse_deg'],bank=f['bank_rmse_deg']<=1.1*max(1,b['bank_rmse_deg']),speed=f['speed_rmse_m_s']<=1.1*max(.1,b['speed_rmse_m_s']),climb=f['climb_rmse_m_s']<=1.1*max(.1,b['climb_rmse_m_s']),spread=posspread<=5 and cs<=15)
 summary.append(dict(day=key[0],case=key[1],lead_s=key[2],selected_start=best['start'],gates=gates,pass_all=all(gates.values()),position_spread_m=float(posspread),course_spread_deg=float(cs),baseline=b,forecast=f))
(O/'bounded-decision.json').write_text(json.dumps(dict(status='fixed matrix completed',fits=len(rows),groups=summary,decision='STOP: no further observer/coefficient tuning in this goal; diagnose missing physical or observation information'),indent=2))
fig,ax=plt.subplots(3,2,figsize=(13,11))
for col,day in enumerate(['2026-01-03','2026-02-15']):
 for lead,color in [(5,'tab:blue'),(10,'tab:orange')]:
  row=next(r for r in summary if r['day']==day and r['case']=='nominal' and r['lead_s']==lead);tag=f"{day}-nominal-{lead}";z=np.load(O/f"bounded-{tag}-{row['selected_start']}.npz");b=np.load(O/f'bounded-{tag}-baseline.npz');i=round(lead/.04);t=z['t'][i:];y=z['states'][i:];truth=z['truth'][i:];pos=z['logged_position'][i:];pos-=pos[0]
  ax[0,col].plot(y[:,1]-y[0,1],y[:,0]-y[0,0],color=color,label=f'{lead}s observer');yb=b['states'][i:];ax[0,col].plot(yb[:,1]-yb[0,1],yb[:,0]-yb[0,0],color=color,ls='--',alpha=.6,label=f'{lead}s heuristic')
  course=np.unwrap(np.arctan2(y[:,8],y[:,7]));ct=np.unwrap(np.arctan2(truth[:,7],truth[:,6]));course-=2*np.pi*np.round((course[0]-ct[0])/(2*np.pi));ax[1,col].plot(t,np.rad2deg(course),color=color)
  ax[2,col].plot(t,np.rad2deg(R.from_quat(y[:,3:7]).as_euler('xyz')[:,0]),color=color)
 ax[0,col].plot(pos[:,1],pos[:,0],'k',lw=2,label='logged');ax[0,col].set(xlabel='East displacement (m)',ylabel='North displacement (m)',title=day);ax[0,col].axis('equal');ax[0,col].legend(fontsize=8)
 ax[1,col].plot(t,np.rad2deg(ct),'k',lw=2);ax[1,col].set(ylabel='Unwrapped ground course (deg)',xlabel='Forecast time (s)')
 ax[2,col].plot(t,np.rad2deg(truth[:,0]),'k',lw=2);ax[2,col].set(ylabel='Payload bank (deg)',xlabel='Forecast time (s)')
for a in ax.ravel():a.grid(alpha=.2)
fig.suptitle('Fixed causal forecast test — nominal geometry, frozen aerodynamics');fig.tight_layout();fig.savefig(O/'bounded-forecast.png',dpi=160)
print(json.dumps([dict(day=r['day'],case=r['case'],lead=r['lead_s'],passed=r['pass_all'],gates=r['gates']) for r in summary],indent=2))
