from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import fast_napkin_causal as fast
O=Path(__file__).parent;p=json.loads((O/'curved-added-fit.json').read_text())['parameters'];plan=json.loads((O/'net-improvement-plan.json').read_text());coverage=[]
for day,cut in plan['broader']:
 a=np.load(O/(day+'-causal-input.npz'));m=(a['t']>=cut)&(a['t']<=cut+10);y=a['Y'][m];course=np.unwrap(np.arctan2(y[:,7],y[:,6]));coverage.append(dict(day=day,cutoff=cut,bank_min_deg=float(np.rad2deg(y[:,0].min())),bank_max_deg=float(np.rad2deg(y[:,0].max())),course_change_deg=float(np.rad2deg(course[-1]-course[0])),brake_min_mm=a['brake_mm'][m].min(axis=0).tolist(),brake_max_mm=a['brake_mm'][m].max(axis=0).tolist()))
fig,ax=plt.subplots(4,2,figsize=(12,11),constrained_layout=True);res=json.loads((O/'sensor-napkin-results.json').read_text());summary=[]
for col,day in enumerate(['2026-01-03','2026-02-15']):
 r=min([r for r in res if r['day']==day],key=lambda r:r['cost']);a=np.load(O/f"sensor-napkin-{day}-nominal-5-{r['start']}.npz");y=a['states'];t=a['t'];wind=np.array(r['wind']);rc=R.from_quat(y[:,16:20]);vb=rc.inv().apply(y[:,20:23]-wind);alpha=np.rad2deg(np.arctan2(vb[:,2],vb[:,0]));euler=rc.as_euler('xyz');vb_p=y[:,7:10]-wind;truth=a['truth'];course=np.unwrap(np.arctan2(vb_p[:,1],vb_p[:,0]));cl=(p['CL0']+p['CLa']*np.deg2rad(alpha))/(1+(abs(np.deg2rad(alpha))/.6)**4)
 ax[0,col].plot(t,alpha,label='Canopy center alpha');ax[0,col].set(ylabel='Alpha (deg)',title='Hood River' if col==0 else 'Trout Lake')
 ax[1,col].plot(t,np.rad2deg(euler[:,0]),label='Sim canopy');ax[1,col].plot(t,np.rad2deg(R.from_quat(y[:,3:7]).as_euler('xyz')[:,0]),label='Sim payload');ax[1,col].plot(t,np.rad2deg(truth[:,0]),label='Logged payload',color='k');ax[1,col].set(ylabel='Bank (deg)');ax[1,col].legend()
 ax[2,col].plot(t,a['controls'][:,1]*1000,label='Left');ax[2,col].plot(t,a['controls'][:,2]*1000,label='Right');ax[2,col].set(ylabel='Engaged brake (mm)');ax[2,col].legend()
 ax[3,col].plot(t,np.linalg.norm(vb,axis=1),label='Sim canopy');ax[3,col].plot(t,np.linalg.norm(truth[:,6:9]-wind,axis=1),label='Logged payload');ax[3,col].set(ylabel='Air-relative speed (m/s)',xlabel='Time from cutoff (s)');ax[3,col].legend()
 for ar in ax[:,col]:ar.axvline(0,color='gray',linestyle=':');ar.grid(alpha=.25)
 summary.append(dict(day=day,alpha_cutoff_deg=float(alpha[np.searchsorted(t,0)]),alpha_forecast_range=[float(alpha[t>=0].min()),float(alpha[t>=0].max())]))
fig.savefig(O/'napkin-canopy-response.png',dpi=150);(O/'napkin-response-audit.json').write_text(json.dumps(dict(coverage=coverage,canopy=summary),indent=2));print(json.dumps(dict(coverage=coverage,canopy=summary),indent=2))
