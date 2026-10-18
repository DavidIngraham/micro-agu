from pathlib import Path
import json,numpy as np
from scipy.spatial.transform import Rotation as Rot
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent;rows=json.loads((R/'yaw-atlas-results.json').read_text());out=[];fig,axs=plt.subplots(2,2,figsize=(12,8))
for di,day in enumerate(['2026-01-03','2026-02-15']):
 for name in ['constant','mass_linear','geometry']:
  rs=[r for r in rows if r['day']==day and r['variant']==name];rel=[];dif=[]
  for row in rs:
   y=np.load(R/f"yaw-atlas-{day}-{int(row['start'])}-{name}.npz")['state'];rr=Rot.from_quat(y[:,3:7]).inv()*Rot.from_quat(y[:,16:20]);e=rr.as_rotvec();q=rr.as_quat();twist=2*np.arctan2(q[:,2],q[:,3]);twist=np.angle(np.exp(1j*twist));rel.extend(np.rad2deg(e[125:,2]));dif.extend(np.rad2deg(np.angle(np.exp(1j*(twist[125:]-e[125:,2])))))
  out.append(dict(day=day,variant=name,count=len(rs),course_E=float(np.mean([r['course_E_deg_s'] for r in rs])),bank_rmse=float(np.mean([r['bank_rmse_deg'] for r in rs])),relative_logyaw_abs_p95_deg=float(np.percentile(abs(np.array(rel)),95)),twist_vs_logyaw_abs_p95_deg=float(np.percentile(abs(np.array(dif)),95))))
  axs[di,0].plot([r['start'] for r in rs],[r['course_E_deg_s'] for r in rs],'.-',label=name);axs[di,1].hist(rel,bins=80,histtype='step',density=True,label=name)
 axs[di,0].set(title=day+' course drift',xlabel='Log time (s)',ylabel='deg/s equivalent');axs[di,1].set(title=day+' relative log-yaw after first 5 seconds',xlabel='Degrees',ylabel='Density')
 for a in axs[di]:a.grid(alpha=.2);a.legend(fontsize=8)
fig.tight_layout();fig.savefig(R/'yaw-comparison.png',dpi=160);(R/'yaw-summary.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
