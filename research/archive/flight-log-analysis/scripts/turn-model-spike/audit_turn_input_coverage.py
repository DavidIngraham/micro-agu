from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
O=Path(__file__).parent;r=json.loads((O/'balanced-body-fit.json').read_text());cache={d:np.load(O/(d+'-causal-input.npz')) for d in ['2026-01-03','2026-02-15']};rows=[]
for group,ws in [('training',r['training_windows']),('development',[('2026-01-03',447),('2026-01-03',450),('2026-01-03',508),('2026-02-15',326),('2026-02-15',402),('2026-02-15',3387)])]:
 for d,start in ws:
  a=cache[d];i=np.searchsorted(a['t'],start);sl=slice(i,i+76);y=a['Y'][sl];br=a['brake_mm'][sl];v=y[:,6:9]-a['wind'][sl];diff=br[:,1]-br[:,0];rates=np.rad2deg(y[:,3:6]);rows.append(dict(group=group,day=d,start=start,median_diff_mm=float(np.median(diff)),max_abs_diff_mm=float(np.max(abs(diff))),median_mean_brake_mm=float(np.median(br.mean(axis=1))),median_speed_m_s=float(np.median(np.linalg.norm(v,axis=1))),max_bank_deg=float(np.rad2deg(np.max(abs(y[:,0])))),median_yaw_rate_deg_s=float(np.median(rates[:,2])),max_yaw_rate_deg_s=float(np.max(abs(rates[:,2]))),median_roll_rate_deg_s=float(np.median(rates[:,0]))))
(O/'turn-input-coverage.json').write_text(json.dumps(rows,indent=2));print(json.dumps([r for r in rows if r['group']=='development'],indent=2))
fig,axes=plt.subplots(1,2,figsize=(12,5))
for ax,xkey,ykey,xlabel,ylabel in [(axes[0],'median_diff_mm','median_speed_m_s','Median right − left line pull (mm)','Median inferred airspeed (m/s)'),(axes[1],'max_bank_deg','max_yaw_rate_deg_s','Maximum payload bank magnitude (deg)','Maximum body yaw-rate magnitude (deg/s)')]:
 for d,color,label in [('2026-01-03','#297c9d','HR training'),('2026-02-15','#8aacb7','TL training')]:
  rr=[r for r in rows if r['group']=='training' and r['day']==d];ax.scatter([r[xkey] for r in rr],[r[ykey] for r in rr],label=label,color=color,alpha=.7)
 for row in [r for r in rows if r['group']=='development']:
  ax.scatter(row[xkey],row[ykey],marker='x',color='#d97628',s=80);ax.annotate(('HR' if row['day'].endswith('03') else 'TL')+str(row['start']),(row[xkey],row[ykey]),fontsize=8,xytext=(4,4),textcoords='offset points')
 ax.set(xlabel=xlabel,ylabel=ylabel);ax.grid(alpha=.2)
axes[0].legend();fig.suptitle('Are the difficult turns represented in the fitting data?');fig.tight_layout();fig.savefig(O/'iteration-turn-input-coverage.png',dpi=150)
