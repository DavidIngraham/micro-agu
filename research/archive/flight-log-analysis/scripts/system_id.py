from pathlib import Path
import numpy as np,json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path('/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/longitudinal-2026-10-09');O=R/'system-identification';O.mkdir(exist_ok=True)
allrows=[]
for name,start,end in [('2026-01-03 15-41-42',298,1074),('2026-02-15 15-38-50',203,760)]:
 a=dict(np.load(R/name/'aligned.npz'));fig,ax=plt.subplots(3,1,figsize=(13,8),sharex=True,layout='constrained');m=(a['t']>=start)&(a['t']<=end)
 for x,k,lab in zip(ax,['thr','climb','h'],['Throttle (%)','Climb rate (m/s)','Height above home (m)']):x.plot(a['t'][m],a[k][m],lw=.7);x.set_ylabel(lab);x.grid(alpha=.2)
 ax[1].axhline(0,color='gray',lw=.8);ax[-1].set_xlabel('Seconds after boot');fig.suptitle(name+' — identification maneuvers')
 if '02-15' in name:
  for t,label in [(312.81,'Max throttle 40→65'),(320.61,'Trim throttle 30→50')]:ax[0].axvline(t,color='red',ls='--');ax[0].text(t,ax[0].get_ylim()[1],label,rotation=90,va='top',fontsize=8)
 fig.savefig(O/(name[:10]+'-identification.png'),dpi=150);plt.close(fig)
 print('\n'+name)
 for t in range(int(start//10)*10,int(end),10):
  m=(a['t']>=t)&(a['t']<t+10)&np.isfinite(a['climb']+a['thr']+a['h'])
  if sum(m)<200:continue
  row={'log':name,'start':t,'end':t+10,'mode':int(np.median(a['mode'][m])),'throttle_mean':round(float(np.mean(a['thr'][m])),2),'throttle_sd':round(float(np.std(a['thr'][m])),2),'climb_mean':round(float(np.mean(a['climb'][m])),3),'height_slope':round(float(np.polyfit(a['t'][m],a['h'][m],1)[0]),3),'roll_rms':round(float(np.sqrt(np.mean(a['roll'][m]**2))),1)};allrows.append(row)
  if '02-15' in name and t<340 or row['throttle_mean']<10 or row['throttle_sd']<1.5:print(row)
(O/'ten-second-windows.json').write_text(json.dumps(allrows,indent=2))
