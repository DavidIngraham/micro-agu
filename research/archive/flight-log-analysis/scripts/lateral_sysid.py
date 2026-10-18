from pathlib import Path
import json,csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path('/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/longitudinal-2026-10-09'); O=R/'system-identification'
ledger=[]
for d in sorted(R.glob('2026-*')):
 if not d.is_dir() or not (d/'PARM.json').exists():continue
 last={}
 for x in json.loads((d/'PARM.json').read_text()):
  k=x['Name'];v=round(x['Value'],6)
  if not k.startswith(('RLL','NAVL1','WP_RADIUS','WP_LOITER','ROLL_LIMIT')):continue
  if k not in last or last[k]!=v:ledger.append([d.name,x['TimeUS']/1e6,k,last.get(k),v]);last[k]=v
with (O/'lateral-parameter-ledger.csv').open('w') as f:
 w=csv.writer(f);w.writerow(['flight','boot_seconds','parameter','previous','value']);w.writerows(ledger)
d=R/'2026-02-15 15-38-50'
c=json.loads((d/'CTUN.json').read_text()); n=json.loads((d/'NTUN.json').read_text());a=json.loads((d/'ATT.json').read_text())
t=np.array([x['TimeUS']/1e6 for x in c]);roll=np.array([x['Roll'] for x in c]);des=np.array([x['NavRoll'] for x in c])
nt=np.array([x['TimeUS']/1e6 for x in n]);xt=np.array([x['XT'] for x in n])
at=np.array([x['TimeUS']/1e6 for x in a]);yaw=np.rad2deg(np.unwrap(np.deg2rad([x['Yaw'] for x in a])))
# Symmetric 2-second difference measures heading change rather than body yaw rate.
yr=(np.interp(t+1,at,yaw)-np.interp(t-1,at,yaw))/2
windows=[('60 m LOITER',780,825),('40 m LOITER',850,900),('40 m LOITER later',950,1045),('Late AUTO',2745,3350)]
rows=[]
for label,lo,hi in windows:
 m=(t>=lo)&(t<hi);z=(nt>=lo)&(nt<hi)
 rows.append(dict(label=label,start=lo,end=hi,roll_rmse=float(np.sqrt(np.mean((roll[m]-des[m])**2))),mean_abs_roll=float(np.mean(abs(roll[m]))),mean_abs_command=float(np.mean(abs(des[m]))),heading_rate_median_abs=float(np.median(abs(yr[m]))),cross_track_rms=float(np.sqrt(np.mean(xt[z]**2))),command_at_30deg_pct=float(100*np.mean(abs(des[m])>=29.5))))
(O/'lateral-response.json').write_text(json.dumps(rows,indent=2))
fig,ax=plt.subplots(3,1,figsize=(11,8),sharex=True)
m=(t>=760)&(t<=1050);z=(nt>=760)&(nt<=1050)
x=(t[m]-202.81)/60
ax[0].plot(x,des[m],label='Commanded bank',lw=1);ax[0].plot(x,roll[m],label='Measured bank',lw=1);ax[0].set_ylabel('Bank (deg)');ax[0].legend(loc='upper right')
ax[1].plot(x,yr[m],lw=1);ax[1].set_ylabel('Heading rate (deg/s)')
ax[2].plot((nt[z]-202.81)/60,xt[z],lw=1);ax[2].set_ylabel('Cross-track error (m)');ax[2].set_xlabel('Minutes after takeoff')
for v in ax:
 v.axvline((828.69-202.81)/60,color='black',ls='--',lw=1);v.axvspan((903.754-202.81)/60,(926.153-202.81)/60,color='grey',alpha=.2);v.grid(alpha=.2)
ax[0].set_title('Trout Lake: loiter radius 60 → 40 m; roll gains unchanged')
ax[1].text((830-202.81)/60,.95,'Radius set to 40 m',transform=ax[1].get_xaxis_transform(),va='top')
fig.tight_layout();fig.savefig(O/'lateral-loiter-response.png',dpi=160)
print(json.dumps(rows,indent=2))
