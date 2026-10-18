from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path('/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/longitudinal-2026-10-09')
O=R/'endurance';O.mkdir(exist_ok=True)
result={}
fig,axes=plt.subplots(3,1,figsize=(11,9),sharex=True,layout='constrained')
for name,start,end in [('2026-01-03 15-41-42',298.44,1073.56),('2026-02-15 15-38-50',202.81,3474.77)]:
 b=json.loads((R/name/'BAT.json').read_text());b=[x for x in b if x['Inst']==0];t=np.array([x['TimeUS']/1e6 for x in b]);f={k:np.array([x[k] for x in b]) for k in ['Volt','Curr','CurrTot','EnrgTot','RemPct']}
 def stats(lo,hi):
  tt=np.r_[lo,t[(t>lo)&(t<hi)],hi];v=np.interp(tt,t,f['Volt']);i=np.interp(tt,t,f['Curr']);dur=hi-lo
  return dict(start_boot_s=lo,end_boot_s=hi,duration_min=dur/60,charge_Ah=float(np.diff(np.interp([lo,hi],t,f['CurrTot']))[0]/1000),energy_Wh=float(np.diff(np.interp([lo,hi],t,f['EnrgTot']))[0]),integrated_Ah=float(np.trapz(i,tt)/3600),integrated_Wh=float(np.trapz(v*i,tt)/3600),mean_A=float(np.trapz(i,tt)/dur),mean_W=float(np.trapz(v*i,tt)/dur),voltage_start_end=np.interp([lo,hi],t,f['Volt']).tolist(),voltage_min=float(v.min()),current_p50_p95_max=np.percentile(i,[50,95,100]).tolist(),seconds_above_2400W=float(np.trapz((v*i>2400).astype(float),tt)),last_10s_voltage_median=float(np.median(v[tt>hi-10])))
 result[name]={'airborne':stats(start,end),'whole_log':stats(t[0],t[-1]),'battery_start_end':{k:[float(v[0]),float(v[-1])] for k,v in f.items()}}
 if '02-15' in name:
  result[name]['late_auto']=stats(2738.09,3360.73)
  result[name]['last_5min']=stats(end-300,end)
  m=(t>=start)&(t<=end);x=(t[m]-start)/60
  axes[0].plot(x,f['Volt'][m],lw=.8);axes[0].set_ylabel('Pack voltage (V)')
  axes[1].plot(x,f['Curr'][m],lw=.5,alpha=.7);axes[1].set_ylabel('Battery current (A)')
  axes[2].plot(x,(f['EnrgTot'][m]-np.interp(start,t,f['EnrgTot'])),color='#20744c');axes[2].set_ylabel('Energy since segment start (Wh)')
  for ax in axes:ax.axvspan((2738.09-start)/60,(3360.73-start)/60,color='green',alpha=.1);ax.grid(alpha=.2)
for ax in axes:ax.spines[['top','right']].set_visible(False)
axes[-1].set_xlabel('Flight time (minutes)')
fig.suptitle('Trout Lake: 54.5 minutes, with capacity to spare\nLanded after testing · Green: late AUTO segment')
fig.savefig(O/'trout-lake-endurance.png',dpi=160)
(O/'metrics.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
