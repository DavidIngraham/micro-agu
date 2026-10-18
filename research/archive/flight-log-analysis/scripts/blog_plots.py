from pathlib import Path
import json,gzip,csv,shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path('/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/longitudinal-2026-10-09')
O=Path('/home/david/repos/ArduPilot-tinkering/docs/assets/flight-testing');O.mkdir(exist_ok=True)
plt.rcParams.update({'font.size':11,'axes.spines.top':False,'axes.spines.right':False})
for name,slug,title,start,end in [('2026-01-03 15-41-42','hood-river','Hood River — January 3, 2026',298.44,1073.56),('2026-02-15 15-38-50','trout-lake','Trout Lake — February 15, 2026',202.81,3474.77)]:
 a=dict(np.load(R/name/'aligned.npz'));m=(a['t']>=start)&(a['t']<=end);t=(a['t'][m]-start)/60
 fig,ax=plt.subplots(3,1,figsize=(12,8),sharex=True,layout='constrained')
 for x,key,label,col in zip(ax,['h','pitch','thr'],['Height above home (m)','Payload pitch (deg)','Throttle (%)'],['#1769aa','#9d4929','#20744c']):
  x.plot(t,a[key][m],lw=.8,color=col);x.set_ylabel(label);x.grid(alpha=.2)
  auto=a['mode'][m]==10;edges=np.flatnonzero(np.diff(np.r_[False,auto,False]))
  for l,h in zip(edges[::2],edges[1::2]):x.axvspan(t[l],t[h-1],color='#4f965f',alpha=.12)
 ax[-1].set_xlabel('Minutes from start of displayed airborne segment');fig.suptitle(title+'\nGreen shading: AUTO mode',fontsize=16)
 fig.savefig(O/(slug+'.png'),dpi=145);plt.close(fig)
 with gzip.open(O/(slug+'-plot-data.csv.gz'),'wt') as f:
  w=csv.writer(f);w.writerow(['boot_s','height_m','pitch_deg','throttle_pct','mode'])
  for i in np.flatnonzero(m)[::5]:w.writerow([round(float(a[k][i]),5) for k in ['t','h','pitch','thr','mode']])
# Successful late AUTO segment, with navigation path expressed relative to its own start.
name='2026-02-15 15-38-50';a=dict(np.load(R/name/'aligned.npz'));m=(a['t']>=2738.09)&(a['t']<=3360.73);t=a['t'][m];te=json.loads((R/name/'TECS.json').read_text());ek=json.loads((R/name/'XKF1.json').read_text());ek=[x for x in ek if x.get('C',0)==0 and t[0]<=x['TimeUS']/1e6<=t[-1]]
e=np.array([x['PE'] for x in ek]);n=np.array([x['PN'] for x in ek]);e-=e[0];n-=n[0]
fig=plt.figure(figsize=(12,7.5),layout='constrained');gs=fig.add_gridspec(2,2);path=fig.add_subplot(gs[:,0]);hax=fig.add_subplot(gs[0,1]);uax=fig.add_subplot(gs[1,1])
path.plot(e,n,lw=1,color='#1769aa');path.plot(e[0],n[0],'o',color='#20744c',label='Segment start');path.set_aspect('equal',adjustable='datalim');path.set_xlabel('East from segment start (m)');path.set_ylabel('North from segment start (m)');path.legend()
tr=np.array([x['TimeUS']/1e6 for x in te]);hh=np.array([x['h'] for x in te]);hd=np.array([x['hdem'] for x in te]);sel=(tr>=t[0])&(tr<=t[-1]);hax.plot((tr[sel]-t[0])/60,hh[sel],label='TECS height');hax.plot((tr[sel]-t[0])/60,hd[sel],'--',label='Demand');hax.set_ylabel('Height (m)');hax.legend();uax.plot((t-t[0])/60,a['thr'][m],color='#20744c');uax.set_ylabel('Throttle (%)')
for x in [hax,uax]:x.set_xlabel('Minutes into AUTO segment')
for x in [path,hax,uax]:x.grid(alpha=.2)
fig.suptitle('Trout Lake: more than ten minutes in AUTO\nFebruary 15, 2026 · boot time 2738–3361 s',fontsize=16);fig.savefig(O/'trout-lake-auto.png',dpi=145);plt.close(fig)
# Matched-setting windows, keep the actual time ordering visible.
a=dict(np.load(R/name/'aligned.npz'));sel=(a['t']>=635)&(a['t']<=681);tt=a['t'][sel];fig,ax=plt.subplots(2,1,figsize=(12,6),sharex=True,layout='constrained')
ax[0].plot(tt,a['q'][sel],color='#9d4929',lw=.85);ax[0].set_ylabel('Pitch rate (deg/s)');ax[1].plot(tt,a['thr'][sel],color='#20744c',lw=.85);ax[1].set_ylabel('Throttle (%)');ax[1].set_xlabel('Autopilot boot time (s)')
for x in ax:x.axvline(655.794567,color='black',ls='--');x.grid(alpha=.2)
ax[0].text(656.5,ax[0].get_ylim()[1]*.8,'Damper set to zero',fontsize=11);fig.suptitle('Disabling the damper reduced the fast oscillation\nCRUISE mode; throttle P = I = 0 throughout',fontsize=16);fig.savefig(O/'damper-off.png',dpi=145);plt.close(fig)
summary={'source_logs':[{k:x[k] for k in ['file','sha256']} for x in json.loads((R/'inventory.json').read_text()) if x['file'] in ['2026-01-03 15-41-42.bin','2026-02-15 15-38-50.bin']], 'comparison':json.loads((R/'model-comparison.json').read_text())['windows'][:2], 'auto_segment':{'start_boot_s':2738.09,'end_boot_s':3360.73,'duration_min':(3360.73-2738.09)/60},'provenance':'Launch and ballast history supplied by the pilot. Log measurements do not encode ballast placement or mass. Overview plot CSVs are sampled at 5 Hz from the 25 Hz aligned analysis; rate metrics were calculated at 25 Hz.'}
(O/'flight-evidence.json').write_text(json.dumps(summary,indent=2))
print('Created four flight plots and evidence metadata')
