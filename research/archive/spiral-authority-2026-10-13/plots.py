from common import *
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
plt.rcParams.update({'figure.dpi':140,'axes.grid':True,'grid.alpha':.25})
r=json.loads((OUT/'sweep.json').read_text());sens=json.loads((OUT/'sensitivity.json').read_text());power=json.loads((OUT/'power.json').read_text())
fig,ax=plt.subplots(2,2,figsize=(12,8),layout='constrained')
for profile,ls in [('mission','-'),('dynamic_roll','--')]:
 for side,c in [('left','C0'),('right','C1')]:
  a=[x for x in r if x['profile']==profile and x['side']==side and x['entry_s']==1.5 and x['release_s']==1.5]
  x=[z['fraction']*100 for z in a]
  for axy,k in zip(ax.ravel(),['mean_canopy_bank','mean_airspeed','mean_sink','exit_peak_airspeed']):axy.plot(x,[abs(z[k]) if k=='mean_canopy_bank' else z[k] for z in a],ls,marker='o',c=c,label=profile+' '+side)
for axy,title,unit in zip(ax.ravel(),['Sustained canopy bank','Sustained airspeed','Sustained sink','Peak airspeed after release'],['deg','m/s','m/s','m/s']):axy.set(xlabel='Fraction of configured side travel (%)',ylabel=unit,title=title);axy.legend(fontsize=8)
fig.suptitle('Full brake travel still gives a moderate descending turn',fontsize=16);fig.savefig(OUT/'authority.png');plt.close(fig)
fig,ax=plt.subplots(2,2,figsize=(12,8),layout='constrained')
for profile,ls in [('mission','-'),('dynamic_roll','--')]:
 for entry,c in [(.15,'C0'),(1.5,'C1'),(4.,'C2')]:
  a=np.load(RAW/f'{profile}-right-1-{entry:g}-1.5.npz');y=a['state'];t=a['t'];bank=Rotation.from_quat(y[:,16:20]).as_euler('xyz',degrees=True)[:,0]
  ax[0,0].plot(t,bank,ls,c=c,label=f'{profile}: {entry:g} s entry');ax[0,1].plot(t,np.linalg.norm(y[:,7:10],axis=1),ls,c=c)
for side,axy in [('left',ax[1,0]),('right',ax[1,1])]:
 for profile,ls in [('mission','-'),('dynamic_roll','--')]:
  for entry,c in [(.15,'C0'),(1.5,'C1'),(4.,'C2')]:
   a=sorted([z for z in r if z['profile']==profile and z['side']==side and z['fraction']==1 and z['entry_s']==entry],key=lambda z:z['release_s'])
   axy.plot([z['release_s'] for z in a],[z['exit_peak_airspeed'] for z in a],ls,c=c,marker='o')
 axy.set(title=side.title()+' full pull: release transient',xlabel='Release ramp (s)',ylabel='Peak airspeed (m/s)')
ax[0,0].set(title='Right full pull, release at 26 s',xlabel='Time (s)',ylabel='Canopy bank (deg)');ax[0,0].legend(fontsize=7)
ax[0,1].set(title='Faster entry does not start a new spiral regime',xlabel='Time (s)',ylabel='Airspeed (m/s)')
fig.savefig(OUT/'entry-release.png');plt.close(fig)
fig,ax=plt.subplots(1,2,figsize=(13,5),layout='constrained')
rows=[x for x in power if x['name'].startswith('mission')];bottom=np.zeros(3)
for k,label in [('base_drag_W','Base canopy drag'),('induced_drag_W','Induced drag'),('brake_drag_W','Explicit brake drag'),('payload_drag_W','Payload drag')]:
 vals=[x['mean'][k] for x in rows];ax[0].bar(['Straight','Full left','Full right'],vals,bottom=bottom,label=label);bottom+=vals
vals=[-x['mean']['extra_moment_power_W'] for x in rows];ax[0].bar(['Straight','Full left','Full right'],vals,bottom=bottom,label='Extra aerodynamic moments')
ax[0].set(ylabel='Dissipation (W)',title='Steady idle energy balance');ax[0].legend(fontsize=8)
labels=[x['label'].replace('_',' ') for x in sens];a=ax[1];a.barh(labels,[x['canopy_bank'] for x in sens]);a.set(xlabel='Sustained canopy bank (deg)',title='Diagnostic ablations: same full-right command')
for i,x in enumerate(sens):a.text(x['canopy_bank']+.2,i,f"{x['airspeed']:.2f} m/s",va='center',fontsize=8)
a.set_xlim(0,31);a.invert_yaxis();fig.savefig(OUT/'mechanisms.png');plt.close(fig)

