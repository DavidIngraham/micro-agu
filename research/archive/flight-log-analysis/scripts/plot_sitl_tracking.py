from pathlib import Path
import json,datetime
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
O=Path('/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/longitudinal-2026-10-09/system-identification')
a=json.loads((O/'sitl-selected-attitude.json').read_text());n=[x for x in json.loads((O/'sitl-selected-navigation.json').read_text()) if x['mavpackettype']=='NAV_CONTROLLER_OUTPUT'];print(n[0])
fig,ax=plt.subplots(2,2,figsize=(12,6),layout='constrained');out=[]
for col,(start,title) in enumerate([('11:30:00','FF 1.0 · D_FF 0.05'),('11:35:15','FF 2.0 · D_FF 0.10')]):
 lo=datetime.datetime.fromisoformat('2026-02-15T'+start+'-08:00').timestamp();hi=lo+45
 aa=[x for x in a if lo<=x['t']<hi];nn=[x for x in n if lo<=x['t']<hi];t=np.array([x['t'] for x in aa]);r=np.rad2deg([x['roll'] for x in aa]);nt=np.array([x['t'] for x in nn]);nr=np.array([x['nav_roll'] for x in nn])
 ax[0,col].plot(nt-lo,nr,label='Commanded bank',lw=1.3);ax[0,col].plot(t-lo,r,label='Simulated bank',lw=1);ax[0,col].set_title(start+' PST — '+title);ax[0,col].set_ylabel('Bank (deg)');ax[0,col].legend(fontsize=9)
 ax[1,col].plot(nt-lo,[x['xtrack_error'] for x in nn]);ax[1,col].set_ylabel('Cross-track error (m)');ax[1,col].set_xlabel('Seconds from window start')
 out.append(dict(start=start,duration_s=45,bank_error_rms_deg=float(np.sqrt(np.mean((r-np.interp(t,nt,nr))**2)))))
for row in ax:
 lim=(min(x.get_ylim()[0] for x in row),max(x.get_ylim()[1] for x in row))
 for x in row:x.set_ylim(lim);x.grid(alpha=.2)
fig.suptitle('Recorded AUTO simulation runs: target tracking at two gain settings')
fig.savefig(O/'sitl-bank-tracking.png',dpi=160);(O/'sitl-tracking-windows.json').write_text(json.dumps(out,indent=2));print(out)
