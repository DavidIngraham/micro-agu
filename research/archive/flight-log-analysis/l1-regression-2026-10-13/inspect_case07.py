import os

from pathlib import Path
import json,numpy as np
from scipy.spatial.transform import Rotation
from pymavlink import mavutil
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
r=Path(os.environ.get('PG_RESEARCH_ROOT', '/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/l1-regression-2026-10-13'))
fig,axs=plt.subplots(3,1,figsize=(10,8),sharex=True)
for period,d in [(8,r.parent/'landing-refinement-2026-10-13/validation-07'),(10,r/'landing10-07'),(12,r/'landing-07')]:
 l=mavutil.mavlink_connection(str(d/'flight.BIN'));pg=[];flare=None
 while True:
  m=l.recv_match(type=['PGYR','PGFL'])
  if m is None:break
  if m.get_type()=='PGYR':pg.append([m.TimeUS/1e6,m.Tgt,m.Out])
  elif m.Active and flare is None:flare=m.TimeUS/1e6
 fs=[f for f in json.loads((d/'viewer.json').read_text()) if flare-20<f['time_s']<flare];t=np.array([f['time_s'] for f in fs]);s=np.array([f['state'] for f in fs])
 roll=Rotation.from_quat(s[:,3:7]).as_euler('xyz',degrees=True)[:,0];pg=np.array(pg);target=np.interp(t,pg[:,0],pg[:,1]);command=np.interp(t,pg[:,0],pg[:,2])
 for ax,z in zip(axs,[roll,target,command]):ax.plot(t-flare,z,label=f'{period} s',color={8:'#d67b42',10:'#168b9a',12:'#888888'}[period])
for ax,y in zip(axs,['Payload roll (deg)','Target heading rate (deg/s)','Brake command']):ax.set_ylabel(y);ax.grid(alpha=.2);ax.legend()
axs[-1].set_xlabel('Seconds before flare');fig.tight_layout();fig.savefig(r/'case07-response.png',dpi=140)
