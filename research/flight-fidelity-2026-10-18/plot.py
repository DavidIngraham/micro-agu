"""Plot frozen flight and model trajectories; no fitting."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
OUT=Path(__file__).resolve().parent
b=np.load(OUT/'baseline-hood-river-turn-0.35.npz');q=np.load(OUT/'yaw-probe-hood-river-turn-0.35.npz');t=b['time'];obs=b['observed']
fig,ax=plt.subplots(5,1,figsize=(11,12),sharex=True,layout='constrained')
colors=['#203e49','#287d79','#c1683a'];labels=['Flight','Current model','Direct-yaw probe (not accepted)']
series=[(obs[:,0]*180/np.pi,obs[:,6:9]),(Rotation.from_quat(b['modeled_state'][:,3:7]).as_euler('xyz')[:,0]*180/np.pi,b['modeled_imu_velocity']),(Rotation.from_quat(q['modeled_state'][:,3:7]).as_euler('xyz')[:,0]*180/np.pi,q['modeled_imu_velocity'])]
for (bank,v),color,label in zip(series,colors,labels):
 course=np.rad2deg(np.unwrap(np.arctan2(v[:,1],v[:,0])));course-=course[0]
 for i,y in enumerate([bank,course,-v[:,2]]):ax[i].plot(t,y,color=color,label=label,lw=1.7)
ax[0].legend(loc='upper left',fontsize=9);ax[0].set_title('Hood River: recorded-input replay with 9 seconds of lead-in\nA yaw-moment probe recovers much of the turn, but misses the climb',loc='left',fontsize=14)
ax[3].plot(t,b['commands'][:,1],label='Left pull',color='#7957a4');ax[3].plot(t,b['commands'][:,2],label='Right pull',color='#be5c2f');ax[3].legend(fontsize=9)
ax[4].plot(t,b['commands'][:,0]*100,color='#203e49')
for a,label in zip(ax,['Payload bank (deg)','Course rotation since 438 s (deg)','Climb at IMU (m/s)','Commanded drum travel (mm)','Throttle (%)']):
 a.set_ylabel(label);a.axvspan(447,456,alpha=.09,color='#397abd');a.grid(alpha=.2);a.set_xlim(438,468)
ax[-1].set_xlabel('Log time (s); shaded interval is the powered-turn benchmark')
fig.savefig(OUT/'hood-river-replay.png',dpi=150)
