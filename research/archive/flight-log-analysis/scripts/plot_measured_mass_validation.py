from pathlib import Path
import numpy as np,json,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
O=Path('/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/longitudinal-2026-10-09/turn-model-spike')
fig,ax=plt.subplots(2,2,figsize=(12,7),layout='constrained')
for col,(day,start,name) in enumerate([('2026-01-03',447,'Hood River: high-bank circle'),('2026-02-15',3387,'Trout Lake: late manual turn')]):
 for label,color in [('Default','#dc6036'),('Fitted','#246ba0')]:
  z=np.load(O/f'{day}-{start}-{label}.npz');t=z['t']-z['t'][0];m=z['model'];y=z['truth']
  ax[0,col].plot(t,np.rad2deg(m[:,0]),color=color,label=label+' coefficients')
  ax[1,col].plot(t,np.rad2deg(m[:,2]-y[0,2]),color=color)
 ax[0,col].plot(t,np.rad2deg(y[:,0]),color='black',label='Measured')
 ax[1,col].plot(t,np.rad2deg(y[:,2]-y[0,2]),color='black')
 ax[0,col].set_title(name)
 ax[1,col].set_xlabel('Replay time (s)')
 for row in range(2):ax[row,col].grid(alpha=.2)
ax[0,0].set_ylabel('Payload bank (deg)');ax[1,0].set_ylabel('Heading change (deg)');ax[0,0].legend()
fig.suptitle('Recorded-input rigid-model replay with measured mass\nCandidate fit improves Trout Lake but fails the Hood River high-bank turn',fontsize=13)
fig.savefig(O/'measured-mass-validation.png',dpi=170)
print('Saved measured-mass-validation.png')
