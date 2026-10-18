from pathlib import Path
import numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent;fig,ax=plt.subplots(3,2,figsize=(13,10))
for j,flight in enumerate(['hood_river','trout_lake']):
 for inertia,label in [(0.,'no spin inertia'),(6.532245e-5,'nominal prop inertia'),(1.306449e-4,'double prop inertia')]:
  d=np.load(R/f'mode-{flight}-{inertia:.3g}-bounded-y.npz');t=d['t'];v=d['response'];ax[0,j].plot(t,v[:,1],label=label);ax[1,j].plot(t,v[:,2],label=label);freq=np.fft.rfftfreq(4096,.04);power=abs(np.fft.rfft(v[:250,1:3],n=4096,axis=0))**2;ax[2,j].plot(freq,power.sum(axis=1),label=label)
 ax[0,j].set_title(flight)
 for i in range(2):ax[i,j].set_xlim(0,5);ax[i,j].set_xlabel('seconds');ax[i,j].grid(alpha=.2)
 ax[2,j].set(xlim=(.5,5),xlabel='Frequency (Hz)',ylabel='Impulse-response spectrum (relative)');ax[2,j].grid(alpha=.2)
 ax[0,j].set_ylabel('Payload pitch rate (deg/s)');ax[1,j].set_ylabel('Payload yaw rate (deg/s)')
ax[0,0].legend();fig.suptitle('One-degree relative pitch kick: propeller momentum couples pitch and yaw');fig.tight_layout();fig.savefig(R/'gyro-mode-comparison.png',dpi=150)
