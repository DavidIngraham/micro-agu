import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
r=Path('/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/lateral-tuning-2026-10-13');rows=[];fig,axes=plt.subplots(1,3,figsize=(13,4),sharey=True)
for ax,sink in zip(axes,[0,.5,1]):
 a=np.load(r/f'descent-linear-{sink:g}.npz');A,B,C=a['A'],a['B'],a['C'];dt=float(a['dt']);scale=5/np.linalg.norm(a['state'][7:10])
 for name,kp,ki,rd,cutoff in [('landing',.04,.002,.03,4),('mission',.005,.002,.003,4),('mission_1hz',.005,.002,.003,1)]:
  values=[]
  for delay in [0,.02,.04,.06,.08,.1,.12]:
   n=round(delay/dt);alpha=dt/(dt+1/(2*np.pi*cutoff))
   def step(x):
    filt=x[16:18]+alpha*(C@x[:16]-x[16:18]);integ=x[18]-ki*dt*filt[0];control=-kp*scale*filt[0]-rd*scale*filt[1]+integ
    plant=A@x[:16]+B*(x[-1] if n else control)
    return np.r_[plant,filt,integ,([control,*x[19:-1]] if n else [])]
   F=np.column_stack([step(e) for e in np.eye(19+n)]);eig=np.linalg.eigvals(F);eig=eig[abs(eig)>1e-12];z=np.log(eig.astype(complex))/dt
   z=z[abs(z)>1e-5];dominant=z[np.argmax(z.real)]
   values.append(dominant.real);rows.append(dict(sink=sink,controller=name,delay_s=delay,dominant_real=float(dominant.real),dominant_hz=float(abs(dominant.imag)/(2*np.pi))))
  ax.plot(np.array([0,.02,.04,.06,.08,.1,.12])*1000,values,'o-',label=name)
 ax.axhline(0,color='black',lw=1);ax.set_title(f'{sink:g} m/s sink');ax.set_xlabel('Pure command delay (ms)');ax.grid(alpha=.25)
axes[0].set_ylabel('Largest non-neutral pole real part (1/s)');axes[0].legend()
fig.suptitle('Local lateral feedback stability: above zero = growing mode')
fig.tight_layout();fig.savefig(r/'descent-delay-poles.png',dpi=150)
(r/'descent-delay-poles.json').write_text(json.dumps(rows,indent=2))
for x in rows:
 if x['delay_s'] in [0,.12]:print(x)
