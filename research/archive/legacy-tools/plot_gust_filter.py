"""Show the selected filter shape and the slow-mode pole movement."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('study',type=Path);a=p.parse_args()
rows=json.loads((a.study/'linear-second-order.json').read_text())
fig,axes=plt.subplots(3,1,figsize=(11,12),layout='constrained')
f=np.geomspace(.03,30,700);s=2j*np.pi*f
for hz,stages,label in [(10,1,'Historical 10 Hz stage'),(.5,1,'First candidate: 0.5 Hz stage'),(1.5,2,'Selected: two 1.5 Hz stages')]:
    h=(2*np.pi*hz/(s+2*np.pi*hz))**stages
    axes[0].semilogx(f,20*np.log10(abs(h)),label=label)
    axes[1].semilogx(f,np.unwrap(np.angle(h))*180/np.pi,label=label)
axes[0].axvline(.34,color='gray',ls=':');axes[0].set(xlabel='Frequency (Hz)',ylabel='Filter magnitude (dB)',ylim=(-65,2),title='Shape the pitch-rate feedback across frequency');axes[0].legend(fontsize=8);axes[0].grid(alpha=.25)
axes[1].set(xlabel='Frequency (Hz)',ylabel='Filter phase (deg)',title='Phase shaping matters as well as attenuation');axes[1].grid(alpha=.25)
points=[]
for gain,hz,label,color in [(0,.75,'Damper off','#3679b5'),(.1,1.5,'Two-stage candidate','#d86c22')]:
    r=next(r for r in rows if r['case']=='level' and r['airspeed_scale']==1.15 and r['gain']==gain and r['cutoff_hz']==hz)
    pole=max(r['slow_poles'],key=lambda z:z[0]);x,y=pole;points.append(pole)
    axes[2].scatter([x,x],[y,-y],marker='x',s=100,color=color,label=label)
    zeta=-x/np.hypot(x,y)
    axes[2].annotate(f'{label}\n{x:.3f} ± j{y:.3f}\nζ = {zeta:.3f}',(x,y),xytext=(x+.03,y+.45),fontsize=9,color=color)
for sign in (-1,1):axes[2].annotate('',xy=(points[1][0],sign*points[1][1]),xytext=(points[0][0],sign*points[0][1]),arrowprops=dict(arrowstyle='->',color='#666'))
axes[2].axvline(0,color='black',lw=1);axes[2].set(xlabel='Real part (1/s)',ylabel='Imaginary part (rad/s)',xlim=(-.9,.25),ylim=(-3.5,4.1),title='Slow pitch / forward-speed pole pair');axes[2].grid(alpha=.25)
fig.suptitle('Local linear design: level trim, coupled lateral loop, airspeed scaling 1.15')
fig.savefig(a.study/'filter-and-poles.png',dpi=160)
