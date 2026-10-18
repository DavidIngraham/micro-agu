"""Frequency response of accepted landing equilibria; no fitted transfer surrogate."""
from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
r=Path(__file__).parent;rows=json.loads((r/'envelope.json').read_text());f=np.geomspace(.05,8,500)
fig,axes=plt.subplots(2,1,figsize=(10,7),sharex=True)
for index in [0,2,3,4,8]:
 row=rows[index];d=np.load(r/f'envelope-{index}.npz');A,B,C,dt=d['A'],d['B'],d['C'],float(d['dt'])
 h=np.array([(C@np.linalg.solve(np.exp(2j*np.pi*x*dt)*np.eye(16)-A,B))[0] for x in f])
 label=f"sink {row['sink']:g} m/s, turn {row['turn']:g} deg/s, common {100*row['common']:g}%"
 axes[0].semilogx(f,20*np.log10(abs(h)),label=label);axes[1].semilogx(f,np.unwrap(np.angle(h))*180/np.pi)
axes[0].set_ylabel('Gain (dB, deg/s per brake fraction)');axes[1].set_ylabel('Phase (deg)');axes[1].set_xlabel('Frequency (Hz)');axes[0].legend(fontsize=9)
for ax in axes:ax.grid(which='both',alpha=.2)
fig.suptitle('Differential brake to heading rate changes across the landing envelope')
fig.tight_layout();fig.savefig(r/'landing-bode.png',dpi=160)
