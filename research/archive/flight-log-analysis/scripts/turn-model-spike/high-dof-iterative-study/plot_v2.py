from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent
fig,axs=plt.subplots(2,2,figsize=(13,8),sharex=True)
for j,flight in enumerate(['hood_river','trout_lake']):
 for gains,color in [('archived','#b04c42'),('conservative','#2476a3')]:
  d=np.load(R/f'v2-controller-{flight}-k4.8622-{gains}.npz');t=d['t'];a=d['data'];axs[0,j].plot(t,a[:,2],label=gains,color=color,lw=.7);axs[1,j].plot(t,a[:,4],color=color,lw=.7)
 axs[0,j].plot(t,a[:,0],'k--',lw=1,label='requested');axs[0,j].set_title(flight.replace('_',' ').title());axs[0,j].legend();axs[1,j].set_xlabel('Time from airborne trim (s)')
axs[0,0].set_ylabel('Payload heading rate (deg/s)');axs[1,0].set_ylabel('Normalized brake command')
fig.suptitle('Controller diagnostic on the modal candidate, including servo travel rate');fig.tight_layout();fig.savefig(R/'v2-controller-comparison.png',dpi=150)
rows=json.loads((R/'roll1-atlas-results.json').read_text());base=json.loads((R/'candidate-v2-atlas-results.json').read_text());idx={(r['day'],r['start']):r for r in base};fig,axs=plt.subplots(1,2,figsize=(12,5))
for ax,day in zip(axs,['2026-01-03','2026-02-15']):
 a=[r for r in rows if r['day']==day];x=np.array([idx[day,r['start']]['course_E_deg_s'] for r in a]);y=np.array([r['course_E_deg_s'] for r in a]);ax.scatter(x,y);lim=max(x.max(),y.max())*1.05;ax.plot([0,lim],[0,lim],'k--');ax.set(xlabel='kroll 4.86: course error (deg/s)',ylabel='kroll 1: course error (deg/s)',title=day);ax.grid(alpha=.2)
fig.suptitle('Softer roll joint: full-set trajectory tradeoff (below diagonal is better)');fig.tight_layout();fig.savefig(R/'roll-atlas-tradeoff.png',dpi=150)
