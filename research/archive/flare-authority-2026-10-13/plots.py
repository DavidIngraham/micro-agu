from study import OUT,RAW
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams.update({'figure.dpi':140,'axes.grid':True,'grid.alpha':.25})
r=json.loads((OUT/'screen-results.json').read_text())+json.loads((OUT/'refine-results.json').read_text())
rob=json.loads((OUT/'robust-results.json').read_text())
fig,axs=plt.subplots(1,2,figsize=(12,5),layout='constrained')
for entry,ax in zip(['glide','release'],axs):
 for rate,c in [(30,'C0'),(100,'C1'),(150,'C2'),(200,'C3'),(300,'C4')]:
  a=[x for x in r if x['entry']==entry and x['rate']==rate and x['pull']==95]
  ax.scatter([x['horizontal'] for x in a],[x['sink'] for x in a],c=c,label=f'{rate} pp/s',s=30)
 base=next(x for x in rob if x['entry']==entry and x['setting']=='current' and x['condition']=='nominal')
 fast=next(x for x in rob if x['entry']==entry and x['setting']=='faster' and x['condition']=='nominal')
 for x,label in [(base,'Current: 1.1 m, 100 pp/s'),(fast,'Candidate: 1.0 m, 150 pp/s')]:
  ax.scatter(x['horizontal'],x['sink'],s=100,facecolors='none',edgecolors='black');ax.annotate(label,(x['horizontal'],x['sink']),xytext=(5,15 if x==base else -25),textcoords='offset points',fontsize=8)
 ax.set(title=entry.title()+' entry — 95% common brake',xlabel='Touchdown horizontal speed (m/s)',ylabel='Touchdown sink (m/s)');ax.legend(fontsize=8)
fig.suptitle('Reducing forward speed can increase touchdown sink');fig.savefig(OUT/'tradeoff.png');plt.close(fig)
fig,axs=plt.subplots(2,3,figsize=(13,8),layout='constrained')
for entry,axes in zip(['glide','release'],axs):
 for label,c in [('current','C0'),('earlier','C1'),('faster','C2')]:
  x=next(x for x in rob if x['entry']==entry and x['setting']==label and x['condition']=='nominal');a=np.load(RAW/(x['name']+'.npz'));t=a['t'];y=a['state']
  axes[0].plot(t,-y[:,2],c=c,label=label);axes[1].plot(t,np.linalg.norm(y[:,7:9],axis=1),c=c);axes[2].plot(t,y[:,9],c=c)
 for ax,title in zip(axes,['Height (m)','Horizontal speed (m/s)','Sink (m/s)']):ax.set(xlabel='Time after flare (s)',ylabel=title,title=entry.title())
 axes[0].legend();axes[0].axhline(0,c='k',lw=.7)
fig.suptitle('Flare response ends at first payload CG ground crossing');fig.savefig(OUT/'flare-traces.png');plt.close(fig)
fig,axs=plt.subplots(2,2,figsize=(12,8),layout='constrained')
conditions=['nominal','weaker','pessimistic','gust0','gust2','gust4','early_error','late_error'];labels=['Nominal','CL×.75/CD×1.25','CL×.5/CD×1.5','Gust phase 0','Gust phase 2','Gust phase 4','Trigger +0.2 m','Trigger −0.2 m']
for entry,axes in zip(['glide','release'],axs):
 for setting,c in [('current','C0'),('faster','C2')]:
  a=[next(x for x in rob if x['entry']==entry and x['setting']==setting and x['condition']==condition) for condition in conditions]
  for ax,key in zip(axes,['horizontal','sink']):ax.plot(labels,[x[key] for x in a],marker='o',c=c,label=setting);ax.tick_params(axis='x',rotation=40)
 axes[0].set(title=entry.title()+' entry',ylabel='Horizontal speed (m/s)');axes[1].set(title=entry.title()+' entry',ylabel='Sink (m/s)');axes[0].legend()
fig.suptitle('Faster flare: modest benefit, sensitive to onset and entry state');fig.savefig(OUT/'robustness.png');plt.close(fig)
