# AP_FLAKE8_CLEAN
from study import *
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
OUT=HERE.parents[1]/'docs/assets/rigging-refinement';OUT.mkdir(parents=True,exist_ok=True)
plt.rcParams.update({'font.size':11,'axes.spines.top':False,'axes.spines.right':False})
rows=json.loads((HERE/'evaluation.json').read_text());rows=[r for r in rows if r['tuning']==[1,1,1]]+json.loads((HERE/'evaluation-final.json').read_text())
configs=[('legacy',[1,1,1],'Legacy'),('geometry',[1,1,1],'132 mm only'),('routed',[1,1,1],'132 mm + routing'),('legacy',[2.5,1.15,.0743/.066],'Legacy retuned'),('routed',[2.5,1.,.0743/.066],'Routed retuned')]
summary=[]
for v,c,label in configs:
 for split in ['fit','check']:
  for alpha in [.25,.35,.45]:
   r=[x for x in rows if x['variant']==v and x['tuning']==c and x['alpha']==alpha and x['start'] in FLIGHTS[x['flight']][1][0 if split=='fit' else 1::2]]
   summary.append(dict(label=label,split=split,alpha=alpha,count=len(r),metrics={k:float(np.mean([x['metrics'][k] for x in r])) for k in r[0]['metrics']}))
(HERE/'summary.json').write_text(json.dumps(summary,indent=2))
fig,axs=plt.subplots(2,3,figsize=(13,7),layout='constrained')
for ax,k,title in zip(axs.flat,['roll_rate','pitch_rate','yaw_rate','bank','course','climb'],['Roll rate (deg/s)','Pitch rate (deg/s)','Yaw rate (deg/s)','Bank (deg)','Ground course (deg)','Climb (m/s)']):
 r=[x for x in summary if x['split']=='check' and x['alpha']==.35]
 ax.bar(np.arange(5),[x['metrics'][k] for x in r],color=['#8a98a4','#c4a54c','#508ab4','#9b73b9','#24978a']);ax.set_title(title);ax.set_xticks(range(5),[x['label'] for x in r],rotation=27,ha='right');ax.set_ylabel('Mean per-window RMSE');ax.grid(axis='y',alpha=.2)
fig.suptitle('Thirty-second replay: eight check windows, two flights\nSame logs and initialization; smaller bars are better',fontsize=16);fig.savefig(OUT/'replay-errors.png',dpi=160);plt.close(fig)
data=json.loads((HERE.parents[1]/'viewer/assets/recordings/rigging-comparison/data.json').read_text())
fig,axs=plt.subplots(2,2,figsize=(12,8),layout='constrained');colors=['#247ba0','#24978a','#b47d20','#9b559a']
for name,color in zip(['current','wide','wide_flat','wide_drag'],colors):
 case={'current':'hardware','wide':'hardware','wide_flat':'flattening','wide_drag':'drag'}[name]+'-turns';family='baseline' if name=='current' else 'direct'
 r=next(x for x in data['runs'] if x['case']==case and x['family']==family);t=np.array(r['trace']);p=np.array(json.loads((HERE.parents[1]/'viewer/assets/recordings/rigging-comparison/poses'/f'{case}-truth-{family}.json').read_text()))
 label={'current':'132 mm routed','wide':'185 mm / same canopy','wide_flat':'185 mm / +53 mm projected span','wide_drag':'185 mm / flattening + drag hypothesis'}[name]
 axs[0,0].plot(p[:,2],p[:,1],label=label,color=color);axs[0,1].plot(t[:,0],t[:,3],color=color);axs[1,0].plot(t[:,0],t[:,1],color=color);axs[1,1].plot(t[:,0],t[:,4],color=color)
axs[0,0].set(xlabel='East (m)',ylabel='North (m)',title='Top-down path');axs[0,0].axis('equal');axs[0,0].legend(fontsize=9)
for ax,title,y in [(axs[0,1],'Heading response','Course change (deg)'),(axs[1,0],'Vertical response','Height from start (m)'),(axs[1,1],'Brake routing response','Mean effective pull (mm)')]:
 ax.set(xlabel='Time (s)',ylabel=y,title=title);ax.axvspan(10,20,alpha=.1,color='gray');ax.axvspan(40,50,alpha=.1,color='gray')
for ax in axs.flat:ax.grid(alpha=.2)
fig.suptitle('Predicted response to +5 mm right brake, then +5 mm left brake\nIndividual level trim; no feedback controller; shaded intervals are the inputs',fontsize=14);fig.savefig(OUT/'predicted-turns.png',dpi=160)
print(json.dumps([x for x in summary if x['split']=='check'],indent=2))
