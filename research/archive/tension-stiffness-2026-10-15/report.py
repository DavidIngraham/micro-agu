from run import *
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
rows=json.loads((HERE/'flight-results.json').read_text());closed=json.loads((HERE/'closed-results.json').read_text());audit=json.loads((HERE/'audit-results.json').read_text())
keys=list(rows[0]['metrics']);variants=list(VARIANTS);summary={}
for alpha in [.35,.25,.45]:
 mean={v:np.mean([[r['metrics'][k] for k in keys] for r in rows if r['alpha']==alpha and r['variant']==v],axis=0) for v in variants}
 summary[str(alpha)]={v:dict(mean_rmse=dict(zip(keys,mean[v])),change_percent=dict(zip(keys,100*(mean[v]/mean['fixed']-1)))) for v in variants}
ref=[]
for kind,original,refined,metrics in [('flight',rows,audit['flight_refinement'],keys),('controller',closed,audit['controller_refinement'],['height_rms','heading_rms','pitch_rms'])]:
 for x in refined:
  matches=[r for r in original if r['variant']==x['variant'] and (r['flight']==x['flight'] and r['start']==x['start'] and r['alpha']==x['alpha'] if kind=='flight' else r['case']==x['case'])]
  base=matches[0];a=x['metrics'] if kind=='flight' else x;b=base['metrics'] if kind=='flight' else base
  ref.append(dict(kind=kind,case=x.get('case',str(x.get('start'))),variant=x['variant'],max_metric_relative_change=max(abs(a[k]/b[k]-1) for k in metrics)))
inputs={}
for day,_ in FLIGHTS.values():
 for suffix in ['causal-input','propulsion-9x45']:
  p=DATA/f'{day}-{suffix}.npz';inputs[p.name]=hashlib.sha256(p.read_bytes()).hexdigest()
short={}
for variant in variants:
 errors=[]
 for row in rows:
  if row['alpha']==.35 and row['variant']==variant:
   z=np.load(HERE/'traces'/f"{row['flight']}-{row['start']}-0.35-{variant}-32.npz")
   errors.append(np.sqrt(np.mean(np.rad2deg(z['rates'][:126]-z['truth'][:126,3:6])**2,axis=0)))
 short[variant]=dict(zip(['roll_rate','pitch_rate','yaw_rate'],np.mean(errors,axis=0)))
(HERE/'summary.json').write_text(json.dumps(dict(aggregate=summary,first_five_seconds_rmse=short,refinement=ref,input_sha256=inputs),indent=2)+'\n')
plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
fig,ax=plt.subplots(figsize=(10,4.5),layout='constrained');x=np.arange(len(keys));width=.19
for j,v in enumerate(variants[1:]):ax.bar(x+(j-1.5)*width,list(summary['0.35'][v]['change_percent'].values()),width,label=v)
ax.set_ylim(-.8,1.0);ax.axhline(0,color='black',lw=.8);ax.set_xticks(x,['Roll rate','Pitch rate','Yaw rate','Bank','Course','Climb']);ax.set_ylabel('Change in mean window RMSE (%)');ax.set_title('Tension-dependent stiffness: small, mixed flight-replay effects');ax.legend(ncol=4);ax.text(.01,.97,'16 × 30 s windows • primary canopy initialization • lower is better',transform=ax.transAxes,va='top',fontsize=9)
fig.savefig(HERE/'flight-comparison.png',dpi=160);plt.close(fig)
fig,axs=plt.subplots(2,1,figsize=(10,7),layout='constrained')
names=list(dict.fromkeys(r['case'] for r in closed));x=np.arange(len(names))
for v in variants[1:]:
 a=[next(r for r in closed if r['case']==c and r['variant']==v) for c in names];b=[next(r for r in closed if r['case']==c and r['variant']=='fixed') for c in names]
 axs[0].plot(x,[100*(aa['heading_rms']/bb['heading_rms']-1) for aa,bb in zip(a,b)],'o-',label=v)
axs[0].set_ylabel('Turn-rate RMSE change (%)');axs[0].set_xticks(x,[n.replace('_','\n') for n in names],fontsize=8);axs[0].axhline(0,color='black',lw=.8);axs[0].legend(ncol=4);axs[0].set_title('Unchanged controller: yaw scaling slightly worsens gust rejection')
p=HERE/'traces'/'closed-strong_gust-both-16';d=np.load(p/'load.npz')['diag'];t=np.arange(len(d))*.02
axs[1].plot(t,d[:,0]/d[:,1],label='Strong-gust tension / trim tension');axs[1].axhline(1,color='black',lw=.8);axs[1].set(xlabel='Time (s)',ylabel='Geometric stiffness multiplier');axs[1].legend();fig.savefig(HERE/'controller-comparison.png',dpi=160);plt.close(fig)
# Transparent choice: largest sampled tensile load, primary initialization.
r=max((r for r in rows if r['alpha']==.35 and r['variant']=='both'),key=lambda r:r['load_quantiles'][-1]);f=r['flight'];start=r['start']
a=np.load(HERE/'traces'/f'{f}-{start}-0.35-fixed-32.npz');b=np.load(HERE/'traces'/f'{f}-{start}-0.35-both-32.npz');t=np.arange(len(a['rates']))*.04
fig,axs=plt.subplots(3,1,figsize=(10,8),layout='constrained',sharex=True)
axs[0].plot(t[::5],b['diag'][:,0]/b['diag'][:,1]);axs[0].axhline(1,color='black',lw=.8);axs[0].set_ylabel('T / T₀');axs[0].set_title(f'Largest sampled load case: {f.replace("_"," ")}, {start} s')
for j,(k,label) in enumerate([(0,'Roll rate (deg/s)'),(2,'Yaw rate (deg/s)')],1):
 axs[j].plot(t,np.rad2deg(a['truth'][:,3+k]),color='black',lw=1,label='Flight');axs[j].plot(t,np.rad2deg(a['rates'][:,k]),label='Fixed',alpha=.8);axs[j].plot(t,np.rad2deg(b['rates'][:,k]),label='Tension-dependent',alpha=.8);axs[j].set_ylabel(label)
axs[1].legend(ncol=3);axs[-1].set_xlabel('Replay time (s)');fig.savefig(HERE/'flight-trace.png',dpi=160);plt.close(fig)
print('Refinement',ref)
print('Representative',f,start)
