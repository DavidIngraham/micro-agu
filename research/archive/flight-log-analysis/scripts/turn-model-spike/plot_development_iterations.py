from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
O=Path(__file__).parent
plt.rcParams.update({'font.size':11,'axes.spines.top':False,'axes.spines.right':False,'axes.grid':True,'grid.alpha':.18,'figure.facecolor':'#fafcfe','axes.facecolor':'#fafcfe'})
fig,ax=plt.subplots(2,3,figsize=(14,7),layout='constrained')
for col,(day,start,title) in enumerate([('2026-01-03',447,'Hood River · entering the tight turn'),('2026-01-03',450,'Hood River · continuing the turn'),('2026-02-15',3387,'Trout Lake · late manual turn')]):
 a=np.load(O/f'{day}-{start}-reanchored-two-body.npz');t=a['t']-a['t'][0];truth=a['truth'];pred=Rotation.from_quat(a['states'][:,3:7]).as_euler('xyz');pred[:,2]=np.unwrap(pred[:,2]);pred[:,2]+=2*np.pi*round((truth[0,2]-pred[0,2])/(2*np.pi))
 old=np.load(O/f'{day}-{start}-Filtered-default.npz');to=old['t']-old['t'][0]
 for row,j in [(0,0),(1,2)]:
  offset=truth[0,2] if j==2 else 0
  ax[row,col].plot(t,np.rad2deg(truth[:,j]-offset),color='#172f40',lw=2.5,label='Flight')
  ax[row,col].plot(to,np.rad2deg(old['model'][:,j]-(old['truth'][0,j] if j==2 else 0)),color='#c87945',lw=1.8,label='Rigid default')
  ax[row,col].plot(t,np.rad2deg(pred[:,j]-offset),color='#178389',lw=2,label='Two-body candidate')
 ax[0,col].set_title(title);ax[1,col].set_xlabel('Prediction time (s)')
ax[0,0].set_ylabel('Payload bank (deg)');ax[1,0].set_ylabel('Heading change (deg)');ax[0,0].legend(loc='upper left',fontsize=9)
fig.suptitle('The candidate captures turn entry—but does not sustain the measured turn\nMeasured payload state at start; canopy state estimated from preceding two seconds',fontsize=15)
fig.savefig(O/'iteration-two-body-comparison.png',dpi=160);plt.close(fig)
rows=json.loads((O/'near-level-straight-windows.json').read_text());corrected=json.loads((O/'near-level-straight-windows-localwind.json').read_text());winds=json.loads((O/'straight-flight-wind-diagnostic.json').read_text())
fig,ax=plt.subplots(1,2,figsize=(12,4.5),layout='constrained');t=np.array([r['start'] for r in rows])/60
ax[0].scatter(t,[r['airspeed_proxy'] for r in rows],s=25,color='#c87945',label='One assumed wind');ax[0].scatter(t,[r['airspeed_proxy'] for r in corrected],s=25,color='#178389',label='Separate phase wind estimates');ax[0].axvline(2500/60,color='#718795',ls='--',lw=1);ax[0].set(xlabel='Boot time (min)',ylabel='Estimated airspeed (m/s)',title='Trout Lake · near-level straight windows');ax[0].legend(fontsize=9)
for name,color in [('early','#c87945'),('late','#178389')]:
 w=winds[name]['wind_NE'];ax[1].quiver(0,0,w[1],w[0],angles='xy',scale_units='xy',scale=1,color=color,width=.012,label=name.capitalize())
ax[1].set(xlim=(-1.5,.5),ylim=(-.5,.8),xlabel='East component (m/s)',ylabel='North component (m/s)',title='Wind inferred from heading and velocity');ax[1].set_aspect('equal');ax[1].legend()
fig.suptitle('A fixed wind can masquerade as an aerodynamic change',fontsize=15);fig.savefig(O/'iteration-wind-diagnostic.png',dpi=160);plt.close(fig)
force=json.loads((O/'force-tilt-diagnostic.json').read_text());fig,ax=plt.subplots(figsize=(10,4.5),layout='constrained');x=np.arange(len(force));ax.bar(x-.18,[r['payload_bank_mean_deg'] for r in force],.36,label='Payload bank',color='#172f40');ax.bar(x+.18,[r['force_tilt_mean_deg'] for r in force],.36,label='Acceleration-derived force tilt',color='#178389');ax.set_xticks(x,[('HR' if r['day'].endswith('03') else 'TL')+' '+str(r['start'])+' s' for r in force]);ax.set_ylabel('Three-second mean angle (deg)');ax.legend();ax.set_title('Measured bank broadly follows the net force direction\nForce tilt is not a direct measurement of canopy attitude');fig.savefig(O/'iteration-force-tilt.png',dpi=160)
p=O/'physics-development.html';s=p.read_text(encoding='utf-8-sig');section='''<section id="progress-plots"><h2>Latest plots</h2><figure><img src="iteration-two-body-comparison.png" alt="Measured bank and heading compared with rigid and two-body simulations"><figcaption>The shared joint-aerodynamic candidate improves turn entry, but continuation and late-flight heading still fail. Payload state is reset to the measured state at prediction start; relative canopy state comes only from the two-second lead-in. The rigid reference also starts from measured state. This figure uses the reanchored joint-aerodynamic fit, not the later rejected hidden-rate variant.</figcaption></figure><figure><img src="iteration-wind-diagnostic.png" alt="Estimated airspeed under fixed and phase-specific wind assumptions"><figcaption>The early/late wind fits assume negligible average sideslip. Their difference can also contain heading or sideslip bias. Later observations estimate the later wind, so the corrected values illustrate sensitivity rather than constitute an independent validation.</figcaption></figure><figure><img src="iteration-force-tilt.png" alt="Payload bank compared with acceleration-derived force tilt"><figcaption>Mean bank and force tilt across the key three-second windows. Net non-gravitational acceleration includes thrust and drag; its direction is not canopy orientation. These data constrain how much independent roll freedom the next model should need.</figcaption></figure></section>'''
if '<section id="progress-plots">' in s:
 start=s.index('<section id="progress-plots">');end=s.index('</section>',start)+len('</section>');s=s[:start]+section+s[end:]
else:s=s.replace('<h2>New evidence</h2>',section+'<h2>New evidence</h2>')
p.write_text(s)
print('Saved three figures and embedded them near the top of the development log')
