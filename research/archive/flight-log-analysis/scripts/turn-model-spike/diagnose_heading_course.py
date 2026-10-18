from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
O=Path(__file__).parent;fit=json.loads((O/'panel-only-lateral-fit.json').read_text());rows=[];fig,axes=plt.subplots(2,3,figsize=(13,7),sharex=True)
def relative(x):return np.rad2deg(np.unwrap(x)-x[0])
def wrap(x):return (x+np.pi)%(2*np.pi)-np.pi
for ax,row in zip(axes.flat,[r for r in fit['validation'] if r['group']=='development']):
 d,s=row['day'],row['start'];a=np.load(O/f'{d}-{s}-panel-only-lateral.npz');inp=np.load(O/(d+'-causal-input.npz'));i=np.searchsorted(inp['t'],s);wind=inp['wind'][i];obs=a['truth'];pred=a['model'];vo=obs[:,3:]-wind;vp=pred[:,3:]-wind;co=np.arctan2(vo[:,1],vo[:,0]);cp=np.arctan2(vp[:,1],vp[:,0]);err=wrap(cp-co);slipo=wrap(obs[:,2]-co);slipp=wrap(pred[:,2]-cp)
 rows.append(dict(day=d,start=s,heading_final_error_deg=row['heading_final_error_deg'],course_final_error_deg=float(np.rad2deg(err[-1])),course_rmse_deg=float(np.rad2deg(np.sqrt(np.mean(err**2)))),observed_heading_minus_course_start_end_deg=np.rad2deg(slipo[[0,-1]]).tolist(),predicted_heading_minus_course_start_end_deg=np.rad2deg(slipp[[0,-1]]).tolist()))
 ax.plot(a['t'],relative(obs[:,2]),color='#252a30',label='Logged heading');ax.plot(a['t'],relative(pred[:,2]),color='#c9842c',label='Model heading');ax.plot(a['t'],relative(co),color='#252a30',ls='--',label='Inferred air course');ax.plot(a['t'],relative(cp),color='#277f9d',ls='--',label='Model air course');ax.set_title(('HR' if d.endswith('03') else 'TL')+f' {s} s');ax.set_ylabel('Change from initial angle (deg)');ax.grid(alpha=.2)
axes[0,0].legend(fontsize=7)
for ax in axes[-1]:ax.set_xlabel('Time (s)')
fig.suptitle('Does the heading mismatch also affect the flight path?');fig.tight_layout();fig.savefig(O/'iteration-heading-course-separation.png',dpi=150);(O/'heading-course-separation.json').write_text(json.dumps(dict(scope='Panel-only fit; air-relative course inferred using assumed wind, no airspeed sensor; payload heading minus course is not measured canopy sideslip.',results=rows),indent=2));print(json.dumps(rows,indent=2))
p=O/'physics-development.html';h=p.read_text(encoding='utf-8');section='''<section id="heading-course-separation"><h2>Separating payload heading from flight-path turning</h2><p>Heading and air-relative course are now scored separately for the panel-only lateral model. Course is computed from horizontal velocity minus the assumed wind; it is not an independent airspeed measurement. Payload heading minus course is not canopy sideslip, since canopy heading is unmeasured.</p><img src="iteration-heading-course-separation.png" alt="Payload heading and inferred air-relative course changes for measured and simulated turn windows" style="max-width:100%"><p>Solid lines show heading and dashed lines course, each referenced to its own initial angle. The companion heading-course-separation.json records absolute final-angle and course RMSE errors. This distinguishes payload orientation mismatch from actual trajectory mismatch without fitting another set of coefficients.</p></section>'''
if 'id="heading-course-separation"' not in h:
 h=h.replace('</body>',section+'\n</body>');p.write_text(h,encoding='utf-8')
