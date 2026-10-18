from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
O=Path(__file__).parent;r=json.loads((O/'brake-loading-fit.json').read_text());rows=r['validation'];fig,axes=plt.subplots(1,2,figsize=(11,4))
for variant,color in [('baseline','#9aaeb8'),('fitted','#277f9d')]:
 rr=[v for v in rows if v['variant']==variant and v['group']=='development'];x=np.arange(6);axes[0].bar(x+(-.18 if variant=='baseline' else .18),[v['bank_rmse_deg'] for v in rr],.36,label=variant,color=color)
 rr=[v for v in rows if v['variant']==variant and v['group']=='remaining'];vals=np.sort([v['bank_rmse_deg'] for v in rr]);axes[1].plot(vals,np.arange(1,len(vals)+1)/len(vals),label=variant,color=color)
axes[0].set_xticks(range(6),['HR447','HR450','HR508','TL326','TL402','TL3387'],rotation=25);axes[0].set_ylabel('Bank RMSE (deg)');axes[0].set_title('Development turns');axes[1].set(xlabel='Bank RMSE (deg)',ylabel='Fraction of comparison windows',title='254 excluded internal comparison windows');axes[1].set_xlim(0,30)
for ax in axes:ax.legend();ax.grid(alpha=.2)
fig.suptitle('Fitting brake lift and drag gives only a small improvement');fig.tight_layout();fig.savefig(O/'iteration-brake-loading.png',dpi=150)
p=O/'physics-development.html';h=p.read_text(encoding='utf-8');section='''<section id="brake-loading-fit"><h2>Testing the fixed brake lift and drag terms</h2><p>The previous model fixed CLbrake = CDbrake = 1. Fitting just those two terms on the same 62 balanced training windows, with all other coefficients frozen, gives CLbrake = 0.712 and CDbrake = 0.0137. These multiply normalized mean brake pull in lift and its square in drag, respectively; they are model coefficients, not measured aerodynamic derivatives.</p><img src="iteration-brake-loading.png" alt="Development turn errors and comparison-window error distributions before and after fitting symmetric brake loading" style="max-width:100%"><p>Median bank RMSE on 254 excluded internal comparison windows changes from 2.57° to 2.52°. Development turn errors remain 4.59–21.54°. The small fitted drag coefficient may compensate other model assumptions; it does not establish negligible real brake drag. This two-parameter change does not recover sustained turns.</p><p>These results use the conservative suspension spring and prescribed thrust from measured RPM and inferred inflow. They remain conditional diagnostics, not a validated autonomous simulation. The next structural investigation must address the mapping from differential brake input to distributed canopy forces and moments rather than assume symmetric loading alone explains the failure.</p></section>'''
if 'id="brake-loading-fit"' not in h:
 h=h.replace('</body>',section+'\n</body>') if '</body>' in h else h.replace('</html>',section+'\n</html>');p.write_text(h,encoding='utf-8')
