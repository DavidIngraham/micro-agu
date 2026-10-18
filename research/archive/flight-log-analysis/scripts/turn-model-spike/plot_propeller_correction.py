from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
O=Path(__file__).parent
old=json.loads((O/'rpm-propulsion-diagnostic.json').read_text())['turns'];new=json.loads((O/'rpm-propulsion-9x45-diagnostic.json').read_text())['turns']
x=np.arange(6);fig,ax=plt.subplots(figsize=(10,5))
ax.bar(x-.18,[r['mean_apc_thrust_N'] for r in old],.36,label='Earlier 9×6 reference',color='#adb8c2')
ax.bar(x+.18,[r['mean_apc_thrust_N'] for r in new],.36,label='Corrected 9×4.5 reference',color='#247d9e')
ax.set_xticks(x,['HR 447','HR 450','HR 508','TL 326','TL 402','TL 3387']);ax.set_ylabel('Estimated mean thrust (N)');ax.set_title('Correcting propeller pitch reduces estimated thrust');ax.legend();ax.grid(axis='y',alpha=.2)
fig.text(.5,.02,'Three-second windows; measured RPM and inferred axial inflow. APC reference maps—not measured thrust.',ha='center',fontsize=9)
fig.tight_layout(rect=(0,.05,1,1));fig.savefig(O/'iteration-propeller-correction.png',dpi=150)
(O/'propeller-configuration-correction.json').write_text(json.dumps({'source':'Pilot clarification: It was actually 9x4.5','diameter_in':9,'pitch_in':4.5,'supersedes':'9x6 inferred from CAD component name','reference_map':'APC_PER3_9x45E.dat','exact_brand_and_blade_variant':'Not established by size clarification'},indent=2))
p=O/'physics-development.html';h=p.read_text(encoding='utf-8');section='''<section id="propeller-size-correction"><h2>Pilot correction: the flown propeller was 9×4.5</h2><p>The pilot confirmed 9×4.5, superseding the 9×6 assumption taken from the CAD component name. Recomputing the same logged RPM and inferred inflow with the APC 9×4.5E reference map lowers estimated thrust in the six development windows by roughly 21–24%.</p><img src="iteration-propeller-correction.png" alt="Old and corrected reference-map thrust estimates for six turn windows" style="max-width:100%"><p>For the selected airborne Trout Lake samples, median mapped shaft power divided by measured battery input drops from 1.13 to 0.80; relative to ESC electrical input it drops from 1.30 to 0.92. The earlier median energy inconsistency is resolved. These are ratios of computed shaft power to measured electrical power, not independently measured motor efficiencies. Exact blade variant, inflow, density, and sensor calibration still affect the comparison.</p><p>The RPM response fit is unchanged because it was fitted to telemetry rather than a propeller map. Historical 9×6 results are retained as superseded comparisons. No full flight-model validation is implied.</p><p>Reference: <a href="https://www.apcprop.com/files/PER3_9x45E.dat">APC 9×4.5E computed performance table</a>.</p></section>'''
if 'id="propeller-size-correction"' not in h:
 h=h.replace('</body>',section+'\n</body>') if '</body>' in h else h.replace('</html>',section+'\n</html>')
 p.write_text(h,encoding='utf-8')
