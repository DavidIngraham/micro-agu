from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
O=Path(__file__).parent;r=json.loads((O/'joint-sensitivity-grid.json').read_text());rows=r['rows'];base=next(v for v in rows if v['factors']==[1,1,1]);best=min(rows,key=lambda v:v['mean_bank'])
fig,axes=plt.subplots(1,3,figsize=(12,4),sharey=True)
for ax,yaw in zip(axes,[.1,1,10]):
 z=np.array([[next(v['mean_bank'] for v in rows if v['factors']==[k,c,yaw]) for c in [.1,1,10]] for k in [.1,1,10]])
 im=ax.imshow(z,vmin=min(v['mean_bank'] for v in rows),vmax=max(v['mean_bank'] for v in rows),cmap='viridis')
 for i in range(3):
  for j in range(3):ax.text(j,i,f'{z[i,j]:.1f}',ha='center',va='center',color='white')
 ax.set_xticks(range(3),['0.1×','1×','10×']);ax.set_yticks(range(3),['0.1×','1×','10×']);ax.set_xlabel('Roll damping');ax.set_title(f'Yaw damping {yaw:g}×')
axes[0].set_ylabel('Roll stiffness');fig.suptitle('Mean bank RMSE across six turns (degrees)');fig.tight_layout(rect=(0,0,1,.92));fig.savefig(O/'iteration-joint-sensitivity.png',dpi=150)
p=O/'physics-development.html';h=p.read_text(encoding='utf-8');section=f'''<section id="joint-sensitivity"><h2>Suspension sensitivity does not recover sustained turns</h2><p>A 27-point grid varied roll stiffness, roll damping, and yaw damping independently by 0.1×, 1×, and 10×, retaining the corrected conservative spring and frozen aerodynamic coefficients. Mean bank RMSE across the six development turns changes from {base['mean_bank']:.2f}° at the baseline to {best['mean_bank']:.2f}° at the best tested combination. That combination still has a worst-case bank RMSE of {best['worst_bank']:.2f}°.</p><img src="iteration-joint-sensitivity.png" alt="Grid of turn prediction error for roll stiffness and damping and yaw damping variations" style="max-width:100%"><p>This is a sensitivity diagnostic, not independent validation or a measurement of suspension constants. It argues against these three constants alone being the dominant cause of the mismatch within the tested ranges. Geometry and distributed line loading remain separate questions; the test does not rule them out. Measured RPM and inferred inflow still prescribe thrust.</p></section>'''
if 'id="joint-sensitivity"' not in h:
 h=h.replace('</body>',section+'\n</body>') if '</body>' in h else h.replace('</html>',section+'\n</html>');p.write_text(h,encoding='utf-8')
print('baseline',base['mean_bank'],'best',best['mean_bank'])
