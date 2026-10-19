import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
P=Path(__file__).resolve().parent;d=json.loads((P/'summary.json').read_text());cells=d['cells'];xx=sorted(set(c['rotor'] for c in cells));yy=sorted(set(c['payload'] for c in cells));fig,axes=plt.subplots(1,3,figsize=(12,4),layout='constrained')
for ax,key,title in zip(axes,['cost','powered','off'],['First-transition fitting cost','Second-transition powered yaw RMSE','Second-transition descent yaw RMSE']):
 a=np.array([[next(c for c in cells if c['payload']==y and c['rotor']==x) for x in xx] for y in yy]);z=np.array([[c['cost'] if key=='cost' else c['validation_rmse'][key]['rmse'][2] for c in row] for row in a]);ax.imshow(z,cmap='viridis_r',aspect='auto');ax.set(xticks=range(len(xx)),xticklabels=xx,yticks=range(len(yy)),yticklabels=yy,xlabel='Rotor inertia multiplier',ylabel='Payload yaw inertia multiplier',title=title)
 for j in range(len(yy)):
  for i in range(len(xx)):ax.text(i,j,f'{z[j,i]:.2f}',ha='center',va='center',color='white' if z[j,i]>np.mean(z) else 'black')
 i=xx.index(d['selected_rotor']);j=yy.index(d['selected_payload']);ax.plot(i,j,'s',markersize=38,markerfacecolor='none',markeredgecolor='red',markeredgewidth=2)
fig.savefig(P/'profile.png',dpi=180)
