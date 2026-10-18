from pathlib import Path
import numpy as np,json
from scipy.ndimage import gaussian_filter1d
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
O=Path(__file__).parent;summary=[];fig,axes=plt.subplots(2,3,figsize=(14,7))
for row,(day,lo,hi) in enumerate([('2026-01-03',444,454),('2026-02-15',3384,3394)]):
 a=np.load(O/(day+'-physical-input.npz'));Y=a['Y'];t=a['t'];ang=gaussian_filter1d(Y[:,:3],2,axis=0);rates=np.gradient(ang,t,axis=0);ph,th=ang[:,0],ang[:,1];pd,td,yd=rates.T
 kin=np.column_stack([pd-yd*np.sin(th),td*np.cos(ph)+yd*np.sin(ph)*np.cos(th),-td*np.sin(ph)+yd*np.cos(ph)*np.cos(th)])
 gyro=gaussian_filter1d(Y[:,3:6],5,axis=0);z=(t>=lo)&(t<=hi)
 for j in range(3):axes[row,j].plot(t[z]-lo,gyro[z,j],label='Filtered gyro');axes[row,j].plot(t[z]-lo,kin[z,j],label='Attitude-derived',alpha=.8);axes[row,j].set_title(('Hood River' if row==0 else 'Trout Lake')+' / '+['p','q','r'][j]);axes[row,j].set(xlabel='Time (s)',ylabel='Body rate (rad/s)');axes[row,j].grid(alpha=.2)
 summary.append(dict(day=day,start=lo,end=hi,rate_difference_rms=np.sqrt(np.mean((gyro[z]-kin[z])**2,axis=0)).tolist()))
 np.savez_compressed(O/(day+'-attitude-derived-rates.npz'),t=t,rates=kin)
axes[0,0].legend();fig.tight_layout();fig.savefig(O/'iteration-rate-initialization.png',dpi=150);(O/'rate-initialization-check.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
