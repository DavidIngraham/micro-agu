from pathlib import Path
import json,ctypes,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import fast_canopy_tilt as model
O=Path(__file__).parent;p=json.loads((O/'curved-added-fit.json').read_text())['parameters'];ptr=np.ctypeslib.ndpointer(dtype=np.float64,flags='C_CONTIGUOUS');model.lib.panel_loads.argtypes=[ptr,ptr,ctypes.c_double,ctypes.c_double,ptr,ptr];d=np.linspace(-.05,.05,101);fig,axs=plt.subplots(1,3,figsize=(13,4),constrained_layout=True)
for gain,label in [(0,'Fixed panel orientations'),(1,'Illustrative tilt: 1 rad/m')]:
 out=[]
 for delta in d:
  y=np.zeros(6);model.lib.panel_loads(np.array([5.,0.,.8]),np.zeros(3),max(-delta,0)/.072,max(delta,0)/.072,model.params(p|dict(brake_tilt_rad_per_m=gain)),y);out.append(y)
 out=np.array(out)
 for ax,index,title,unit in zip(axs,[1,3,5],['Rightward force','Right-roll moment','Right-yaw moment'],['N','Nm','Nm']):ax.plot(d*1000,out[:,index],label=label);ax.set(title=title,xlabel='Right minus left engaged pull (mm)',ylabel=unit);ax.grid(alpha=.25);ax.axhline(0,color='gray',lw=.7)
axs[0].legend(fontsize=8);fig.suptitle('Static panel-load illustration: same air velocity and coefficients; not a flight fit');fig.savefig(O/'canopy-tilt-static-loads.png',dpi=150)
