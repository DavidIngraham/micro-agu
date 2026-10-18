from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from OCP.STEPControl import STEPControl_Reader
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_EDGE
from OCP.TopoDS import TopoDS
from OCP.BRepAdaptor import BRepAdaptor_Curve
HERE=Path(__file__).resolve().parent
for name in ['SpreaderBar','SpreaderBarSpar']:
 r=STEPControl_Reader();r.ReadFile(str(Path('/mnt/c/Users/davin/OneDrive/Desktop')/(name+'.STEP')));r.TransferRoots();sh=r.OneShape()
 ex=TopExp_Explorer(sh,TopAbs_EDGE);lines=[]
 while ex.More():
  c=BRepAdaptor_Curve(TopoDS.Edge(ex.Current()));ts=np.linspace(c.FirstParameter(),c.LastParameter(),40);pts=np.array([[c.Value(t).X(),c.Value(t).Y(),c.Value(t).Z()] for t in ts]);lines.append(pts);ex.Next()
 np.savez_compressed(HERE/(name+'-edges.npz'),edges=np.array(lines))
 fig,axs=plt.subplots(2,1,figsize=(12,7),layout='constrained')
 for ax,ij in zip(axs,[(1,2),(1,0)]):
  for pts in lines:ax.plot(pts[:,ij[0]],pts[:,ij[1]],color='#254050',lw=.5)
  ax.set_aspect('equal');ax.set_xlabel('CAD Y (mm)');ax.set_ylabel('CAD '+('Z' if ij[1]==2 else 'X')+' (mm)');ax.grid(alpha=.2)
 fig.suptitle(name+' — direct STEP edge projection');fig.savefig(HERE/(name+'-projection.png'),dpi=150)
