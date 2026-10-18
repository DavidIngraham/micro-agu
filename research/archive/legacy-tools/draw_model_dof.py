from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Ellipse, FancyArrowPatch
fig,axs=plt.subplots(3,3,figsize=(13,11),facecolor='white')
blue,orange,purple='#2678a6','#b96920','#7754a3'
for ax in axs.flat:
 ax.set(xlim=(-1.7,1.7),ylim=(-1.35,1.35));ax.set_aspect('equal');ax.axis('off')
def arrow(ax,a,b,c,lw=2):ax.add_patch(FancyArrowPatch(a,b,arrowstyle='-|>',mutation_scale=17,color=c,lw=lw))
def rotation(ax,c):
 t=np.linspace(.25,2.5,70);x=.88*np.cos(t);y=.88*np.sin(t)
 ax.plot(x,y,color=c,lw=2);arrow(ax,(x[-3],y[-3]),(x[-1],y[-1]),c)
 ax.plot(0,0,'o',ms=9,mfc='white',mec=c,mew=1.6);ax.plot(0,0,'.',ms=5,color=c)
 ax.text(0,-1.18,'Axis points out of the page',ha='center',fontsize=10,color='#555')
def body(ax,kind,color,angle=0,alpha=1):
 if kind=='front':pts=np.array([[-.58,-.23],[.58,-.23],[.58,.23],[-.58,.23],[-.58,-.23]])
 elif kind=='side':pts=np.array([[-.55,-.25],[.48,-.25],[.67,0],[.48,.25],[-.55,.25],[-.55,-.25]])
 else:pts=np.array([[-.45,-.48],[.45,-.48],[.45,.25],[0,.58],[-.45,.25],[-.45,-.48]])
 a=np.deg2rad(angle);pts=pts@np.array([[np.cos(a),np.sin(a)],[-np.sin(a),np.cos(a)]])
 ax.fill(pts[:,0],pts[:,1],color=color,alpha=.13*alpha)
 ax.plot(pts[:,0],pts[:,1],color=color,lw=2,alpha=alpha)
for i,(name,vector) in enumerate([('1  North / south',(1,0)),('2  East / west',(1,0)),('3  Up / down',(0,1))]):
 ax=axs[0,i];ax.add_patch(Rectangle((-.2,-.2),.4,.4,facecolor='#d9e9ed',edgecolor=blue))
 arrow(ax,(-vector[0],-vector[1]),(vector[0],vector[1]),blue)
 arrow(ax,(vector[0],vector[1]),(-vector[0],-vector[1]),blue)
 ax.set_title(name,color=blue,fontsize=15);ax.text(0,-1.12,'Translate the shared attachment',ha='center',fontsize=11)
for j,(kind,axis,view) in enumerate([('front','roll','Front view · longitudinal axis'),('side','pitch','Side view · lateral axis'),('top','yaw','Top view · vertical axis')]):
 ax=axs[1,j];body(ax,kind,orange);rotation(ax,orange)
 ax.set_title(f'{4+j}  Payload {axis}',color=orange,fontsize=15)
 ax.text(0,1.13,view,ha='center',fontsize=10)
 ax=axs[2,j];body(ax,kind,'#89939c',alpha=.65);body(ax,kind,purple,angle=25);rotation(ax,purple)
 ax.set_title(f'{7+j}  Relative {axis}',color=purple,fontsize=15)
 ax.text(0,1.13,view,ha='center',fontsize=10)
fig.suptitle('Nine independent motions: 3 translations + 3 payload rotations + 3 relative rotations',fontsize=17,y=.985)
fig.text(.5,.945,'Each rotation is shown in its own plane; the dot marks the axis coming toward you.',ha='center',fontsize=12)
fig.text(.5,.065,'Bottom row: gray = payload-aligned reference; purple = canopy orientation relative to that reference.',ha='center',fontsize=11)
fig.text(.5,.035,'Schematic orientations, not aircraft geometry. Curved arrows illustrate rotation, not the positive sign convention.',ha='center',fontsize=11)
fig.subplots_adjust(top=.89,bottom=.11,hspace=.48,wspace=.18)
out=Path(__file__).resolve().parents[1]/'docs/assets/model-mechanics';out.mkdir(parents=True,exist_ok=True)
for ext in ['png','svg']:fig.savefig(out/f'nine-dof.{ext}',dpi=160)
