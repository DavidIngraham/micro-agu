from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch,Rectangle,Ellipse
R=Path(__file__).resolve().parents[1]/'docs/assets/flight-testing'
fig,axs=plt.subplots(1,3,figsize=(15,6.8),layout='constrained')
blue='#1769aa';green='#20744c';red='#ae363c';gray='#52615d'
for ax in axs:ax.set_xlim(-1,1.3);ax.set_ylim(-.9,1.35);ax.axis('off');ax.set_aspect('equal')
def arrow(ax,a,b,label,color,xy):
 ax.add_patch(FancyArrowPatch(a,b,arrowstyle='-|>',mutation_scale=18,lw=2.3,color=color));ax.text(*xy,label,color=color,fontsize=12)
a=axs[0];a.set_title('Payload: take moments about its CG',fontsize=13,fontweight='bold');a.add_patch(Rectangle((-.16,-.12),.32,.45,fc='#e9ede7',ec=gray));a.plot([0,0],[0,1],color=gray,lw=2)
a.add_patch(Rectangle((-.23,-.21),.46,.09,fc='#525962',ec=gray));a.annotate('Bottom-mounted\nballast',xy=(-.2,-.17),xytext=(-.94,-.45),fontsize=9,arrowprops=dict(arrowstyle='->',color=gray))
for y,label in [(0,'P: loaded payload CG'),(.32,'S: thrust line'),(1,'H: suspension')]:a.plot(0,y,'ko');a.text(.07,y-.07,label,fontsize=10)
a.plot([-.28,-.28],[.19,.45],color=gray,lw=3);a.plot([-.28,-.16],[.32,.32],color=gray,lw=3)
a.text(-.92,.56,'Rear pusher prop',fontsize=10,color=gray)
arrow(a,(-.92,.32),(-.17,.32),'T: forward thrust',green,(-.94,.43));arrow(a,(0,1),(-.5,1.28),'R',blue,(-.6,1.23));arrow(a,(0,0),(0,-.7),'mₚ g',red,(.09,-.68));arrow(a,(0,0),(-.7,0),'Dₚ',blue,(-.82,.08))
a.annotate('',xy=(-.2,.32),xytext=(-.2,0),arrowprops=dict(arrowstyle='<->',color=gray));a.text(-.4,.13,'hₜ',fontsize=12)
a.add_patch(FancyArrowPatch((.55,.09),(.55,-.4),connectionstyle='arc3,rad=-.7',arrowstyle='-|>',mutation_scale=16,color=green,lw=2));a.text(.42,-.57,'Thrust moment:\nnose down',color=green,fontsize=10)
a.text(-.92,-.85,'Forward →   Up ↑   Nose-up pitch positive',fontsize=9,color=gray)
a=axs[1];a.set_title('Canopy: suspension reacts on it too',fontsize=13,fontweight='bold');a.add_patch(Ellipse((0,.8),1.2,.23,fc='#e8f2fa',ec=blue,lw=2));a.plot([0,0],[0,.8],ls='--',color=gray);a.plot([0,0],[0,.8],'ko');a.text(.1,.65,'C: canopy CG',fontsize=10);a.text(.1,-.07,'H',fontsize=10)
arrow(a,(0,.8),(0,1.3),'Lc',blue,(.08,1.22));arrow(a,(0,.8),(-.85,.8),'Dc',blue,(-.87,.9));arrow(a,(0,.8),(0,.36),'mc g',red,(.09,.38));arrow(a,(0,0),(.5,-.45),'−R',blue,(.56,-.4));a.text(-.94,-.72,'Aerodynamic pitching moment and equal/opposite\njoint couples also enter the angular equations.',fontsize=9,color=gray)
a=axs[2];a.set_title('The reinforcing feedback loop',fontsize=13,fontweight='bold');a.text(-.95,1.1,'Code:  Δu = −K qₚ',fontsize=16);a.text(-.95,.78,'Payload starts pitching down\nqₚ < 0',fontsize=12);a.text(-.95,.38,'Controller adds throttle\nΔu > 0',fontsize=12);a.text(-.95,-.02,'Payload accelerates\nfurther nose-down',fontsize=12,color=red);a.text(-.95,-.52,'The damper adds energy\nto the oscillation.',fontsize=11)
fig.suptitle('Why the throttle damper reinforced the pitch oscillation',fontsize=18,fontweight='bold')
fig.supxlabel('Pusher thrust above the payload CG creates a nose-down moment. Schematic, not to scale.',fontsize=11)
fig.savefig(R/'pitch-damper-fbd-pusher.png',dpi=160);plt.close(fig)

