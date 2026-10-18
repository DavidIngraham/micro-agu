"""Annotate full-order lateral poles and return-ratio zeros using modal participation."""
import sys,json
from pathlib import Path
import numpy as np
from scipy.linalg import eig
from scipy.optimize import linear_sum_assignment
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(sys.argv[1]);p=json.loads((root/'plant.json').read_text());A=np.array(p['A']);B=np.array(p['B']);C=np.array(p['C']);S=5/p['speed'];wf=8*np.pi
v,V=np.linalg.eig(A);W=np.linalg.inv(V);part=abs(V*W.T);part/=part.sum(axis=0)
ix=sorted([i for i,z in enumerate(v) if z.imag>=-1e-7],key=lambda i:abs(v[i].imag))
for n,i in enumerate(ix):
    top=np.argsort(part[:,i])[-4:][::-1];print('P'+str(n+1),v[i],[(p['state_order'][j],round(part[j,i],3)) for j in top])
aa=np.zeros((17,17));aa[:14,:14]=A;aa[14:16,:14]=wf*C;aa[14:16,14:16]=-wf*np.eye(2);aa[16,14]=-.002
bb=np.r_[B,0,0,0];cc=np.r_[np.zeros(14),S*.005,S*.003,-1]
# Generalized Rosenbrock pencil avoids high-order polynomial root conditioning.
def zeros(a,b,c):
    n=len(a);M=np.zeros((n+1,n+1));M[:n,:n]=a;M[:n,n]=b;M[n,:n]=c;E=np.zeros_like(M);E[:n,:n]=np.eye(n)
    z=eig(M,E,right=False);return z[np.isfinite(z)]
z=zeros(aa,bb,cc);print('LOOP ZEROS',z)
for j in range(2):print('OUTPUT ZEROS',j,zeros(A,B,C[j]))

# Infinite generalized eigenvalues may appear as enormous finite floats.
z=z[abs(z)<1e8]
zg=sorted([q for q in z if q.imag>=-1e-7 and abs(q+wf)>1e-5],key=lambda q:abs(q.imag))
names=['Canopy roll-rate relaxation','Bank / lateral relaxation','Slow pitch–forward-speed mode','Side velocity / canopy yaw','Vertical velocity / relative pitch','Relative pitch / canopy pitch rate','Relative pitch / payload pitch–yaw','Relative roll / payload roll rate']
ks=np.r_[0,np.geomspace(.0001,10000,1600)];roots=[]
for k in ks:
    e=np.linalg.eigvals(aa-k*np.outer(bb,cc))
    if roots:
        _,idx=linear_sum_assignment(abs(roots[-1][:,None]-e[None,:]));e=e[idx]
    roots.append(e)
roots=np.array(roots);selected=np.linalg.eigvals(aa-np.outer(bb,cc))
fig=plt.figure(figsize=(19,14));gs=fig.add_gridspec(2,2,height_ratios=[1.3,1],width_ratios=[1.05,1]);ax=fig.add_subplot(gs[0,0]);txt=fig.add_subplot(gs[0,1]);bottom=fig.add_subplot(gs[1,:]);txt.axis('off');bottom.axis('off')
for j in range(17):ax.plot(roots[:,j].real,roots[:,j].imag,color='#7897b3',lw=1,alpha=.8)
ax.scatter(v.real,v.imag,marker='x',color='black',s=65,label='Plant poles, gain = 0',zorder=5)
ax.scatter([0,-wf],[0,0],marker='x',color='#9b59b6',s=80,label='Controller poles',zorder=6)
ax.scatter(zg and np.real(zg),np.imag(zg),facecolors='none',edgecolors='#d35400',s=90,label='Combined-loop zeros',zorder=5)
ax.scatter(selected.real,selected.imag,marker='s',s=32,color='#18834b',label='Selected gains (λ = 1)',zorder=4)
ax.axvline(0,color='black',lw=1);ax.axvspan(0,15,color='#f9e2e2',alpha=.5);ax.grid(alpha=.2)
for n,i in enumerate(ix):
    q=v[i];offset={1:(-65,-18),2:(-75,12),3:(-45,25)}.get(n,(-65,12) if n%2==0 else (10,18))
    ax.annotate('P'+str(n+1),(q.real,q.imag),xytext=offset,textcoords='offset points',fontsize=10,fontweight='bold',arrowprops=dict(arrowstyle='-',color='black'))
for n,q in enumerate(zg):
    offset={1:(50,-10),2:(25,32),3:(35,12)}.get(n,(12,-22) if n%2==0 else (-45,-25))
    ax.annotate('Z'+str(n+1),(q.real,q.imag),xytext=offset,textcoords='offset points',fontsize=9,color='#ad4800',arrowprops=dict(arrowstyle='-',color='#ad4800'))
ax.annotate('I integrator',(0,0),xytext=(30,28),textcoords='offset points',color='#9b59b6',arrowprops=dict(arrowstyle='->'))
ax.annotate('4 Hz measurement filters\n−25.133; two internal states',(-wf,0),xytext=(0,40),textcoords='offset points',fontsize=9,arrowprops=dict(arrowstyle='->'))
crit=1.52413542939;ep=np.linalg.eigvals(aa-crit*np.outer(bb,cc));cp=ep[np.argmin(abs(ep.real))];ax.plot(0,abs(cp.imag),'r*',ms=12)
ax.annotate('Instability boundary\nλ = 1.524, 2.612 Hz',(0,abs(cp.imag)),xytext=(35,30),textcoords='offset points',fontsize=10,color='#b22222',arrowprops=dict(arrowstyle='->',color='#b22222'))
ax.set(xlim=(-29,14),ylim=(-2,43),xlabel='Real part (1/s): farther left → faster decay',ylabel='Imaginary part (rad/s): oscillation frequency × 2π',title='Baseline 4 Hz filter: scale all feedback gains by λ');ax.legend(loc='upper left',fontsize=9)
def coord(q):return f'{q.real:+.3f}'+(f' ± j{abs(q.imag):.3f}' if abs(q.imag)>1e-6 else '')
text='PLANT MODES — labels from left/right eigenvector participation\n\n'
for n,i in enumerate(ix):
    q=v[i];freq=abs(q.imag)/2/np.pi;top=np.argsort(part[:,i])[-2:][::-1]
    text+=f'P{n+1}  {coord(q)} s⁻¹'+(f'   ({freq:.3f} Hz)' if freq else '')+'\n'+names[n]+'\n'+', '.join(p['state_order'][j].replace('payload_body_','payload ').replace('canopy_body_','canopy ').replace('relative_rotation_','relative ') + f' {100*part[j,i]:.0f}%' for j in top)+'\n\n'
txt.text(0,1,text,va='top',fontsize=10.5,linespacing=1.2)
left='LOOP ZEROS — destinations of the moving root-locus branches\n\n'
for n,q in enumerate(zg):left+=f'Z{n+1}  {coord(q)} s⁻¹'+('   RIGHT-HALF PLANE' if q.real>0 else '')+'\n'
left+='\nEach ± pair represents two roots; upper-half roots are labeled.\nThe lower half is its mirror image. Roots and paths outside the\naxis limits are omitted; the listed finite poles and zeros are complete.'
bottom.text(0,1,left,va='top',fontsize=11,linespacing=1.55)
right='HOW TO READ THE PHYSICS\n\nPoles describe natural motions; zeros describe cancellation in an\ninput–output response. A zero is not an independent physical mode.\nThe mixed-mode names are participation-based interpretations,\nnot experimentally identified flight modes. p, q, r are body rates.\n\nThe 2.54 Hz closed-loop pair is −0.986 ± j15.969 (ζ ≈ 0.062).\nFeedback reshapes coupled plant motion; there is no one-to-one\n“2.54 Hz plant pole.” The slowest closed-loop pole is −0.158.\n\nTwo identical measurement filters yield a fixed pole at −25.133.\nOne −25.133 zero cancels it from the minimal scalar return ratio;\nthis internal stable pole stays fixed and is not counted as Z1–Z8.\nThe integrator starts at zero. Target smoothing adds −10 to the\ncommand response but is outside this feedback root locus.\n\nL(s) = H(s)[(S K_P + K_I/s)Gψ(s) + S K_RD Gp(s)]\nClosed-loop characteristic: 1 + λL(s) = 0. Feedforward does not\nmove these poles. S=1.1408; KP=.005, KI=.002, KRD=.003.\n\nTrout Lake 1.758 kg, fixed-throttle level trim; original 4 Hz filter.\nThe recommended 1 Hz candidate has a different root locus.'
bottom.text(.52,1,right,va='top',fontsize=11,linespacing=1.5)
fig.suptitle('Lateral root locus — every plant mode and combined-loop zero',fontsize=19,y=.995);fig.tight_layout();fig.savefig(root/'annotated-root-locus.png',dpi=150);plt.close(fig)
fig,axs=plt.subplots(1,2,figsize=(17,9));zero_data={}
for j,(ax,title) in enumerate(zip(axs,['Brake → heading rate ψ̇','Brake → body roll rate p'])):
    zz=zeros(A,B,C[j]);zz=zz[abs(zz)<1e8];gg=sorted([q for q in zz if q.imag>=-1e-7],key=lambda q:abs(q.imag));zero_data[title]=[[q.real,q.imag] for q in zz]
    ax.scatter(v.real,v.imag,marker='x',c='black',label='Plant poles');ax.scatter(zz.real,zz.imag,facecolors='none',edgecolors='tab:orange',s=80,label='This output’s zeros');ax.axvline(0,color='black');ax.axvspan(0,12,color='#f9e2e2',zorder=0);ax.grid(alpha=.2)
    for n,q in enumerate(gg):ax.annotate(f'{n+1}',(q.real,q.imag),xytext=(8,7),textcoords='offset points',color='#ad4800')
    table='\n'.join(f'{n+1}: {coord(q)}' for n,q in enumerate(gg));ax.text(.03,.97,table,transform=ax.transAxes,va='top',fontsize=10,bbox=dict(facecolor='white',alpha=.9,edgecolor='#ddd'))
    ax.set(xlim=(-23,12),ylim=(-2,41),title=title,xlabel='Real part (1/s)',ylabel='Imaginary part (rad/s)');ax.legend(loc='upper right')
fig.suptitle('Individual plant zeros — these are NOT the combined-feedback root-locus endpoints',fontsize=16)
fig.text(.5,.03,'Zeros are frequencies of input–output cancellation, not natural modes. Red half-plane: nonminimum-phase zeros.\nAll roots are listed in ± pairs; lower-half-plane conjugates are omitted from the drawing. Model-derived, not flight-identified.',ha='center',fontsize=12)
fig.tight_layout(rect=(0,.08,1,.95));fig.savefig(root/'plant-zero-map.png',dpi=150);plt.close(fig)
(root/'annotated-roots.json').write_text(json.dumps(dict(plant_modes=[dict(label=f'P{n+1}',pole=[float(v[i].real),float(v[i].imag)],interpretation=names[n],participation=part[:,i].tolist()) for n,i in enumerate(ix)],loop_zeros=[[float(q.real),float(q.imag)] for q in z],individual_zeros=zero_data),indent=2))