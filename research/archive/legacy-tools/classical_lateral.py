"""Local classical analysis of the native high-DOF lateral plant; AI-assisted research."""
import argparse
import json
import sys
from pathlib import Path
import numpy as np
from scipy.linalg import logm
from scipy.signal import ss2tf
from scipy.spatial.transform import Rotation as R
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser()
parser.add_argument('--model', type=Path, help='Deprecated; uses the installed microagu-sim package')
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
from microagu_sim.compat.profiles import make_simulator
from microagu_sim.compat.trim import find_trim
sim = make_simulator('trout_lake', 'mission')
sim.substeps = 2
out = args.output
out.mkdir(parents=True, exist_ok=True)
cache = out / 'trim.npz'
if cache.exists():
    t = np.load(cache)
    base, command, rpm = t['state'], t['command'], float(t['rpm'])
else:
    trim = find_trim(sim)
    base, command, rpm = trim['state'], trim['command'], trim['rpm']
    np.savez(cache, state=base, command=command, rpm=rpm)
rp0, rc0 = R.from_quat(base[3:7]), R.from_quat(base[16:20])
a0 = rp0.as_euler('xyz')
rel0 = rp0.inv()*rc0
v0 = rp0.inv().apply(base[7:10])
p0 = rp0.inv().apply(base[10:13])
c0 = rc0.inv().apply(base[23:26])
# Chart removes translation and arbitrary global heading, retaining all internal dynamics.
def decode(x):
    rp = R.from_euler('xyz', [a0[0]+x[0], a0[1]+x[1], a0[2]])
    rc = rp*rel0*R.from_rotvec(x[2:5])
    y = base.copy()
    y[3:7], y[16:20] = rp.as_quat(), rc.as_quat()
    y[7:10] = rp.apply(v0+x[5:8])
    y[10:13], y[23:26] = rp.apply(p0+x[8:11]), rc.apply(c0+x[11:14])
    ap, ac = rp.apply(sim.geometry['hinge']), rc.apply([0,0,.85])
    y[13:16] = y[:3]+ap-ac
    y[20:23] = y[7:10]+np.cross(y[10:13],ap)-np.cross(y[23:26],ac)
    return y

def encode(y):
    rp, rc = R.from_quat(y[3:7]), R.from_quat(y[16:20])
    return np.r_[rp.as_euler('xyz')[:2]-a0[:2], (rel0.inv()*rp.inv()*rc).as_rotvec(),
                 rp.inv().apply(y[7:10])-v0, rp.inv().apply(y[10:13])-p0,
                 rc.inv().apply(y[23:26])-c0]

def obs(y):
    rot = R.from_quat(y[3:7])
    phi, theta, _ = rot.as_euler('xyz')
    p,q,r = rot.inv().apply(y[10:13])
    return np.rad2deg([(q*np.sin(phi)+r*np.cos(phi))/np.cos(theta),p])
span = np.pi*55/360*.1*1500  # actual SERVO4 900..2400 us mapping, mm per unit u
u0 = command[2]/span
assert command[1] == 0 and u0 > .01

def linearize(dt, eps):
    def advance(x, du=0):
        u = command.copy(); u[2] += span*du
        y,_ = sim.step(decode(x),rpm,u,dt=dt)
        return encode(y)
    eye = np.eye(14)*eps
    ad = np.column_stack([(advance(e)-advance(-e))/(2*eps) for e in eye])
    bd = (advance(np.zeros(14),eps)-advance(np.zeros(14),-eps))/(2*eps)
    c = np.column_stack([(obs(decode(e))-obs(decode(-e)))/(2*eps) for e in eye])
    aug = np.eye(15); aug[:14,:14]=ad;aug[:14,14]=bd
    cont = logm(aug)/dt
    assert np.max(abs(cont.imag)) < 1e-6
    return cont[:14,:14].real,cont[:14,14].real,c,float(np.linalg.norm(advance(np.zeros(14))))
A,B,C,residual = linearize(.0025,1e-5)
A2,B2,C2,_ = linearize(.00125,5e-6)
f = np.geomspace(.01,30,1600)
w = 2*np.pi*f
H = np.array([C@np.linalg.solve(1j*ww*np.eye(14)-A,B) for ww in w])
H2 = np.array([C2@np.linalg.solve(1j*ww*np.eye(14)-A2,B2) for ww in w])
num,den = ss2tf(A,B[:,None],C,np.zeros((2,1)))
poles = np.linalg.eigvals(A)
print('Trim:',command,'speed:',np.linalg.norm(v0),'u0:',u0,'residual:',residual,flush=True)
print('Plant poles:',poles,flush=True)
print('Linearization convergence:',np.max(abs(H-H2)/np.maximum(abs(H),1e-8),axis=0),flush=True)
np.savez(out/'plant.npz',A=A,B=B,C=C,num=num,den=den,f=f,H=H,trim_command=command)
(out/'plant.json').write_text(json.dumps(dict(A=A.tolist(),B=B.tolist(),C=C.tolist(),numerator=num.tolist(),denominator=den.tolist(),state_order=['payload_phi','payload_theta','relative_rotation_x','relative_rotation_y','relative_rotation_z','payload_body_vx','payload_body_vy','payload_body_vz','payload_body_p','payload_body_q','payload_body_r','canopy_body_p','canopy_body_q','canopy_body_r'],input='delta normalized differential brake, positive right, on right-brake trim branch',outputs=['heading rate deg/s','body roll rate deg/s'],trim_command=command.tolist(),trim_u=float(u0),speed=float(np.linalg.norm(v0)),dt=.0025,residual=residual),indent=2))
from scipy.linalg import solve_continuous_lyapunov, eigh, svd
from scipy.optimize import brentq, linear_sum_assignment
# Balanced reductions are assessed against the full-state frequency response.
P = solve_continuous_lyapunov(A,-np.outer(B,B))
Q = solve_continuous_lyapunov(A.T,-C.T@C)
def psqrt(m):
    e,v=eigh((m+m.T)/2)
    return v@np.diag(np.sqrt(np.maximum(e,0)))
Rp,Rq=psqrt(P),psqrt(Q)
U,sigma,Vh=svd(Rq.T@Rp)
T=Rp@Vh.T@np.diag(1/np.sqrt(sigma))
Ti=np.diag(1/np.sqrt(sigma))@U.T@Rq.T
reductions=[]
for order in [4,6,8,10,12,14]:
    ar,br,cr=Ti[:order]@A@T[:,:order],Ti[:order]@B,C@T[:,:order]
    hr=np.array([cr@np.linalg.solve(1j*ww*np.eye(order)-ar,br) for ww in w])
    band=(f>=.05)&(f<=10)
    errors=np.max(abs(hr[band]-H[band])/np.maximum(abs(H[band]),1e-8),axis=0)
    reductions.append(dict(order=order,max_relative_complex_error=errors.tolist()))
    if max(errors)<.05:
        rn,rd=ss2tf(ar,br[:,None],cr,np.zeros((2,1)))
        reduced=dict(order=order,A=ar.tolist(),B=br.tolist(),C=cr.tolist(),num=rn.tolist(),den=rd.tolist(),errors=errors.tolist())
        break
print('Reductions:',reductions,flush=True)
S=5/np.linalg.norm(v0);wf=2*np.pi*4
kp,ki,kr,kff=.005,.002,.003,.009

def loop(freq,kp=kp,ki=ki,kr=kr):
    z=2j*np.pi*freq
    gr,gp=C@np.linalg.solve(z*np.eye(14)-A,B)
    return wf/(z+wf)*((S*kp+ki/z)*gr+S*kr*gp)
L=np.array([loop(ff) for ff in f])
Lold=np.array([loop(ff,kp=.04,kr=.03) for ff in f])
def closed(kp=kp,ki=ki,kr=kr,gain=1):
    aa=np.zeros((17,17));aa[:14,:14]=A
    aa[:14,14]=-gain*S*kp*B;aa[:14,15]=-gain*S*kr*B;aa[:14,16]=gain*B
    aa[14:16,:14]=wf*C;aa[14:16,14:16]=-wf*np.eye(2)
    aa[16,14]=-ki
    return aa
nominal=np.linalg.eigvals(closed())
old=np.linalg.eigvals(closed(kp=.04,kr=.03))
# Locate every unity-gain / negative-real crossing rather than selecting one visually.
gain_cross=[];phase_cross=[]
for j in range(len(f)-1):
    if (abs(L[j])-1)*(abs(L[j+1])-1)<0:
        fc=brentq(lambda ff:abs(loop(ff))-1,f[j],f[j+1])
        phase=float(np.interp(np.log(fc),np.log(f),np.unwrap(np.angle(L))*180/np.pi))
        gain_cross.append(dict(frequency_hz=fc,phase_unwrapped_deg=phase,phase_offset_to_minus180_deg=180+phase))
    if L[j].imag*L[j+1].imag<0:
        fc=brentq(lambda ff:loop(ff).imag,f[j],f[j+1])
        if loop(fc).real<0:
            gm=1/abs(loop(fc));phase_cross.append(dict(frequency_hz=fc,gain_margin_factor=gm,gain_margin_db=20*np.log10(gm)))
print('Crossovers:',gain_cross,phase_cross,flush=True)
print('Closed poles selected:',nominal,'old:',old,flush=True)
# Parameter continuation with nearest-pole assignment keeps root-locus branches connected.
def locus(values,fn):
    rows=[]
    for value in values:
        eig=np.linalg.eigvals(fn(value))
        if rows:
            _,j=linear_sum_assignment(abs(rows[-1][:,None]-eig[None,:]))
            eig=eig[j]
        rows.append(eig)
    return np.array(rows)
lambdas=np.r_[0,np.geomspace(.001,30,450)]
pvals=np.linspace(0,.06,500)
rl=locus(lambdas,lambda k:closed(gain=k))
rp=locus(pvals,lambda k:closed(kp=k))
def boundary(values,roots,fn):
    result=[]
    maxreal=roots.real.max(axis=1)
    for j in range(1,len(values)-1):
        if maxreal[j]*maxreal[j+1]<0:
            result.append(brentq(lambda k:np.linalg.eigvals(fn(k)).real.max(),values[j],values[j+1]))
    return result
uniform_boundary=boundary(lambdas,rl,lambda k:closed(gain=k))
p_boundary=boundary(pvals,rp,lambda k:closed(kp=k))
fig,axes=plt.subplots(2,2,figsize=(13,8),sharex=True)
for j,label in enumerate(['Heading rate / brake fraction','Body roll rate / brake fraction']):
    axes[0,j].semilogx(f,20*np.log10(abs(H[:,j])));axes[0,j].set_title(label)
    axes[1,j].semilogx(f,np.unwrap(np.angle(H[:,j]))*180/np.pi)
    axes[0,j].set_ylabel('Magnitude (dB re 1 deg/s per fraction)');axes[1,j].set(ylabel='Phase (deg)',xlabel='Frequency (Hz)')
for ax in axes.flat:ax.grid(True,which='both',alpha=.25)
fig.suptitle('Trout Lake level trim: full 14-state plant, throttle fixed at 44.59%')
fig.tight_layout();fig.savefig(out/'plant-bode.png',dpi=160);plt.close(fig)
fig,axes=plt.subplots(2,1,figsize=(11,8),sharex=True)
for response,label in [(L,'Selected gains'),(Lold,'Earlier gains: P=.04, RD=.03')]:
    axes[0].semilogx(f,20*np.log10(abs(response)),label=label)
    axes[1].semilogx(f,np.unwrap(np.angle(response))*180/np.pi,label=label)
axes[0].axhline(0,color='gray',ls='--');axes[1].axhline(-180,color='gray',ls='--')
axes[0].set(ylabel='Loop magnitude (dB)',title='Combined return ratio: heading-rate PI + body-roll damping')
axes[1].set(ylabel='Phase (deg)',xlabel='Frequency (Hz)')
for row in gain_cross:axes[0].plot(row['frequency_hz'],0,'o',color='tab:green')
for ax in axes:ax.grid(True,which='both',alpha=.25);ax.legend()
fig.tight_layout();fig.savefig(out/'loop-bode.png',dpi=160);plt.close(fig)
fig,axes=plt.subplots(1,2,figsize=(13,6))
for ax,roots,title in [(axes[0],rl,'All feedback gains × λ (0 to 30)'),(axes[1],rp,'Sweep heading-rate P (0 to 0.06); I and RD fixed')]:
    for j in range(roots.shape[1]):ax.plot(roots[:,j].real,roots[:,j].imag,color='tab:blue',alpha=.6,lw=1)
    ax.scatter(roots[0].real,roots[0].imag,marker='x',color='black',label='Sweep start')
    ax.scatter(nominal.real,nominal.imag,facecolors='none',edgecolors='tab:green',s=65,label='Selected controller')
    ax.axvline(0,color='black',lw=.8);ax.axhline(0,color='gray',lw=.5)
    ax.set(xlabel='Real part (1/s)',ylabel='Imaginary part (rad/s)',title=title,xlim=(-35,12),ylim=(-55,55));ax.grid(alpha=.2);ax.legend(fontsize=9)
for ax,critical,fn,label in [(axes[0],uniform_boundary,lambda k:closed(gain=k),'λ'),(axes[1],p_boundary,lambda k:closed(kp=k),'K_P')]:
    for value in critical:
        eig=np.linalg.eigvals(fn(value));cross=eig[np.argmin(abs(eig.real))]
        ax.plot(0,abs(cross.imag),'ro',ms=5)
        ax.annotate(f'{label} = {value:.5g}',(0,abs(cross.imag)),xytext=(-100,30),textcoords='offset points',arrowprops=dict(arrowstyle='->',color='red'),fontsize=10)
fig.suptitle('Root locus of the full coupled lateral feedback model; feedforward does not move these poles')
fig.tight_layout();fig.savefig(out/'root-locus.png',dpi=160);plt.close(fig)
# A sampled-data variant uses the native backward-Euler filters and the same-tick I update.
from scipy.linalg import expm
sample_dt=.0025
ab=np.zeros((15,15));ab[:14,:14]=A;ab[:14,14]=B
exp=expm(ab*sample_dt);Ad,Bd=exp[:14,:14],exp[:14,14]
al=sample_dt/(sample_dt+1/wf)
def discrete(kp=kp,ki=ki,kr=kr):
    def step(z):
        x=z[:14]; filt=al*C@x+(1-al)*z[14:16]
        integral=z[16]-ki*sample_dt*filt[0]
        u=-S*kp*filt[0]-S*kr*filt[1]+integral
        return np.r_[Ad@x+Bd*u,filt,integral]
    return np.column_stack([step(v) for v in np.eye(17)])
ds=np.linalg.eigvals(discrete());do=np.linalg.eigvals(discrete(kp=.04,kr=.03))
summary=dict(trim=dict(throttle=float(command[0]),left_mm=float(command[1]),right_mm=float(command[2]),u=float(u0),speed=float(np.linalg.norm(v0)),rpm=float(rpm)),linearization_step_residual=residual,convergence_relative_complex=np.max(abs(H-H2)/np.maximum(abs(H),1e-8),axis=0).tolist(),plant_poles=[[float(z.real),float(z.imag)] for z in poles],dc_gain=(-C@np.linalg.solve(A,B)).tolist(),reductions=reductions,reduced=reduced,gain_crossovers=gain_cross,phase_crossovers=phase_cross,uniform_gain_stability_boundaries=uniform_boundary,p_gain_stability_boundaries=p_boundary,selected_poles=[[float(z.real),float(z.imag)] for z in nominal],earlier_poles=[[float(z.real),float(z.imag)] for z in old],selected_max_pole_real=float(nominal.real.max()),earlier_max_pole_real=float(old.real.max()),sampled_dt=sample_dt,sampled_selected_spectral_radius=float(abs(ds).max()),sampled_earlier_spectral_radius=float(abs(do).max()))
(out/'summary.json').write_text(json.dumps(summary,indent=2))
print('SUMMARY:',{k:summary[k] for k in ['dc_gain','uniform_gain_stability_boundaries','p_gain_stability_boundaries','selected_max_pole_real','earlier_max_pole_real','sampled_selected_spectral_radius','sampled_earlier_spectral_radius']},flush=True)
# Validate local transfer response against the original nonlinear physical model.
checks=[]
for ff in [.1,.35,1.,2.6,6.]:
    for amplitude in [.0005,.001]:
        dt=.0025;duration=max(70,15/ff)
        times=np.arange(round(duration/dt)+1)*dt
        commands=np.tile(command,(len(times),1))
        commands[:,2]+=span*amplitude*np.sin(2*np.pi*ff*times)
        states,rpms=sim.replay(base,commands,rpm=rpm,dt=dt)
        rot=R.from_quat(states[:,3:7]);angles=rot.as_euler('xyz');body=rot.inv().apply(states[:,10:13])
        outputs=np.rad2deg(np.c_[(body[:,1]*np.sin(angles[:,0])+body[:,2]*np.cos(angles[:,0]))/np.cos(angles[:,1]),body[:,0]])
        mask=times>duration/2;tt=times[mask]
        X=np.c_[np.sin(2*np.pi*ff*tt),np.cos(2*np.pi*ff*tt),np.ones(len(tt)),tt-tt.mean()]
        coeff=np.linalg.lstsq(X,outputs[mask],rcond=None)[0]
        response=(coeff[0]+1j*coeff[1])/amplitude
        expected=C@np.linalg.solve(2j*np.pi*ff*np.eye(14)-A,B)
        sampled_expected=C@np.linalg.solve(np.exp(2j*np.pi*ff*dt)*np.eye(14)-Ad,Bd)
        error=abs(response-sampled_expected)/np.maximum(abs(sampled_expected),1e-10)
        checks.append(dict(frequency_hz=ff,amplitude=amplitude,gain=abs(response).tolist(),phase_deg=np.angle(response,deg=True).tolist(),relative_complex_error=error.tolist(),continuous_relative_complex_error=(abs(response-expected)/np.maximum(abs(expected),1e-10)).tolist()))
        print('Nonlinear sine:',checks[-1],flush=True)
        if amplitude==.001:np.savez_compressed(out/f'sine-{ff}.npz',time=times,output=outputs,brake=commands[:,2])
summary['nonlinear_sine_validation']=checks
summary['max_sine_relative_complex_error']=np.max([c['relative_complex_error'] for c in checks],axis=0).tolist()
# Preserve physical full-order polynomials; avoid concealing modes through pole cancellation.
summary['transfer_functions']=dict(denominator=den.tolist(),heading_numerator=num[0].tolist(),roll_numerator=num[1].tolist(),heading_zeros=[[float(z.real),float(z.imag)] for z in np.roots(np.trim_zeros(num[0],'f'))],roll_zeros=[[float(z.real),float(z.imag)] for z in np.roots(np.trim_zeros(num[1],'f'))])
(out/'summary.json').write_text(json.dumps(summary,indent=2))
fig,axes=plt.subplots(2,2,figsize=(13,8),sharex=True)
for j,label in enumerate(['Heading rate / brake fraction','Body roll rate / brake fraction']):
    axes[0,j].semilogx(f,20*np.log10(abs(H[:,j])),label='Local linear model')
    axes[1,j].semilogx(f,np.unwrap(np.angle(H[:,j]))*180/np.pi)
    for check in checks:
        if check['amplitude']!=.001:continue
        ff=check['frequency_hz'];phase=check['phase_deg'][j]
        target=np.interp(np.log(ff),np.log(f),np.unwrap(np.angle(H[:,j]))*180/np.pi)
        phase+=360*round((target-phase)/360)
        axes[0,j].plot(ff,20*np.log10(check['gain'][j]),'o',color='tab:orange')
        axes[1,j].plot(ff,phase,'o',color='tab:orange')
    axes[0,j].plot([],[],'o',color='tab:orange',label='Nonlinear sine, ±0.001 fraction')
    axes[0,j].set(title=label,ylabel='Magnitude (dB re 1 deg/s per fraction)');axes[0,j].legend(fontsize=9)
    axes[1,j].set(ylabel='Phase (deg)',xlabel='Frequency (Hz)')
for ax in axes.flat:ax.grid(True,which='both',alpha=.25)
fig.suptitle('Trout Lake level trim: fixed throttle, right-brake branch, full coupled plant')
fig.tight_layout();fig.savefig(out/'plant-bode.png',dpi=160);plt.close(fig)
# Closed-loop command response including target smoothing and feedforward.
from scipy.signal import lsim
cl=closed();aa=np.zeros((18,18));aa[:17,:17]=cl;aa[17,17]=-10
bb=np.zeros(18);bb[17]=10
ref_gain=S*(kp+kff)
aa[:14,17]=B*ref_gain;aa[16,17]=ki
cc=np.zeros((3,18));cc[:2,:14]=C;cc[2,14]=-S*kp;cc[2,15]=-S*kr;cc[2,16]=1;cc[2,17]=ref_gain
step_time=np.linspace(0,45,18001)
_,ys,_=lsim((aa,bb[:,None],cc,np.zeros((3,1))),np.full(len(step_time),.2),step_time)
fig,axes=plt.subplots(3,1,figsize=(11,8),sharex=True)
for ax,y,label in zip(axes,ys.T,['Δ heading rate (deg/s)','Δ body roll rate (deg/s)','Δ brake fraction']):ax.plot(step_time,y);ax.set_ylabel(label);ax.grid(alpha=.25)
axes[0].axhline(.2,color='gray',ls='--',label='Requested increment');axes[0].legend()
axes[0].set_title('Local command response: +0.2 deg/s step, selected gains and feedforward')
axes[-1].set_xlabel('Time (s)');fig.tight_layout();fig.savefig(out/'closed-step.png',dpi=160);plt.close(fig)
print('NUM HEADING',num[0],flush=True);print('NUM ROLL',num[1],flush=True);print('DEN',den,flush=True)
print('Validation max',summary['max_sine_relative_complex_error'],flush=True)
from microagu_sim.compat.actuator import BrakeActuator
state=base.copy();motor=rpm;act=BrakeActuator(position_mm=command[1:])
filt=obs(base);integral=u0;target=0.;nonlinear=[]
dt=.0025
for time in step_time:
    actual=obs(state)
    filt+=al*(actual-filt)
    target+=dt/(dt+.1)*(.2-target)
    error=target-filt[0]
    sv=np.clip(5/max(np.linalg.norm(state[7:10]),2),.25,2.5)
    candidate_i=np.clip(integral+ki*dt*error,-.15,.15)
    unconstrained=sv*(kff*target+kp*error-kr*filt[1])+candidate_i
    if (unconstrained<=1 or error<=0) and (unconstrained>=-1 or error>=0):integral=candidate_i
    control=np.clip(sv*(kff*target+kp*error-kr*filt[1])+integral,-1,1)
    nonlinear.append([*actual,control-u0])
    applied=np.array([command[0],max(-control,0)*np.pi*55/360*.1*1300,max(control,0)*span])
    state,motor,_=sim.step_actuated(state,motor,act,applied,dt=dt)
nonlinear=np.array(nonlinear)
summary['nonlinear_step_max_absolute_error'] = np.max(abs(nonlinear[:,:2]-ys[:,:2]),axis=0).tolist()
summary['trim_payload_euler_deg'] = np.rad2deg(a0).tolist()
summary['model_source_revision'] = __import__('subprocess').check_output(['git','-C',str(args.model),'rev-parse','HEAD'],text=True).strip()
(out/'summary.json').write_text(json.dumps(summary,indent=2))
np.savez_compressed(out/'closed-step.npz',time=step_time,linear=ys,nonlinear=nonlinear)
fig,axes=plt.subplots(3,1,figsize=(11,8),sharex=True)
for j,(ax,label) in enumerate(zip(axes,['Δ heading rate (deg/s)','Δ body roll rate (deg/s)','Δ brake fraction'])):
    ax.plot(step_time,ys[:,j],label='Continuous linear model')
    ax.plot(step_time,nonlinear[:,j],ls='--',label='Nonlinear model + sampled control')
    ax.set_ylabel(label);ax.grid(alpha=.25)
axes[0].axhline(.2,color='gray',ls=':',label='Requested increment');axes[0].legend(fontsize=9)
axes[0].set_title('Small-signal closed-loop check: +0.2 deg/s command at level trim')
axes[-1].set_xlabel('Time (s)');fig.tight_layout();fig.savefig(out/'closed-step.png',dpi=160);plt.close(fig)
print('Step validation max absolute errors:',summary['nonlinear_step_max_absolute_error'],flush=True)
