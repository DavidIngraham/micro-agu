"""Coupled longitudinal control analysis, with causal rotor state and nonlinear checks."""
import argparse,json,sys
from pathlib import Path
import numpy as np
from scipy.linalg import logm,expm
from scipy.signal import tf2ss,lsim
from scipy.optimize import linear_sum_assignment
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('--model',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
source=Path(__file__).with_name('classical_lateral.py').read_text().split('A,B,C,residual = linearize')[0]
f=np.geomspace(.005,15,1500);s=2j*np.pi*f
kv=.6/(5*7);ki=.3*kv;kh=.1
models={};rows=[];curves={}
def battery_scaling(voltage):
    ratio = 22.2 / np.clip(voltage, 18., 22.2)
    # Native servos.cpp feeds its integer-percent compensated limit back to TECS.
    limit = int(min(60 * ratio, 100)) / 100
    return ratio, limit

def filt(kind):
    if kind=='body':return [2*np.pi*10],[1,2*np.pi*10]
    return [2*np.pi*3,0],np.polymul([1,2*np.pi*3],[1,2*np.pi*.6])

def mechanical(A,B,C,S,lateral):
    if lateral=='fixed':return A,B[:,0],C
    n=len(A);aa=np.zeros((n+3,n+3));aa[:n,:n]=A;wf=2*np.pi*float(lateral)
    aa[n:n+2,:n]=wf*C[3:5];aa[n:n+2,n:n+2]=-wf*np.eye(2);aa[n+2,n]=-.002
    aa[:n,n]=-S*.005*B[:,1];aa[:n,n+1]=-S*.003*B[:,1];aa[:n,n+2]=B[:,1]
    cc=np.pad(C,((0,0),(0,3)));return aa,np.r_[B[:,0],0,0,0],cc

def close(A,B,C,gff,kind='body',gain=0,voltage=22.2):
    ratio,limit=battery_scaling(voltage);kv=limit/35;ki=.3*kv
    if gff == .15:gff=limit-.45
    B=B*ratio
    # e_h integrates reference climb minus actual climb; I integrates v_des-v.
    n=len(A);num,den=filt(kind);af,bf,cf,df=tf2ss(num,den);nf=len(af)
    aa=np.zeros((n+3+nf,n+3+nf));aa[:n,:n]=A
    control=np.zeros(len(aa));control[:n]=-kv*C[0];control[n]=(gff+kv)*kh;control[n+1]=1;control[n+2]=gff+kv
    signal=C[2] if kind=='relative' else C[1]
    control[n+3:]=gain*cf[0];aa[:n,:]+=np.outer(B,control)
    aa[n,:n]=-C[0];aa[n+1,:n]=-ki*C[0];aa[n+1,n]=ki*kh;aa[n+1,n+2]=ki
    aa[n+2,n+2]=-2;aa[n+3:,:n]=bf@signal[None,:];aa[n+3:,n+3:]=af
    bb=np.zeros(len(aa));bb[n]=1;bb[n+2]=2
    cc=np.zeros((5,len(aa)));cc[:3,:n]=C[:3];cc[3,n]=1;cc[4]=control
    return aa,bb,cc

cases=[('level',0,22.2,'mission',.15),('climb',.5,22.2,'mission',.15),('descent',-.5,22.2,'mission',.18),('low_voltage',0,20.4,'mission',.15),('dynamic_roll',0,22.2,'dynamic_roll',.15),('dynamic_climb',.5,22.2,'dynamic_roll',.15),('level_descent_slope',0,22.2,'mission',.18)]
configs=[('off','body',0),('wrong sign','body',-.02),('reversed body','body',.02),('washed body','washed',.02),('relative +','relative',.02),('relative −','relative',-.02)]
for name,climb,voltage,profile,gff in cases:
    out=a.output/name;out.mkdir(exist_ok=True);sys.argv=['base','--model',str(a.model),'--output',str(out)]
    code=source.replace("make_simulator('trout_lake', 'mission')",f"make_simulator('trout_lake', '{profile}')").replace('find_trim(sim)',f'find_trim(sim,climb_m_s={climb},voltage={voltage})');ns={};exec(code,ns)
    sim=ns['sim'];base=ns['base'];rpm=ns['rpm'];command=ns['command'];R=ns['R'];span=ns['span'];S=5/np.linalg.norm(ns['v0'])
    def obs(y):
        rp=R.from_quat(y[3:7]);wp=rp.inv().apply(y[10:13]);relative=rp.inv().apply(y[23:26]-y[10:13])
        return np.r_[-y[9],wp[1],relative[1],ns['obs'](y)]
    def linearize(dt,eps):
        def advance(x,u):
            cmd=command.copy();cmd[0]+=u[0];cmd[2]+=span*u[1]
            yn,rn=sim.step(ns['decode'](x[:14]),rpm+1000*x[14],cmd,voltage=voltage,dt=dt)
            return np.r_[ns['encode'](yn),(rn-rpm)/1000]
        E=np.eye(15)*eps;Z=np.zeros(15);U=np.eye(2)*eps
        ad=np.column_stack([(advance(e,[0,0])-advance(-e,[0,0]))/(2*eps) for e in E]);bd=np.column_stack([(advance(Z,u)-advance(Z,-u))/(2*eps) for u in U])
        c=np.column_stack([(obs(ns['decode'](e[:14]))-obs(ns['decode'](-e[:14])))/(2*eps) for e in E]);aug=np.zeros((17,17));aug[:15,:15]=ad;aug[:15,15:]=bd;aug[15:,15:]=np.eye(2);ac=logm(aug)/dt
        assert max(abs(ac.imag).flat)<1e-6
        return ac[:15,:15].real,ac[:15,15:].real,c,float(np.linalg.norm(advance(Z,[0,0]))),ad,bd
    A,B,C,res,ad,bd=linearize(.0025,1e-5);A2,B2,C2,_,_,_=linearize(.00125,5e-6)
    H=np.array([C@np.linalg.solve(z*np.eye(15)-A,B[:,0]) for z in s]);H2=np.array([C2@np.linalg.solve(z*np.eye(15)-A2,B2[:,0]) for z in s]);conv=np.max(abs(H-H2)/np.maximum(abs(H),1e-7),axis=0)
    np.savez(out/'plant.npz',A=A,B=B,C=C,Ad=ad,Bd=bd,frequency=f,response=H)
    models[name]=(ns,A,B,C,S,voltage,climb,gff);print('TRIM',name,command,'convergence',conv,flush=True)
    for lat in ['fixed','4','1']:
        am,bm,cm=mechanical(A,B,C,S,lat);hh=np.array([cm@np.linalg.solve(z*np.eye(len(am))-am,bm) for z in s]);curves[name,lat]=hh
        for label,kind,gain in configs:
            ac,bc,cc=close(am,bm,cm,gff,kind,gain,voltage);eig=np.linalg.eigvals(ac);t=np.linspace(0,60,12001);_,y,_=lsim((ac,bc[:,None],cc,np.zeros((5,1))),np.full(len(t),.05),t)
            row=dict(case=name,lateral=lat,damper=label,gain=gain,max_real=float(eig.real.max()),stable=bool(eig.real.max()<0),climb_error_integral=float(np.trapz(abs(y[:,0]-.05),t)),peak_q_deg_s=float(max(abs(y[:,1]))*180/np.pi),peak_relative_deg_s=float(max(abs(y[:,2]))*180/np.pi),peak_height_error=float(max(abs(y[:,3]))),trim=command.tolist(),voltage=voltage,compensation_ratio=float(battery_scaling(voltage)[0]),tecs_throttle_max=battery_scaling(voltage)[1],residual=res,convergence=conv.tolist())
            rows.append(row)
        print(name,lat,[(r['damper'],round(r['max_real'],4),round(r['peak_q_deg_s'],3)) for r in rows if r['case']==name and r['lateral']==lat],flush=True)
(a.output/'summary.json').write_text(json.dumps(rows,indent=2))
fig,ax=plt.subplots(2,3,figsize=(16,8),sharex=True)
for lat in ['fixed','4','1']:
    H=curves['level',lat]
    for j,title in enumerate(['Climb rate (m/s)','Payload pitch rate (rad/s)','Relative pitch rate (rad/s)']):
        ax[0,j].semilogx(f,20*np.log10(abs(H[:,j])),label='Brakes fixed' if lat=='fixed' else 'Lateral '+lat+' Hz');ax[1,j].semilogx(f,np.unwrap(np.angle(H[:,j]))*180/np.pi)
        ax[0,j].set(title=title+' / throttle',ylabel='Magnitude (dB)');ax[1,j].set(ylabel='Phase (deg)',xlabel='Frequency (Hz)')
for aa in ax.flat:aa.grid(alpha=.25)
ax[0,0].legend();fig.suptitle('15-state longitudinal plant including rotor RPM: effect of active steering');fig.tight_layout();fig.savefig(a.output/'plant-bode.png',dpi=150);plt.close(fig)
fig,ax=plt.subplots(1,3,figsize=(16,6))
ns,A,B,C,S,*_=models['level'];am,bm,cm=mechanical(A,B,C,S,'4')
for axis,kind,title in zip(ax,['body','washed','relative'],['Payload rate, 10 Hz LPF','Payload rate, washout + LPF','Relative rate, washout + LPF']):
    gains=np.linspace(-.3,.3,701);roots=[]
    for gain in gains:
        e=np.linalg.eigvals(close(am,bm,cm,.15,kind,gain)[0])
        if roots:
            _,idx=linear_sum_assignment(abs(roots[-1][:,None]-e[None,:]));e=e[idx]
        roots.append(e)
    roots=np.array(roots)
    for j in range(roots.shape[1]):axis.plot(roots[:,j].real,roots[:,j].imag,lw=.7,color='steelblue',alpha=.6)
    for gain,marker,color,label in [(0,'x','black','Off'),(.02,'o','green','+0.02'),(-.02,'s','red','−0.02')]:
        eig=np.linalg.eigvals(close(am,bm,cm,.15,kind,gain)[0]);axis.scatter(eig.real,eig.imag,marker=marker,c=color,s=25,label=label)
    axis.axvline(0,color='black');axis.set(xlim=(-8,3),ylim=(-2,45),title=title,xlabel='Real part (1/s)',ylabel='Imaginary part (rad/s)');axis.grid(alpha=.25);axis.legend()
fig.suptitle('Damper gain locus, −0.3 to +0.3: positive k means +k × measured rate added to throttle');fig.tight_layout();fig.savefig(a.output/'damper-root-locus.png',dpi=150);plt.close(fig)

# Modal participation and static trim slopes, including the rotor pole.
modal={};trimslopes=[]
for name,(ns,A,B,C,S,voltage,climb,gff) in models.items():
    eig,V=np.linalg.eig(A);W=np.linalg.inv(V);participation=abs(V*W.T);participation/=participation.sum(axis=0)
    modal[name]=[dict(pole=[float(z.real),float(z.imag)],participation=participation[:,j].tolist()) for j,z in enumerate(eig) if z.imag>=-1e-7]
    gm=-C@np.linalg.solve(A,B[:,0]);trimslopes.append(dict(case=name,voltage=voltage,trim_throttle=float(ns['command'][0]),climb=climb,climb_dc_gain=float(gm[0]),inverse_climb_gain=float(1/gm[0]),inverse_command_gain=float(1/gm[0]/battery_scaling(voltage)[0])))
(a.output/'modes.json').write_text(json.dumps(modal,indent=2));(a.output/'trim-slopes.json').write_text(json.dumps(trimslopes,indent=2))
# Validate rotor-inclusive throttle plant with nonlinear sine response.
validation=[];ns,A,B,C,S,voltage,climb,gff=models['level'];sim=ns['sim'];base=ns['base'];rpm=ns['rpm'];cmd=ns['command'];R=ns['R'];dt=.0025
plant=np.load(a.output/'level/plant.npz');ad,bd=plant['Ad'],plant['Bd']
for ff in [.1,.33,1,3]:
    for amplitude in [.0005,.001]:
        duration=max(80,15/ff);t=np.arange(0,duration+dt/2,dt);u=np.tile(cmd,(len(t),1));u[:,0]+=amplitude*np.sin(2*np.pi*ff*t);y,rr=sim.replay(base,u,rpm=rpm,voltage=voltage,dt=dt)
        rp=R.from_quat(y[:,3:7]);q=rp.inv().apply(y[:,10:13])[:,1];rel=rp.inv().apply(y[:,23:26]-y[:,10:13])[:,1];output=np.c_[-y[:,9],q,rel]
        use=t>duration/2;tt=t[use];xx=np.c_[np.sin(2*np.pi*ff*tt),np.cos(2*np.pi*ff*tt),np.ones(len(tt)),tt-tt.mean()];coef=np.linalg.lstsq(xx,output[use],rcond=None)[0];actual=(coef[0]+1j*coef[1])/amplitude;expected=C[:3]@np.linalg.solve(np.exp(2j*np.pi*ff*dt)*np.eye(15)-ad,bd[:,0]);validation.append(dict(frequency=ff,amplitude=amplitude,relative_complex_error=(abs(actual-expected)/np.maximum(abs(expected),1e-10)).tolist()))
(a.output/'sine-validation.json').write_text(json.dumps(validation,indent=2));print('SINE',validation,flush=True)
# Nonlinear diagnostic controller follows native algebra; truth sensing, no EKF.
from microagu_sim.compat.actuator import BrakeActuator
checks=[];histories={};dt=.005
for name,lat in [('level','4'),('level','1'),('climb','1'),('low_voltage','1'),('dynamic_roll','1')]:
    ns,A,B,C,S,voltage,climb,gff=models[name];sim=ns['sim'];sim.substeps=4;base=ns['base'];R=ns['R'];command=ns['command'];rpm=ns['rpm']
    ratio,limit=battery_scaling(voltage);kv=limit/35;ki=.3*kv
    for label,kind,gain in [configs[i] for i in [0,1,2,3,5]]:
        x=np.zeros(14);x[9]=np.deg2rad(1);state=ns['decode'](x);motor=rpm;act=BrakeActuator(position_mm=command[1:]);latf=np.zeros(2);lati=ns['u0'];qf=wash=0.;rateff=climb
        def ff(v):return .45+v*(limit-.45 if v>=0 else .18)
        integral=command[0]/ratio-ff(climb);last=ff(climb);trace=[];t=np.arange(0,90+dt/2,dt)
        for time in t:
            rp=R.from_quat(state[3:7]);rates=rp.inv().apply(state[10:13]);q=rates[1];rel=rp.inv().apply(state[23:26]-state[10:13])[1];height=-state[2];v=-state[9]
            targeth=climb*time+.1*max(time-40,0);targetrate=climb+(.1 if time>=40 else 0);rateff+=dt/(dt+.5)*(targetrate-rateff);vdes=np.clip((targeth-height)/10+rateff,-2.5,1);err=vdes-v
            pd=np.clip(ff(vdes)+kv*err,0,limit);pd=np.clip(pd,last-dt*limit,last+dt*limit);last=pd
            lo=np.clip(-pd-.1,-limit/2,limit/2);hi=np.clip(limit-pd+.1,-limit/2,limit/2);integral=np.clip(integral+ki*dt*err,lo,hi)
            hz=10 if kind=='body' else 3;signal=rel if kind=='relative' else q;qf+=dt/(dt+1/(2*np.pi*hz))*(signal-qf);wash+=dt/(dt+1/(2*np.pi*.6))*(qf-wash);filtered=qf if kind=='body' else qf-wash
            logical_throttle=np.clip(pd+integral+np.clip(gain*filtered,-.25,.25),0,limit)
            throttle=np.clip(ratio*logical_throttle,0,limit)
            actual=ns['obs'](state);latf+=dt/(dt+1/(2*np.pi*float(lat)))*(actual-latf);sv=np.clip(5/max(np.linalg.norm(state[7:10]),2),.25,2.5);candidate=np.clip(lati-.002*dt*latf[0],-.15,.15);u=sv*(-.005*latf[0]-.003*latf[1])+candidate
            if (u<=1 or latf[0]>=0) and (u>=-1 or latf[0]<=0):lati=candidate
            u=np.clip(sv*(-.005*latf[0]-.003*latf[1])+lati,-1,1);applied=np.array([throttle,max(-u,0)*np.pi*55/360*.1*1300,max(u,0)*np.pi*55/360*.1*1500])
            trace.append([v,q,rel,height-targeth,throttle,u,motor])
            state,motor,_=sim.step_actuated(state,motor,act,applied,voltage=voltage,dt=dt)
        y=np.array(trace);kick=t<40;post=(t>=40);settle=t>=70
        row=dict(case=name,lateral=lat,damper=label,pitch_kick_rms_deg_s=float(np.sqrt(np.mean(y[kick,1]**2))*180/np.pi),relative_kick_rms_deg_s=float(np.sqrt(np.mean(y[kick,2]**2))*180/np.pi),kick_height_peak=float(max(abs(y[kick,3]))),climb_height_rmse=float(np.sqrt(np.mean(y[post,3]**2))),settled_pitch_std_deg_s=float(np.std(y[settle,1])*180/np.pi),saturation_fraction=float(np.mean((y[:,4]<=.00001)|(y[:,4]>=limit-.00001))))
        checks.append(row);histories[name,lat,label]=(t,y);np.savez_compressed(a.output/name/(label.replace(' ','_')+'-lat'+lat+'.npz'),time=t,trace=y);print('NONLINEAR',row,flush=True)
(a.output/'nonlinear-summary.json').write_text(json.dumps(checks,indent=2))
fig,axes=plt.subplots(3,2,figsize=(14,11),sharex=True)
for col,(name,lat) in enumerate([('level','4'),('dynamic_roll','1')]):
    for label,kind,gain in [configs[i] for i in [0,1,2,3,5]]:
        t,y=histories[name,lat,label]
        for j,idx in enumerate([1,3,4]):axes[j,col].plot(t,y[:,idx]*(180/np.pi if j==0 else 1),label=label)
    axes[0,col].set_title(name+'; lateral '+lat+' Hz')
    for j,lab in enumerate(['Payload pitch rate (deg/s)','Height error (m)','Throttle fraction']):axes[j,col].set_ylabel(lab);axes[j,col].axvline(40,color='gray',ls='--');axes[j,col].grid(alpha=.25)
    axes[2,col].set_xlabel('Time (s)')
axes[0,0].legend(fontsize=8);fig.suptitle('Nonlinear validation: 1 deg/s pitch kick at 0 s; +0.1 m/s climb command at 40 s');fig.tight_layout();fig.savefig(a.output/'nonlinear-dampers.png',dpi=150);plt.close(fig)
# Closed-loop reference transfer and body-rate damper return ratios.
fig,axes=plt.subplots(2,2,figsize=(13,8),sharex=True);ns,A,B,C,S,*_=models['level'];am,bm,cm=mechanical(A,B,C,S,'4')
for label,kind,gain in [configs[i] for i in [0,1,2,3,5]]:
    ac,bc,cc=close(am,bm,cm,.15,kind,gain);hh=np.array([cc@np.linalg.solve(z*np.eye(len(ac))-ac,bc) for z in s])
    for j,idx in enumerate([0,1]):axes[0,j].semilogx(f,20*np.log10(abs(hh[:,idx])),label=label);axes[1,j].semilogx(f,np.unwrap(np.angle(hh[:,idx]))*180/np.pi)
axes[0,0].set_title('Climb / reference climb');axes[0,1].set_title('Payload pitch rate / reference climb')
for aa in axes.flat:aa.grid(alpha=.25)
for aa in axes[0]:aa.set_ylabel('Magnitude (dB)')
for aa in axes[1]:aa.set(xlabel='Frequency (Hz)',ylabel='Phase (deg)')
axes[0,0].legend(fontsize=8);fig.suptitle('Closed-loop longitudinal frequency response with current 4 Hz lateral feedback');fig.tight_layout();fig.savefig(a.output/'closed-bode.png',dpi=150);plt.close(fig)