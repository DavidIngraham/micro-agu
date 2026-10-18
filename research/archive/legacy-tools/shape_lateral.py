"""Reproducible local lateral frequency-shaping experiment; AI-assisted research."""
import argparse,json,sys
from pathlib import Path
import numpy as np
from scipy.signal import tf2ss,lsim
from scipy.optimize import brentq
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('--model',type=Path,required=True);p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
source=Path(__file__).with_name('classical_lateral.py').read_text().split('A,B,C,residual = linearize')[0]
f=np.geomspace(.001,40,6000);s=2j*np.pi*f
kp,ki,kr,kff=.005,.002,.003,.009
candidates={}
for hz in [4,2,1]:candidates[f'LPF {hz} Hz']=([2*np.pi*hz],[1,2*np.pi*hz])
for hz,z in [(2.5,.2),(2.5,.4),(2.8,.3)]:
    wn=2*np.pi*hz
    candidates[f'Notch {hz} Hz z={z}']=(np.polymul([2*np.pi*4],[1,2*z*wn,wn*wn]),np.polymul([1,2*np.pi*4],[1,1.4*wn,wn*wn]))

def closed(A,B,C,S,num,den):
    af,bf,cf,df=tf2ss(num,den);m=len(af);n=len(A);assert abs(df).max()<1e-12
    aa=np.zeros((n+2*m+2,n+2*m+2));bb=np.zeros(len(aa));aa[:n,:n]=A
    for j,k in enumerate([kp,kr]):
        sl=slice(n+j*m,n+(j+1)*m);aa[sl,:n]=bf@C[j:j+1];aa[sl,sl]=af;aa[:n,sl]=-S*k*np.outer(B,cf[0])
    aa[:n,-2]=B;aa[-2,n:n+m]=-ki*cf[0];aa[-2,-1]=ki;aa[:n,-1]=B*S*(kp+kff);aa[-1,-1]=-10;bb[-1]=10
    cc=np.zeros((3,len(aa)));cc[:2,:n]=C;cc[2,n:n+m]=-S*kp*cf[0];cc[2,n+m:n+2*m]=-S*kr*cf[0];cc[2,-2]=1;cc[2,-1]=S*(kp+kff)
    return aa,bb,cc

allresults=[];plants={};curves={};steps={}
for name,climb,turn in [('level',0,0),('climb',.5,0),('descent',-.5,0),('right_turn',0,10),('left_turn',0,-10)]:
    directory=args.output/name;directory.mkdir(exist_ok=True)
    if name=='level' and not (directory/'trim.npz').exists():
        import shutil;shutil.copy2(args.source/'trim.npz',directory/'trim.npz')
    sys.argv=['classical_lateral.py','--model',str(args.model),'--output',str(directory)]
    ns={};exec(source.replace('find_trim(sim)',f'find_trim(sim,climb_m_s={climb},turn_deg_s={turn})').replace('assert command[1] == 0 and u0 > .01',''),ns)
    # On a left-brake trim, positive differential releases left brake.
    if ns['command'][1]>0:
        exec("span=np.pi*55/360*.1*1300\nu0=-command[1]/span",ns)
        original=source[source.index('def linearize'):]
        exec(original.replace('u[2] += span*du','u[1] -= span*du'),ns)
    A,B,C,res=ns['linearize'](.0025,1e-5);S=5/np.linalg.norm(ns['v0']);plants[name]=(ns,A,B,C,S)
    G=np.array([C@np.linalg.solve(z*np.eye(14)-A,B) for z in s]);np.savez(directory/'plant.npz',A=A,B=B,C=C)
    for label,(num,den) in candidates.items():
        H=np.polyval(num,s)/np.polyval(den,s);L=H*((S*kp+ki/s)*G[:,0]+S*kr*G[:,1]);aa,bb,cc=closed(A,B,C,S,num,den)
        poles=np.linalg.eigvals(aa);stable=max(poles.real)<0
        def loop(ff):
            z=2j*np.pi*ff;g=C@np.linalg.solve(z*np.eye(14)-A,B)
            return np.polyval(num,z)/np.polyval(den,z)*((S*kp+ki/z)*g[0]+S*kr*g[1])
        gm=[]
        for j in np.where(L[:-1].imag*L[1:].imag<0)[0]:
            fc=brentq(lambda ff:loop(ff).imag,f[j],f[j+1]);v=loop(fc)
            if v.real<0 and 1/abs(v)>1:gm.append((1/abs(v),fc))
        delay=[]
        for j in np.where((abs(L[:-1])-1)*(abs(L[1:])-1)<0)[0]:
            fc=brentq(lambda ff:abs(loop(ff))-1,f[j],f[j+1]);phase=np.angle(loop(fc));delay.append(((phase+np.pi)%(2*np.pi)/(2*np.pi*fc),fc))
        t=np.linspace(0,45,9001);_,ys,_=lsim((aa,bb[:,None],cc,np.zeros((3,1))),np.ones(len(t))*.2,t)
        err=abs(ys[:,0]-.2);outside=np.where(err>.004)[0];settle=float(t[outside[-1]+1]) if len(outside) and outside[-1]+1<len(t) else (0 if not len(outside) else None)
        row=dict(trim=name,controller=label,stable=bool(stable),max_real=float(max(poles.real)),gain_margin_db=float(20*np.log10(min(gm)[0])) if gm else None,delay_margin_ms=float(min(delay)[0]*1000) if delay else None,peak_sensitivity=float(max(abs(1/(1+L)))),iae=float(np.trapz(err,t)),overshoot_percent=float((max(ys[:,0])/.2-1)*100),settling_2pct_s=settle,trim_command=ns['command'].tolist(),speed=float(np.linalg.norm(ns['v0'])),residual=res)
        allresults.append(row);curves[name,label]=L;steps[name,label]=(t,ys)
    print(name,[(r['controller'],round(r['gain_margin_db'],2),round(r['delay_margin_ms'],2),round(r['iae'],3)) for r in allresults if r['trim']==name],flush=True)
(args.output/'summary.json').write_text(json.dumps(allresults,indent=2))
# Keep selection evidence visible instead of silently optimizing a scalar score.
fig,ax=plt.subplots(2,1,figsize=(12,8),sharex=True)
for label in candidates:
    L=curves['level',label];ax[0].semilogx(f,20*np.log10(abs(L)),label=label);ax[1].semilogx(f,abs(1/(1+L)),label=label)
ax[0].axhline(0,color='gray',ls='--');ax[0].set(ylabel='Return ratio magnitude (dB)',title='Same controller gains; change only measurement frequency shaping');ax[1].set(xlabel='Frequency (Hz)',ylabel='Sensitivity |1/(1+L)|',xlim=(.01,15))
for a in ax:a.grid(alpha=.25);a.legend(fontsize=8)
fig.tight_layout();fig.savefig(args.output/'shaping-bode.png',dpi=160);plt.close(fig)
fig,ax=plt.subplots(2,1,figsize=(12,8),sharex=True)
for label in candidates:
    t,y=steps['level',label];ax[0].plot(t,y[:,0],label=label);ax[1].plot(t,y[:,2],label=label)
ax[0].axhline(.2,color='gray',ls='--');ax[0].set(ylabel='Heading rate increment (deg/s)',title='Linear +0.2 deg/s step: tracking and brake activity');ax[1].set(xlabel='Time (s)',ylabel='Brake fraction increment',xlim=(0,20))
for a in ax:a.grid(alpha=.25);a.legend(fontsize=8)
fig.tight_layout();fig.savefig(args.output/'shaping-step.png',dpi=160);plt.close(fig)
fig,ax=plt.subplots(1,2,figsize=(13,6))
for label in candidates:
    rr=[r for r in allresults if r['controller']==label];ax[0].plot([r['trim'] for r in rr],[r['gain_margin_db'] for r in rr],'o-',label=label);ax[1].plot([r['trim'] for r in rr],[r['delay_margin_ms'] for r in rr],'o-',label=label)
ax[0].set(ylabel='First gain-increase boundary (dB)',title='Feedback gain margin');ax[1].set(ylabel='First common sensing-delay boundary (ms)',title='Exact pure-delay frequency crossing')
for a in ax:a.grid(alpha=.25);a.tick_params(axis='x',rotation=25);a.legend(fontsize=8)
fig.tight_layout();fig.savefig(args.output/'trim-margins.png',dpi=160);plt.close(fig)

# Nonlinear held-out checks: actual actuator, speed scaling, antiwindup and common sensor delay.
from scipy.signal import bilinear,tf2sos,sosfilt
from microagu_sim.compat.actuator import BrakeActuator
checks=[];traces={}
for name in ['level','right_turn','left_turn']:
    ns,A,B,C,S=plants[name];sim=ns['sim'];base=ns['base'];command=ns['command'];obs=ns['obs'];u0=ns['u0']
    y0=obs(base);r0=y0[0];increment=-2. if name=='left_turn' else 2.
    for label in ['LPF 4 Hz','LPF 1 Hz','Notch 2.5 Hz z=0.2']:
        for delay in [0,.08]:
            dt=.0025;t=np.arange(0,30+dt/2,dt);num,den=candidates[label];bn,an=bilinear(num,den,fs=1/dt);
            if label.startswith('LPF'):
                alpha=dt/(dt+1/num[0]);bn,an=[alpha],[1,-(1-alpha)]
            sos=tf2sos(bn,an);zi=np.zeros((len(sos),2,2))
            state=base.copy();motor=ns['rpm'];act=BrakeActuator(position_mm=command[1:]);integral=u0-S*(kff*r0-kr*y0[1]);target=r0;history=[];trace=[];nd=round(delay/dt)
            for time in t:
                actual=obs(state);history.append(actual);delayed=history[max(0,len(history)-1-nd)]
                filtered,zi=sosfilt(sos,(delayed-y0)[None,:],axis=0,zi=zi);measured=y0+filtered[0]
                target+=np.clip(dt/(dt+.1)*(r0+increment-target),-40*dt,40*dt)
                error=target-measured[0];sv=np.clip(5/max(np.linalg.norm(state[7:10]),2),.25,2.5)
                candidate_i=np.clip(integral+ki*dt*error,-.15,.15)
                unconstrained=sv*(kff*target+kp*error-kr*measured[1])+candidate_i
                if (unconstrained<=1 or error<=0) and (unconstrained>=-1 or error>=0):integral=candidate_i
                control=np.clip(sv*(kff*target+kp*error-kr*measured[1])+integral,-1,1)
                trace.append([*actual,control,target])
                applied=np.array([command[0],max(-control,0)*np.pi*55/360*.1*1300,max(control,0)*np.pi*55/360*.1*1500])
                state,motor,_=sim.step_actuated(state,motor,act,applied,dt=dt)
            trace=np.array(trace);err=trace[:,0]-(r0+increment);row=dict(trim=name,controller=label,delay_ms=delay*1000,increment=increment,iae=float(np.trapz(abs(err),t)),last10_rmse=float(np.sqrt(np.mean(err[t>=20]**2))),peak_abs_rollrate=float(max(abs(trace[:,1]))),max_abs_u=float(max(abs(trace[:,2]))),initial_i=float(u0-S*(kff*r0-kr*y0[1])))
            checks.append(row);traces[name,label,delay]=(t,trace,y0);print('Nonlinear',row,flush=True)
            np.savez_compressed(args.output/name/(label.replace(' ','_')+f'-delay{delay}.npz'),time=t,trace=trace)
(args.output/'nonlinear-summary.json').write_text(json.dumps(checks,indent=2))
fig,ax=plt.subplots(3,2,figsize=(14,11),sharex=True)
for j,name in enumerate(['level','right_turn','left_turn']):
    for delay,ls in [(0,'-'),(.08,'--')]:
        for k,label in enumerate(['LPF 4 Hz','LPF 1 Hz','Notch 2.5 Hz z=0.2']):
            t,y,y0=traces[name,label,delay];kw=dict(color=f'C{k}',ls=ls,label=label+f', {delay*1000:.0f} ms')
            ax[j,0].plot(t,y[:,0]-y0[0],**kw);ax[j,1].plot(t,y[:,2],**kw)
    ax[j,0].axhline(-2 if name=='left_turn' else 2,color='gray',ls=':');ax[j,0].set(ylabel='Δ heading rate (deg/s)',title=name.replace('_',' '));ax[j,1].set(ylabel='Brake fraction',title=name.replace('_',' ')+' control effort')
for a in ax.flat:a.grid(alpha=.25)
ax[0,0].legend(fontsize=7);ax[-1,0].set_xlabel('Time (s)');ax[-1,1].set_xlabel('Time (s)');fig.suptitle('Nonlinear 2 deg/s steps: solid = no delay; dashed = 80 ms common sensing delay')
fig.tight_layout();fig.savefig(args.output/'nonlinear-shaping.png',dpi=160);plt.close(fig)