"""AI-assisted research controller; does not modify or install flight firmware.

Scheduled LQI with equilibrium feedforward and shared references/actuator limits.
Run with OPENBLAS_NUM_THREADS=1 and the documented ArduPilot Python environment.
"""
import argparse
import hashlib
import itertools
import json
import sys
from pathlib import Path

import numpy as np
from scipy.linalg import solve_discrete_are, solve_discrete_lyapunov
from scipy.spatial.transform import Rotation as R

HERE = Path(__file__).resolve().parent
OLD = HERE.parent / 'state-space-2026-10-14'
sys.path.insert(0, str(OLD))
from study import Chart, DT, XS, US, BASE, observations, plants, setup, source_hashes

MODEL = HERE.parents[1] / 'simulation'
VOLTAGES = [20.4, 22.2]
CLIMBS = [-.5, 0., .5]
TURNS = [-10., 0., 10.]
STROKES = np.array([1300, 1500])*np.pi*55/360*.1


def save(name, data):
    (HERE/name).write_text(json.dumps(data, indent=2, allow_nan=False)+'\n')


def scaling(voltage, speed):
    return np.array([22.2/voltage, np.clip(5/max(speed, 2), .25, 2.5)])


def limits(voltage):
    # Continuous ceiling avoids artificial one-percentage-point jumps in voltage ramps.
    return min(.6*22.2/voltage, 1.)


def brake_command(value):
    return np.array([max(-value, 0)*STROKES[0], max(value, 0)*STROKES[1]])


def prepare():
    make = setup(MODEL)
    from microagu_sim.compat.equilibrium import solve
    p = plants()[0]
    sim = make('trout_lake', 'mission'); sim.substeps = 16
    chart = Chart(sim, p['state'], float(p['rpm']))
    records = []; arrays = {k: [] for k in ['A', 'B', 'E', 'C', 'x', 'u', 'scale']}
    for voltage, climb, turn in itertools.product(VOLTAGES, CLIMBS, TURNS):
        trim = solve(sim, p['state'], p['command'], climb=climb, turn=turn, voltage=voltage)
        residual = float(np.max(np.abs(trim['residual'])))
        if residual > 1e-6:
            raise RuntimeError((voltage, climb, turn, residual))
        x0 = chart.encode(trim['state'], trim['rpm']); x0[15] = 0
        # Solver's signed brake convention includes left-offset compensation.
        cmd = trim['command']; signed = cmd[2]/STROKES[1] if cmd[2]>0 else -cmd[1]/STROKES[0]
        u0 = np.array([cmd[0], signed])
        def advance(x, u, wind):
            y, rpm = chart.decode(x)
            c = np.r_[u[0], brake_command(u[1])]
            y, rpm = sim.step(y, rpm, c, voltage=voltage, wind=wind, dt=DT)
            out = chart.encode(y, rpm); out[15] -= climb*DT
            return out
        def jac(eps):
            A = np.column_stack([(advance(x0+e,u0,np.zeros(3))-advance(x0-e,u0,np.zeros(3)))/(2*eps) for e in np.eye(16)*eps])
            B = np.column_stack([(advance(x0,u0+e,np.zeros(3))-advance(x0,u0-e,np.zeros(3)))/(2*eps) for e in np.eye(2)*eps])
            E = np.column_stack([(advance(x0,u0,e)-advance(x0,u0,-e))/(2*eps) for e in np.eye(3)*eps])
            C = np.column_stack([(observations(chart.decode(x0+e)[0])-observations(chart.decode(x0-e)[0]))/(2*eps) for e in np.eye(16)*eps])
            return A,B,E,C
        A,B,E,C = jac(1e-5); A2,B2,_,_ = jac(5e-6)
        speed = float(np.linalg.norm(trim['state'][7:10]))
        drift = float(np.linalg.norm(advance(x0,u0,np.zeros(3))-x0))
        for k,v in dict(A=A,B=B,E=E,C=C,x=x0,u=u0,scale=scaling(voltage,speed)).items(): arrays[k].append(v)
        records.append(dict(voltage=voltage,climb=climb,turn=turn,speed=speed,residual=residual,
                            drift=drift,A_relative_error=float(np.linalg.norm(A-A2)/np.linalg.norm(A)),
                            B_relative_error=float(np.linalg.norm(B-B2)/np.linalg.norm(B))))
        print(records[-1], flush=True)
    np.savez_compressed(HERE/'grid.npz', **{k:np.array(v) for k,v in arrays.items()})
    save('grid.json', dict(nodes=records,source_hashes=source_hashes(),
        note='Common nominal chart; trim at requested turn/climb/voltage; no new trim oracle during tests.'))


def augmented(A,B,C,scale):
    An=A*XS[None,:]/XS[:,None]
    Bn=B*(US*scale)[None,:]/XS[:,None]
    H=C[[5,3]]*XS[None,:]/np.array([1.,10.])[:,None]
    AA=np.eye(18);AA[:16,:16]=An;AA[:16,16:]=0;AA[16:,:16]=DT*H
    BB=np.zeros((18,2));BB[:16]=Bn
    return AA,BB,H


def design(grid, motion, integral, effort):
    gains=[]; scores=[]; poles=[]
    for A,B,E,C,scale in zip(*(grid[k] for k in ['A','B','E','C','scale'])):
        AA,BB,H=augmented(A,B,C,scale)
        Y=np.zeros((7,18));Y[:5,:16]=C[[5,3,0,1,2]]*XS[None,:]/np.array([1.,10.,.5,np.deg2rad(30),np.deg2rad(30)])[:,None]
        Q=Y.T@np.diag([1,1,1,motion,motion,1,1])@Y+np.diag(np.r_[np.full(16,.002),integral,integral])
        P=solve_discrete_are(AA,BB,Q,np.eye(2)*effort)
        K=np.linalg.solve(np.eye(2)*effort+BB.T@P@BB,BB.T@P@AA)
        F=AA-BB@K
        pole=float(np.log(max(abs(np.linalg.eigvals(F))))/DT)
        if pole>=0: raise RuntimeError('Unstable synthesized node')
        Y[5:]=-K*scale[:,None]
        W=np.zeros((18,3));W[:16]=E/XS[:,None]
        gust=0.; weights=np.array([1,1,.8,.6,.4,.25]);weights/=np.linalg.norm(weights)
        for f,w in zip([.08,.13,.21,.34,.55,.89],weights):
            response=Y@np.linalg.solve(np.exp(2j*np.pi*f*DT)*np.eye(18)-F,W)
            gust+=float(np.sum(abs(response*np.array([.5,.5,.25]))**2)*w*w)
        # Initial height / turn-rate tracking offsets; gust and recovery scored separately.
        initial=np.zeros((18,2));initial[15,0]=1.
        initial[:16,1]=np.linalg.pinv(H)[...,1]
        P=solve_discrete_lyapunov(F,initial@initial.T,method='bilinear')
        recovery=float(DT*np.trace(Y@P@Y.T))
        gains.append(K);scores.append(dict(gust=gust,recovery=recovery));poles.append(pole)
    return np.array(gains),scores,poles


def screen():
    grid=dict(np.load(HERE/'grid.npz')); records=[]
    for motion,integ,effort in itertools.product([.3,3,30],[.03,.3,3],[1,10,100,1000,10000]):
        K,metrics,poles=design(grid,motion,integ,effort)
        stress=stress_poles(grid,K)
        records.append(dict(motion=motion,integral=integ,effort=effort,metrics=metrics,worst_pole=max(poles),
                            stress_worst=max(stress),stress_unstable=sum(p>=0 for p in stress)))
    # Normalize each component to its median across this declared family, not to baseline.
    gm=np.median([np.mean([m['gust'] for m in r['metrics']]) for r in records])
    rm=np.median([np.mean([m['recovery'] for m in r['metrics']]) for r in records])
    for r in records:
        v=np.array([.5*m['gust']/gm+.5*m['recovery']/rm for m in r['metrics']])
        r['score']=float(.5*np.mean(v)+.5*max(v))
    feasible=[r for r in records if r['stress_unstable']==0]
    if not feasible:
        save('failed-screen.json',records)
        raise RuntimeError('No candidate passed the combined uncertainty screen')
    best=min(feasible,key=lambda r:r['score'])
    K,_,_=design(grid,best['motion'],best['integral'],best['effort'])
    np.savez_compressed(HERE/'gains.npz',K=K)
    save('screen.json',dict(selected=best,normalizers=dict(gust=gm,recovery=rm),candidates=records))
    print('Selected', {k:v for k,v in best.items() if k!='metrics'},flush=True)


def stress_poles(grid,gains):
    poles=[]
    for i in range(18):
        AA,BB,_=augmented(grid['A'][i],grid['B'][i],grid['C'][i],grid['scale'][i])
        for gt,gb,dt,db in itertools.product([.75,1.25],[.75,1.25],[0,2],[0,2]):
            F=np.zeros((18+dt+db,18+dt+db));F[:18,:18]=AA;pos=18
            for j,delay,gain in [(0,dt,gt),(1,db,gb)]:
                if delay:
                    F[:18,pos+delay-1]=BB[:,j]*gain;F[pos,:18]=-gains[i,j]
                    for k in range(1,delay):F[pos+k,pos+k-1]=1
                    pos+=delay
                else:F[:18,:18]-=np.outer(BB[:,j]*gain,gains[i,j])
            poles.append(float(np.log(max(abs(np.linalg.eigvals(F))))/DT))
    return poles


def weights(voltage,climb,turn):
    out=np.zeros(18)
    def bracket(grid,value):
        value=np.clip(value,grid[0],grid[-1]);i=min(np.searchsorted(grid,value,side='right')-1,len(grid)-2)
        f=(value-grid[i])/(grid[i+1]-grid[i]);return [(i,1-f),(i+1,f)]
    for (iv,wv),(ic,wc),(it,wt) in itertools.product(bracket(VOLTAGES,voltage),bracket(CLIMBS,climb),bracket(TURNS,turn)):
        out[iv*9+ic*3+it]=wv*wc*wt
    return out


class Reference:
    def __init__(self):self.height=0.;self.turn=0.
    def update(self,height,turn):
        climb=np.clip((height-self.height)/.5,-.5,.5)
        self.height+=DT*climb
        target=np.clip(turn,-10,10)
        self.turn+=np.clip(DT/(DT+.1)*(target-self.turn),-40*DT,40*DT)
        return self.height,climb,self.turn


class Actuation:
    def __init__(self,sim,command):
        from microagu_sim.compat.actuator import BrakeActuator
        self.brake=BrakeActuator(position_mm=command[1:]);self.throttle=command[0]
    def apply(self,requested,voltage):
        limit=limits(voltage)
        throttle=np.clip(np.clip(requested[0],self.throttle-DT*limit,self.throttle+DT*limit),0,limit)
        target=brake_command(np.clip(requested[1],-1,1))
        average=self.brake.advance(target,DT)
        effective=average[1]/STROKES[1]-average[0]/STROKES[0]
        self.throttle=throttle
        return np.r_[throttle,average],np.array([throttle,effective])


class Conventional:
    def __init__(self,p,cfg):
        self.cfg=cfg;self.z=np.zeros(4);self.integral=np.zeros(2)
        self.bias=np.array([p['command'][0]/(22.2/p['voltage']),p['command'][2]/STROKES[1]])
    def request(self,obs,height,climb,turn,voltage,speed):
        cfg=self.cfg;ratio,sv=scaling(voltage,speed);limit=limits(voltage)
        al=DT/(DT+1/(2*np.pi*cfg['lat_hz']));aq=DT/(DT+1/(2*np.pi*cfg['qhz']))
        self.z[:2]+=al*(obs[[3,4]]-self.z[:2]);self.z[2]+=aq*(obs[1]-self.z[2]);self.z[3]+=aq*(self.z[2]-self.z[3])
        turnerror=turn-self.z[0];vd=np.clip(climb+cfg['kh']*(height-obs[5]),-2.5,1.)
        ev=vd-obs[0];kv=cfg['kv']*limit/.6
        ff=vd*(limit-.45 if vd>=0 else .18)
        raw=np.array([ratio*(self.bias[0]+ff+kv*ev-cfg['qgain']*self.z[3]+self.integral[0]),
                      self.bias[1]+sv*(.009*turn+cfg['kp']*turnerror-cfg['rd']*self.z[1])+self.integral[1]])
        self.integral+=DT*np.array([.3*kv*ev,cfg['ki']*turnerror])
        self.ratio=ratio;self.raw=raw
        return raw
    def applied(self,value):
        self.integral+=DT/.25*(value-self.raw)/np.array([self.ratio,1.])
        self.integral=np.clip(self.integral,[-.3,-.15],[.3,.15])


class Scheduled:
    def __init__(self,grid,K,fixed_voltage=False,fixed_speed=False):
        self.grid=grid;self.K=K;self.integral=np.zeros(2)
        self.fixed_voltage=fixed_voltage;self.fixed_speed=fixed_speed
    def request(self,x,obs,height,climb,turn,voltage,speed):
        if self.fixed_voltage:voltage=22.2
        w=weights(voltage,climb,turn);xref=w@self.grid['x'];xref[15]=height
        uref=w@self.grid['u'];K=np.tensordot(w,self.K,axes=1)
        scale=scaling(voltage,speed)*US
        if self.fixed_speed:scale[1]=(w@self.grid['scale'])[1]*US[1]
        self.Ki=scale[:,None]*K[:,16:]
        self.raw=uref-scale*(K@np.r_[(x-xref)/XS,self.integral])
        self.integral+=DT*np.array([obs[5]-height,(obs[3]-turn)/10])
        return self.raw
    def applied(self,value):
        # Track actual post-slew / travel-rate-limited command in integral coordinates.
        self.integral-=DT/.25*np.linalg.solve(self.Ki,value-self.raw)
        self.integral=np.clip(self.integral,-20,20)


def scenarios():
    return [dict(name='maneuver',profile='mission',voltage=22.2,gust=0.,seed=101),
            dict(name='gust',profile='mission',voltage=22.2,gust=1.,seed=2026101501),
            dict(name='strong_gust',profile='mission',voltage=22.2,gust=1.5,seed=2026101502),
            dict(name='soft_roll_gust',profile='dynamic_roll',voltage=22.2,gust=1.,seed=2026101501),
            dict(name='low_voltage_gust',profile='mission',voltage=20.4,gust=1.,seed=2026101501),
            dict(name='voltage_ramp',profile='mission',voltage=22.2,gust=1.,seed=2026101503),
            dict(name='speed_transition',profile='mission',voltage=22.2,gust=0.,seed=101),
            dict(name='limits_recovery',profile='mission',voltage=22.2,gust=1.,seed=2026101504)]


def run_case(case,name,duration=80.):
    make=setup(MODEL)
    sys.path.insert(0,str(HERE.parents[1]/'research/archive/viewer-studies'))
    from turbulence import Gusts,SETTINGS
    p=next(p for p in plants() if p['profile']==case['profile'] and p['voltage']==case['voltage'] and p['climb']==0)
    sim=make('trout_lake',case['profile']);sim.substeps=16
    nominal=plants()[0];chart=Chart(sim,nominal['state'],float(nominal['rpm']))
    state=p['state'].copy();rpm=float(p['rpm']);state[:3]-=p['state'][:3];state[13:16]-=p['state'][:3]
    grid=dict(np.load(HERE/'grid.npz'));K=np.load(HERE/'gains.npz')['K']
    candidate=json.loads((OLD/'optimization.json').read_text())['candidate']
    family=name.replace('_observer','')
    ctrl=Scheduled(grid,K,fixed_voltage=family=='scheduled_no_voltage',fixed_speed=family=='scheduled_no_speed') if family.startswith('scheduled') else Conventional(p,BASE if family=='baseline' else candidate)
    observer=None
    if name.endswith('_observer'):
        from observer import Observer
        observer=Observer(sim,chart,grid,chart.encode(state,rpm),case['seed'])
    ref=Reference();act=Actuation(sim,p['command']);trace=[];poses=[];previous_command=p['command'].copy()
    SETTINGS['seed']=case['seed'];SETTINGS['rms_ned_m_s']=[.5*case['gust'],.5*case['gust'],.25*case['gust']];gust=Gusts()
    for j in range(round(duration/DT)+1):
        t=j*DT;wind=gust.at(t+DT/2)
        if case['name']=='limits_recovery' and t>=50:wind=np.zeros(3)
        voltage=case['voltage']
        if case['name']=='voltage_ramp':voltage=22.2-1.8*np.clip((t-15)/30,0,1)
        height=0.;turn=0.
        if case['name']=='maneuver':
            height=case.get('maneuver_height',1.)*np.clip((t-5)/2,0,1)
            turn=case.get('maneuver_turn',10.)*np.clip((t-20)/2,0,1)*np.clip((45-t)/2,0,1)
        if case['name']=='speed_transition':
            height=.5*np.clip(t-10,0,15)-.5*np.clip(t-35,0,15)
            turn=8*np.clip((t-20)/2,0,1)*np.clip((45-t)/2,0,1)
        if case['name']=='limits_recovery':
            height=10. if 10<t<30 else 0.;turn=25. if 15<t<30 else 0.
        h,climb,turn=ref.update(height,turn)
        obs=observations(state);x=chart.encode(state,rpm);speed=float(np.linalg.norm(state[7:10]-wind))
        sensed=obs
        if observer is not None:
            if hasattr(observer,'context'):
                observer.context(state,rpm,voltage,wind,previous_command)
            estimate=observer.correct(x)
            estimated_state,_=chart.decode(estimate)
            sensed=observations(estimated_state);x=estimate
            speed=float(np.linalg.norm(estimated_state[7:10]-observer.wind))
        if family.startswith('scheduled'):raw=ctrl.request(x,sensed,h,climb,turn,voltage,speed)
        else:raw=ctrl.request(sensed,h,climb,turn,voltage,speed)
        command,applied=act.apply(raw,voltage);ctrl.applied(applied)
        if observer is not None:observer.predict(command,voltage,climb,turn)
        trace.append([t,obs[5],h,obs[0],climb,obs[3],turn,np.rad2deg(obs[1]),np.rad2deg(obs[2]),
                      applied[0],applied[1],voltage,float(np.linalg.norm(state[7:10]-wind)),float(np.max(abs(raw-applied))),limits(voltage)-applied[0]])
        if j % 5 == 0:poses.append(np.r_[t,state,rpm,voltage,previous_command])
        state,rpm=sim.step(state,rpm,command,voltage=voltage,wind=wind,dt=DT)
        previous_command=command.copy()
        if not np.isfinite(state).all():raise RuntimeError((case,name,t))
    d=np.array(trace);window=(d[:,0]>=5) if case['name'] in ['maneuver','speed_transition','limits_recovery'] else (d[:,0]>=15)
    a=d[window];tail=d[d[:,0]>=duration-10]
    row=dict(case=case['name'],controller=name,height_rms=float(np.sqrt(np.mean((a[:,1]-a[:,2])**2))),
             height_iae=float(DT*np.sum(abs(a[:,1]-a[:,2]))),heading_rms=float(np.sqrt(np.mean((a[:,5]-a[:,6])**2))),
             pitch_rms=float(np.sqrt(np.mean(a[:,7]**2))),pitch_peak=float(max(abs(a[:,7]))),
             throttle_std=float(np.std(a[:,9])),brake_std=float(np.std(a[:,10])),
             limited_fraction=float(np.mean(a[:,13]>1e-6)),ceiling_fraction=float(np.mean(a[:,14]<1e-7)),
             minimum_headroom=float(min(a[:,14])),tail_height_rms=float(np.sqrt(np.mean((tail[:,1]-tail[:,2])**2))),
             speed_min=float(min(a[:,12])),speed_max=float(max(a[:,12])),finite=True,
             gains_sha256=hashlib.sha256((HERE/'gains.npz').read_bytes()).hexdigest(),
             sensing='noisy attitude/gyro 50 Hz, navigation velocity/height 5 Hz; inferred canopy/rotor/wind' if observer else 'truth state and airspeed')
    (HERE/'traces').mkdir(exist_ok=True)
    np.savez_compressed(HERE/'traces'/f'{case["name"]}-{name}.npz',trace=d,poses=np.array(poses))
    save(f'{case["name"]}-{name}.json',row);print(row,flush=True)
    return row


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('stage',choices=['prepare','screen','run']);parser.add_argument('--controller',default='scheduled');parser.add_argument('--case',default='maneuver');parser.add_argument('--duration',type=float,default=80.)
    a=parser.parse_args()
    if a.stage=='prepare':prepare()
    elif a.stage=='screen':screen()
    else:run_case(next(c for c in scenarios() if c['name']==a.case),a.controller,a.duration)
