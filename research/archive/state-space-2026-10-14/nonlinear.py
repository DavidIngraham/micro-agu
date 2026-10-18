"""Matched nonlinear checks with actuator limits; truth sensors, not native SITL."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
from study import HERE, TRACES, DT, XS, US, BASE, Chart, observations, plants, setup, save, controller
from analyze import lqi, COLORS
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def run(model, selected=None):
    TRACES.mkdir(parents=True,exist_ok=True)
    make=setup(model)
    from microagu_sim.compat.actuator import BrakeActuator
    sys_path=Path(__file__).resolve().parents[3]/'research/archive/viewer-studies'
    import sys
    sys.path.insert(0,str(sys_path))
    from turbulence import Gusts, SETTINGS
    candidate=json.loads((HERE/'optimization.json').read_text())['candidate']
    effort=json.loads((HERE/'optimization-effort.json').read_text())['candidate']
    configs={'baseline':BASE,'candidate':candidate,'effort':effort}
    ps=plants(); nominal=ps[0]
    oracle_cfg=json.loads((HERE/'oracle-selected.json').read_text()) if selected=='oracle_selected' else {}
    _,_,_,_,K=lqi(nominal,**oracle_cfg)
    rows=[]; validations=[]
    suffix='-'+selected if selected else ''
    scenarios=[('small_ramp',0.,20261014),('gust_holdout',1.,2026101401),
               ('strong_gust',1.5,2026101402),('maneuver',0.,20261014)]
    for p in [ps[0],ps[4],ps[5],ps[3]]:
        for scenario,strength,seed in scenarios:
            if scenario=='small_ramp' and p['name']!='level':continue
            for name in ['baseline','candidate','effort','oracle','oracle_selected']:
                if name=='oracle_selected' and selected!='oracle_selected':continue
                if selected and name!=selected:continue
                sim=make('trout_lake',p['profile']);sim.substeps=16
                chart=Chart(sim,p['state'],float(p['rpm'])); state=chart.base.copy();rpm=float(p['rpm'])
                command=p['command'];act=BrakeActuator(position_mm=command[1:]);z=np.zeros(8)
                iz=np.zeros(2); prev=np.zeros(2);trace=[]; xlin=np.zeros(24)
                ratio=22.2/p['voltage'];limit=int(min(60*ratio,100))/100
                strokes=np.array([1300,1500])*np.pi*55/360*.1;u0=command[2]/strokes[1]
                base_obs=observations(state);cfg=configs.get(name,BASE)
                baseff=.45+p['climb']*(limit-.45 if p['climb']>=0 else .18)
                initial_integral=command[0]/ratio-baseff;lastpd=baseff
                SETTINGS['seed']=seed;SETTINGS['rms_ned_m_s']=[.5*strength,.5*strength,.25*strength];gust=Gusts()
                F,G,W,U,Ur=controller(p,cfg);linear_trace=[];sat_count=0
                for j in range(round(120/DT)+1):
                    t=j*DT;obs=observations(state);wind=gust.at(t+DT/2) if t<80 else np.zeros(3)
                    ramp=.001 if scenario=='small_ramp' else (1. if scenario=='maneuver' else 0.)
                    href=ramp*np.clip((t-5)/2,0,1)
                    turn=10.*np.clip((t-20)/2,0,1)*np.clip((45-t)/2,0,1) if scenario=='maneuver' else 0.
                    ref=np.array([href,turn]);hrelative=obs[5]-p['climb']*t
                    if name.startswith('oracle'):
                        x=chart.encode(state,rpm);x[15]-=p['climb']*t
                        xa=np.r_[x/XS,prev,iz];du=-K@xa;u=prev+du
                        raw=command[0]+US[0]*u[0]
                        throttle=np.clip(raw,0,limit)
                        throttle=np.clip(throttle,command[0]+US[0]*prev[0]-DT*limit,command[0]+US[0]*prev[0]+DT*limit)
                        brake=np.clip(u0+US[1]*u[1],-1,1)
                        if 0<raw<limit and abs(u0+US[1]*u[1])<1:
                            iz+=DT*np.array([hrelative-href,(obs[3]-turn)/10])
                        prev=np.array([(throttle-command[0])/US[0],(brake-u0)/US[1]])
                    else:
                        aq=DT/(DT+1/(2*np.pi*cfg['qhz']));al=DT/(DT+1/(2*np.pi*cfg['lat_hz']))
                        z[:2]+=al*(obs[3:5]-base_obs[3:5]-z[:2])
                        # The tested references stay below turn acceleration/rate limits.
                        z[7]+=DT/(DT+.1)*(turn-z[7]);error=z[7]-z[0]
                        integ=np.clip(z[2]+u0+cfg['ki']*DT*error,-.15,.15)
                        sv=np.clip(5/max(np.linalg.norm(state[7:10]-wind),2),.25,2.5)
                        raw=sv*(.009*z[7]+cfg['kp']*error-cfg['rd']*z[1])+integ
                        if (raw<=1 or error<=0) and (raw>=-1 or error>=0):z[2]=integ-u0
                        brake=np.clip(sv*(.009*z[7]+cfg['kp']*error-cfg['rd']*z[1])+z[2]+u0,-1,1)
                        z[6]+=DT/(DT+.5)*(href-z[6]);rate=(href-z[6])/.5
                        vd=np.clip(p['climb']+cfg['kh']*(href-hrelative)+rate,-2.5,1.)
                        ev=vd-obs[0];kv=cfg['kv']*limit/.6
                        slope=limit-.45 if vd>=0 else .18
                        # Smooth branch for the local map equivalence check only.
                        if scenario=='small_ramp':slope=limit-.45
                        ff=.45+vd*slope;pd=np.clip(ff+kv*ev,0,limit)
                        pd=np.clip(pd,lastpd-DT*limit,lastpd+DT*limit);lastpd=pd
                        integ=z[3]+initial_integral+.3*kv*DT*ev
                        integ=np.clip(integ,np.clip(-pd-.1,-limit/2,limit/2),np.clip(limit-pd+.1,-limit/2,limit/2))
                        z[3]=integ-initial_integral
                        z[4]+=aq*(obs[1]-base_obs[1]-z[4]);z[5]+=aq*(z[4]-z[5])
                        throttle=np.clip(ratio*np.clip(pd+integ+np.clip(-cfg['qgain']*z[5],-.25,.25),0,limit),0,limit)
                    cmd=np.array([throttle,max(-brake,0)*strokes[0],max(brake,0)*strokes[1]])
                    sat_count+=int(throttle<=1e-8 or throttle>=limit-1e-8 or abs(brake)>=1-1e-8)
                    pitch=R.from_quat(state[3:7]).as_euler('xyz')[1]
                    trace.append([t,hrelative,obs[0]-p['climb'],pitch*180/np.pi,obs[1]*180/np.pi,
                                  obs[2]*180/np.pi,obs[3],throttle,brake,href,turn])
                    if scenario=='small_ramp' and not name.startswith('oracle'):
                        linear_trace.append(np.r_[p['C']@xlin[:16], U@xlin+Ur@ref])
                        xlin=F@xlin+G@ref
                    state,rpm,_=sim.step_actuated(state,rpm,act,cmd,voltage=p['voltage'],wind=wind,dt=DT)
                    if not np.isfinite(state).all():raise RuntimeError((p['name'],scenario,name,t))
                data=np.array(trace);use=(data[:,0]>=15)&(data[:,0]<80)
                row=dict(plant=p['name'],scenario=scenario,controller=name,
                         height_rmse=float(np.sqrt(np.mean((data[use,1]-data[use,9])**2))),
                         climb_rms=float(np.sqrt(np.mean(data[use,2]**2))),pitch_std=float(np.std(data[use,3])),
                         pitch_rate_rms=float(np.sqrt(np.mean(data[use,4]**2))),relative_rate_rms=float(np.sqrt(np.mean(data[use,5]**2))),
                         heading_rmse=float(np.sqrt(np.mean((data[use,6]-data[use,10])**2))),
                         throttle_std=float(np.std(data[use,7])),brake_std=float(np.std(data[use,8])),
                         saturation_fraction=sat_count/len(data))
                rows.append(row);print(row,flush=True)
                np.savez_compressed(TRACES/f'{p["name"]}-{scenario}-{name}.npz',trace=data)
                if linear_trace:
                    lin=np.array(linear_trace);actual=data[:,[1,2,4,7,8]].copy();actual[:,3:]-=np.array([command[0],u0])
                    expected=lin[:,[5,0,1,6,7]].copy();expected[:,2]*=180/np.pi
                    validations.append(dict(controller=name,relative_l2_errors=(np.linalg.norm(actual-expected,axis=0)/np.maximum(np.linalg.norm(expected,axis=0),1e-10)).tolist()))
                save('nonlinear-summary'+suffix+'.json',rows)
    save('linear-validation'+suffix+'.json',dict(outputs=['height','climb','payload q','throttle','brake'],
         note='1 mm ramp, smooth climb feedforward branch; includes nonlinear physics and travel-rate actuator.',rows=validations))
    if selected:return
    plot()

def plot():
    fig,ax=plt.subplots(3,2,figsize=(13,10),layout='constrained')
    for col,scenario in enumerate(['gust_holdout','maneuver']):
        for name,color in zip(['baseline','candidate','oracle_selected'],COLORS):
            d=np.load(TRACES/f'level-{scenario}-{name}.npz')['trace']
            for a,k in zip(ax[:,col],[1,4,7]):a.plot(d[:,0],d[:,k],label='Full-state LQI' if name=='oracle_selected' else name,color=color,lw=1)
        ax[0,col].set_title('Held-out gust phase' if col==0 else '1 m climb and 10 deg/s turn request')
        if col==1:ax[0,col].plot(d[:,0],d[:,9],'k--',lw=1,label='Height request')
        for a,lab in zip(ax[:,col],['Altitude change (m)','Payload pitch rate (deg/s)','Throttle fraction']):
            a.set_ylabel(lab);a.grid(alpha=.2)
            if col==0:a.axvline(80,color='gray',ls='--',lw=.8)
        ax[0,col].legend(fontsize=9);ax[2,col].set_xlabel('Time (s)')
    fig.suptitle('Matched nonlinear checks: truth sensors, motor dynamics and actuator limits')
    fig.savefig(HERE/'nonlinear.png',dpi=160);plt.close(fig)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--model',type=Path,required=True);p.add_argument('--controller')
    a=p.parse_args();run(a.model,a.controller)
