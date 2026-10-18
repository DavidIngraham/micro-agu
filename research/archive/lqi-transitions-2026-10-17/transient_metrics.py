"""Settled-flight command and isolated-gust responses; no new tuning."""
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from holistic import settling, longest, rms

HERE = Path(__file__).resolve().parent
OUT = HERE/'holistic/transients-valid'


def assess(path):
    d = json.loads(path.read_text())
    r = dict(variant=d['spec']['variant'], case=d['spec']['id'], complete=d['complete'], failure=d['failure'],
             input=str(path.relative_to(HERE)),sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    if not d['complete']:
        return r, None
    rows = d['samples']
    ts = np.array([s['time'] for s in rows]); y = np.array([s['truth'] for s in rows])
    assert np.all(np.diff(ts)>0) and np.max(np.diff(ts)) < .3
    t = np.arange(.2,319.8,.1)
    grid = lambda a: np.interp(t, ts, a)
    alt=grid(-y[:,2]); target=grid([s['requestedAltitude'] for s in rows])
    rp,rc=R.from_quat(y[:,3:7]),R.from_quat(y[:,16:20])
    relative_rate=grid(rp.inv().apply(y[:,23:26]-y[:,10:13])[:,1]*180/np.pi)
    quiet=np.sqrt(np.convolve(relative_rate**2,np.ones(20)/20,'full')[:len(t)])<=2
    pitch=grid(rp.as_euler('xyz',degrees=True)[:,1])
    speed=grid(np.linalg.norm(y[:,7:10]-np.array([s['wind'] for s in rows]),axis=1))
    commands=np.array([s['command'] for s in rows])
    throttle=grid(commands[:,0]); brakes=np.array([s['brakes'] for s in rows])/(np.array([1300,1500])*np.pi*55/360*.1)
    high=grid(np.any(brakes>.99,axis=1).astype(float))>.5
    event={e['name']:e['time'] for e in d['events']}
    steps={}
    for name,key,end,goal,initial,sign in [('climb','climb-request',120,70,60,1),('descent','descent-request',220,60,70,-1)]:
        start=event[key]; m=(t>=start)&(t<end)
        tt=t[m]; height=alt[m]; error=height-goal
        response=sign*(height-initial)
        times=[tt[np.flatnonzero(response>=v)[0]]-start if np.any(response>=v) else None for v in [1,9]]
        settle=settling(tt,(abs(error)<=.5)&quiet[m],start=start,hold=10)
        steps[name]=dict(rise_10_90_s=float(times[1]-times[0]) if None not in times else None,
            command_to_10_s=float(times[0]) if times[0] is not None else None,
            command_to_90_s=float(times[1]) if times[1] is not None else None,
            settling_s=settle, iae_m_s=float(np.trapezoid(abs(error),tt)),
            overshoot_m=float(max(0,np.max(sign*error))),
            final_error_m=float(np.mean(error[-100:])), mean_speed_m_s=float(np.mean(speed[m])),
            throttle_ceiling_fraction=float(np.mean(throttle[m]>=.594)),
            max_relative_pitch_rate_deg_s=float(np.max(abs(relative_rate[m]))))
    gusts={}
    for name,on,off,end in [('vertical','vertical-gust-on','vertical-gust-off',260),('crosswind','crosswind-gust-on','crosswind-gust-off',319.8)]:
        start,stop=event[on],event[off]; m=(t>=start)&(t<end); tm=t[m]
        before=(t>=start-5)&(t<start)
        excursion=alt[m]-float(np.mean(alt[before]))
        pitch_excursion=pitch[m]-float(np.mean(pitch[before]))
        recovery=settling(tm,(abs(excursion)<=.5)&quiet[m],start=stop,hold=5)
        gusts[name]=dict(peak_height_excursion_m=float(np.max(abs(excursion))),
            altitude_iae_m_s=float(np.trapezoid(abs(excursion),tm)),
            peak_pitch_excursion_deg=float(np.max(abs(pitch_excursion))),
            recovery_after_off_s=recovery, relative_pitch_rate_rms_deg_s=rms(relative_rate[m]),
            final_height_error_m=float(np.mean(alt[m][-100:])-60))
    # Sensor validity is asserted on actual AP diagnostics for the LQI cases.
    if d['scheme']=='lqi':
        diag=np.array([x['values'] for x in d['diagnostics']])
        assert diag.shape[1]==43 and np.all(diag[:,42]==1)
        expected=4 if d['spec'].get('sensor')=='truth' else 3
        modes={int(line.split(',')[2]) for line in d['logs'] if line.startswith('LQIP,')}
        assert modes=={expected}, modes
        r['lqi_target_tas_m_s']=float(np.median(diag[:,6]))
    r.update(steps=steps,gusts=gusts,settled_after_s=d['settledAfter'],
        initial_error_m=float(np.mean(alt[t<20])-60),
        mean_truth_speed_m_s=float(np.mean(speed)),
        mean_pitot_eas_m_s=float(np.mean([s['pitotEas'] for s in rows])),
        mean_tecs_speed_error_m_s=float(np.mean([s['tecsSpeedError'] for s in rows])),
        full_brake_longest_s=longest(high),max_pitch_deg=float(np.max(abs(pitch))))
    return r,(t,alt,target,pitch)


def main():
    records=[]; traces={}
    for variant in ['standard','airspeed','sensor','damping']:
        for i in [1,3,13]:
            r,trace=assess(OUT/variant/f'{i:02}.json')
            records.append(r); traces[variant,i]=trace
    (OUT/'metrics.json').write_text(json.dumps(records,indent=2,allow_nan=False)+'\n')
    lines=['# Settled-flight supplemental tests','','AI-assisted analysis. Twelve additional actual AP/EKF-in-loop runs; '
        'three physics configurations (nominal and two retained perturbations), four controller/sensing variants. '
        'No gain changes. All use CRUISE, start at 60 m after ten seconds within 0.5 m and 0.1 m/s vertical speed, '
        'then +10 m at 20 s, -10 m at 120 s, a 1 m/s updraft at 220–225 s, and 2 m/s westward wind at 260–265 s. '
        'Requests, ACKs and actual event times are retained. Finite sensor noise remains active.', '',
        '**Reference limitation:** an initial attempt to set AIRSPEED_MIN=3.8 failed the firmware arming check '
        '(minimum 5). Those nine preflight failures are retained in ../transients and are not flight outcomes. '
        'The valid runs use AIRSPEED_CRUISE=5, AIRSPEED_MIN=5, AIRSPEED_MAX=7. LQI internally clamps TAS to 4.4 m/s. '
        'The standard controller runs TECS_PG_ENABLE=1: its paraglider branch controls height with throttle '
        'and pitch-rate damping, explicitly sets _using_airspeed_for_throttle=false, and assumes nearly constant '
        'natural airspeed. It returns before ordinary TECS speed regulation. Consequently NAV aspd_error '
        'is not a meaningful tracking error here (its reported target is zero). Raw telemetry is retained '
        'but excluded from scoring. These are actual operating responses, not equal-speed controller '
        'transfer-function tests; achieved speed is reported below.', '',
        '| Controller | Completed | Initial altitude error m | Mean truth speed m/s | Mean pitot EAS m/s |',
        '|---|---:|---:|---:|---:|']
    variants=['standard','airspeed','sensor','damping']
    for v in variants:
        rr=[r for r in records if r['variant']==v and r['complete']]
        med=lambda k:np.median([r[k] for r in rr])
        lines.append(f'| {v} | {len(rr)}/3 | {med("initial_error_m"):.3f} | {med("mean_truth_speed_m_s"):.3f} | '
            f'{med("mean_pitot_eas_m_s"):.3f} |')
    lines+=['','## Altitude command response','','Scenario medians (worst of three in parentheses). '
        'IAE is integral absolute truth error relative to requested altitude. Settling requires ±0.5 m '
        'and relative pitch-rate <=2 °/s for ten seconds; right-censored cases stay null.', '',
        '| Step | Controller | 10–90% time s | Settling time s | IAE m·s | Overshoot m |',
        '|---|---|---:|---:|---:|---:|']
    for step in ['climb','descent']:
        for v in variants:
            rr=[r['steps'][step] for r in records if r['variant']==v and r['complete']]
            entries=[]
            for k in ['rise_10_90_s','settling_s','iae_m_s','overshoot_m']:
                a=[r[k] for r in rr if r[k] is not None]
                entries.append(f'{np.median(a):.2f} ({max(a):.2f})'+(f' [{len(rr)-len(a)} censored]' if len(a)<len(rr) else '') if a else 'censored')
            lines.append(f'| {step} | {v} | '+' | '.join(entries)+' |')
    lines+=['','## Isolated gust response','','Recovery is after gust removal, relative to pre-gust height, '
        'within ±0.5 m and relative pitch-rate <=2 °/s for five seconds. Gusts are abrupt imposed wind changes; '
        'they do not identify physical turbulence spectra.', '',
        '| Gust | Controller | Peak height excursion m | IAE m·s | Peak pitch excursion ° | Recovery s |',
        '|---|---|---:|---:|---:|---:|']
    for gust in ['vertical','crosswind']:
        for v in variants:
            rr=[r['gusts'][gust] for r in records if r['variant']==v and r['complete']]
            vals=[]
            for k in ['peak_height_excursion_m','altitude_iae_m_s','peak_pitch_excursion_deg','recovery_after_off_s']:
                a=[r[k] for r in rr if r[k] is not None]
                vals.append(f'{np.median(a):.2f} ({max(a):.2f})'+(f' [{len(rr)-len(a)} censored]' if len(a)<len(rr) else '') if a else 'censored')
            lines.append(f'| {gust} | {v} | '+' | '.join(vals)+' |')
    lines+=['','## Integrity and scope','',
        'Actual LQI pitot-use and proprioception modes are asserted for each successful run. '
        'Firmware hashes and restoration verification are retained alongside results. '
        'ArduPilot sources are untouched; five viewer files were temporarily replaced from archived artifacts. '
        'These three chosen physics cases are diagnostic, not a new Monte Carlo or population stability claim.', '',
        '![All supplemental trajectories](responses.png)']
    (OUT/'REPORT.md').write_text('\n'.join(lines)+'\n')
    fig,axes=plt.subplots(3,3,figsize=(13,9),constrained_layout=True)
    for row,i in enumerate([1,3,13]):
        for v,color in zip(variants,['#c89337','#999999','#2b9a82','#4b80b5']):
            trace=traces[v,i]
            if trace is None:continue
            t,alt,target,pitch=trace
            for column,(lo,hi) in enumerate([(0,220),(215,260),(255,320)]):
                m=(t>=lo)&(t<hi)
                axes[row,column].plot(t[m],alt[m],label=v,color=color,lw=.9)
                if v=='standard':axes[row,column].plot(t[m],target[m],'k--',lw=.6)
                axes[row,column].set_title(f'Physics {i:02}: '+['Altitude commands','Updraft pulse','Crosswind pulse'][column])
                axes[row,column].set_ylabel('Truth altitude (m)');axes[row,column].set_xlabel('Time (s)');axes[row,column].grid(alpha=.2)
    axes[0,0].legend(fontsize=8)
    fig.savefig(OUT/'responses.png',dpi=150);plt.close(fig)
    print('\n'.join(lines))


if __name__=='__main__':main()
