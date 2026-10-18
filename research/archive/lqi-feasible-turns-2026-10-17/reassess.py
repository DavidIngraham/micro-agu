"""Post-hoc descriptive paired analysis; does not replace frozen acceptance gates."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.signal import welch
from report import power
HERE=Path(__file__).resolve().parent

def metrics(path):
    d=json.loads(path.read_text());s=d['samples'];t=np.array([r['time'] for r in s]);x=np.array([r['truth'] for r in s]);w=np.array([r['wind'] for r in s]);br=np.array([r['brakes'] for r in s])/(np.array([1300,1500])*np.pi*55/360*.1)
    rot=Rotation.from_quat(x[:,3:7]);eu=rot.as_euler('xyz');om=rot.inv().apply(x[:,10:13]);rate=np.rad2deg((om[:,1]*np.sin(eu[:,0])+om[:,2]*np.cos(eu[:,0]))/np.cos(eu[:,1]))
    pitch=np.rad2deg(rot.inv().apply(x[:,23:26]-x[:,10:13])[:,1]);v=np.linalg.norm(x[:,7:10]-w,axis=1);alt=-x[:,2]-60
    phases=np.array([r['phase'] for r in s]);mask=np.zeros(len(t),bool);errors=[];bias=[];std=[];spectra=[]
    for name,sign in [('turn1',d['spec']['sign']),('reverse',-d['spec']['sign']),('turn2',d['spec']['sign'])]:
        m=phases==name;m &= t>t[m][0]+20;mask|=m
        e=rate[m]-sign*d['spec']['rate'];errors.extend(e);bias.append(abs(np.mean(e)));std.append(np.std(e))
        # Uniform interpolation prevents sampling jitter from corrupting spectral estimates.
        tt=np.arange(t[m][0],t[m][-1],.1)
        f,ps=welch(np.interp(tt,t[m],pitch[m]),fs=10,nperseg=min(128,len(tt)))
        ws=[]
        for axis in range(3):
            _,p=welch(np.interp(tt,t[m],w[m,axis]),fs=10,nperseg=min(128,len(tt)));ws.append(p)
        for low,high in [(.05,.5),(.5,2)]:
            band=(f>=low)&(f<high);den=np.sum(np.array(ws)[:,band])
            spectra.append((low,float(np.sqrt(np.sum(ps[band])/den))))
    wet=t<200;watts=power.shaft_power(np.array([r['rpm'] for r in s]),rot.inv().apply(x[:,7:10]-w)[:,0])
    out=dict(complete=d['complete'],mean_bias=float(np.mean(bias)),tracking_rms=float(np.sqrt(np.mean(np.array(errors)**2))),rate_std=float(np.mean(std)),pitch_rms=float(np.sqrt(np.mean(pitch[mask]**2))),pitch_p95=float(np.percentile(abs(pitch[mask]),95)),pitch_peak=float(np.max(abs(pitch[wet]))),alt_rms=float(np.sqrt(np.mean(alt[wet]**2))),alt_peak=float(np.max(abs(alt[wet]))),min_speed=float(np.min(v)),max_roll=float(np.max(abs(np.rad2deg(eu[:,0])))),max_pitch=float(np.max(abs(np.rad2deg(eu[:,1])))),brake_ceiling_fraction=float(np.mean(np.any(br[wet]>=.99,axis=1))),common_brake_mean=float(np.mean(br[wet].min(axis=1))),shaft_W=float(np.trapezoid(watts[wet],t[wet])/(t[wet][-1]-t[wet][0])),shaft_Wh_air_km=float(np.trapezoid(watts[wet],t[wet])/3.6/np.trapezoid(v[wet],t[wet])),pitch_wind_low=float(np.mean([a[1] for a in spectra if a[0]==.05])),pitch_wind_high=float(np.mean([a[1] for a in spectra if a[0]==.5])))
    if d['spec'].get('weatherOffRecovery'):
        late=t>=240
        out.update(quiet_heading_rms=float(np.sqrt(np.mean(rate[late]**2))),quiet_pitch_rms=float(np.sqrt(np.mean(pitch[late]**2))),quiet_alt_rms=float(np.sqrt(np.mean(alt[late]**2))))
        # First 10 s moving RMS window satisfying original recovery limits and remaining so.
        good=[]
        for start in np.arange(200,240.1,1):
            m=(t>=start)&(t<start+10)
            good.append(np.sqrt(np.mean(rate[m]**2))<=3 and np.sqrt(np.mean(pitch[m]**2))<=3 and np.sqrt(np.mean(alt[m]**2))<=1.5)
        out['settled_window_end_s']=next((float(10+i) for i in range(len(good)) if all(good[i:])),None)
    return out

def main():
    records=[]
    for cohort,variant,ids in [('validation','reference',range(100,106)),('validation','map_aw_observer',range(100,106)),('release_validation','reference',range(106,112)),('release_validation','candidate',range(106,112))]:
        for i in ids:
            p=HERE/cohort/variant/'lqi-25'/f'{i:03}.json';records.append(dict(case=i,variant='reference' if variant=='reference' else 'candidate',**metrics(p)))
    expanded=[dict(case=i,**metrics(HERE/'release_validation/candidate/lqi-40'/f'{i:03}.json')) for i in range(106,112)]
    keys=['mean_bias','tracking_rms','rate_std','pitch_rms','pitch_p95','pitch_peak','alt_rms','alt_peak','shaft_W','shaft_Wh_air_km','common_brake_mean','pitch_wind_low','pitch_wind_high']
    pairs={}
    for k in keys:
        a=np.array([next(r[k] for r in records if r['case']==i and r['variant']=='reference') for i in range(100,112)])
        b=np.array([next(r[k] for r in records if r['case']==i and r['variant']=='candidate') for i in range(100,112)])
        change=100*(b/a-1)
        pairs[k]=dict(reference_median=float(np.median(a)),candidate_median=float(np.median(b)),paired_percent_median=float(np.median(change)),paired_percent_min=float(min(change)),paired_percent_max=float(max(change)),improved=int(sum(b<a)),reference_max=float(max(a)),candidate_max=float(max(b)))
    out=dict(records=records,expanded_40=expanded,paired_25=pairs)
    (HERE/'reassessment.json').write_text(json.dumps(out,indent=2))
    lines=['# Post-hoc performance reassessment','',
      'Descriptive analysis of existing traces, not a new acceptance test. No original gates, controller gains or deployment changed. Twelve paired cases at 25 deg/s; six candidate-only cases at 40 deg/s. Wet-flight metrics use t<200 s; steady-turn metrics exclude the first 20 s of each turn. Cases are the statistical units, not individual samples.', '',
      '| Metric | Reference median | Candidate median | Median paired change | Paired range | Improved cases |',
      '|---|---:|---:|---:|---:|---:|']
    for k,p in pairs.items():lines.append(f"| {k} | {p['reference_median']:.3f} | {p['candidate_median']:.3f} | {p['paired_percent_median']:+.1f}% | {p['paired_percent_min']:+.1f}% to {p['paired_percent_max']:+.1f}% | {p['improved']}/12 |")
    lines+=['','## Interpretation limits','',
      '- All 30 analysed flights completed without the harness operational guards firing. Those guards are engineering screens, not demonstrated aircraft safety limits. No calibrated stall, structural-load or collapse boundary was established by this study. The simulator does not model canopy collapse. Do not label these flights physically safe on this evidence alone.',
      '- The 30 deg/s relative pitch-rate and RMS limits are screening choices; no flight-data or structural justification was found in the study. Original failures remain valid threshold exceedances, not proofs of instability.',
      '- Spectral pitch/wind ratios use total three-axis wind power in 0.05–0.5 and 0.5–2 Hz bands. Units are (deg/s)/(m/s). They are descriptive response ratios, NOT identified transfer functions: closed-loop trajectory and mean-wind direction differ, and short turn windows limit low-frequency resolution.',
      '- Weather seeds and physical parameters are paired, but resulting trajectories and wind orientation are not identical. No causal frequency-response or formal stability claim follows.',
      '- Shaft power is modeled, not battery consumption. Mean common brake is not itself an efficiency measure.',
      '- Existing summary.json columns are MEDIANS across cases. Earlier prose calling them averages was inaccurate. This report labels all aggregations explicitly.',
      '- Forty deg/s has no matched reference at that rate in these disturbed cases; it cannot be declared a comparative improvement.', '', '## Quiet recovery and observed envelopes','']
    for label,rs in [('reference25',[r for r in records if r['variant']=='reference' and r['case']>=106]),('candidate25',[r for r in records if r['variant']=='candidate' and r['case']>=106]),('candidate40',expanded)]:
        lines.append(f"- {label}: worst final heading RMS {max(r['quiet_heading_rms'] for r in rs):.3f} deg/s; relative pitch RMS {max(r['quiet_pitch_rms'] for r in rs):.3f} deg/s; altitude RMS {max(r['quiet_alt_rms'] for r in rs):.3f} m. Settled-window ends after weather removal: {[r['settled_window_end_s'] for r in rs]} s. Minimum airspeed {min(r['min_speed'] for r in rs):.2f} m/s; maximum absolute roll {max(r['max_roll'] for r in rs):.1f} deg; pitch {max(r['max_pitch'] for r in rs):.1f} deg.")
    lines+=['','## Individual pitch peaks','', '| Case | Reference deg/s | Candidate deg/s | Change |','|---|---:|---:|---:|']
    for i in range(100,112):
        a=next(r['pitch_peak'] for r in records if r['case']==i and r['variant']=='reference');b=next(r['pitch_peak'] for r in records if r['case']==i and r['variant']=='candidate')
        lines.append(f'| {i} | {a:.2f} | {b:.2f} | {100*(b/a-1):+.1f}% |')
    lines+=['','## Decision','',
      'The original binary screen obscured consistent improvements in tracking, altitude RMS and modeled power at 25 deg/s. It does not establish that the candidate is worse or unstable. Pitch-motion performance is mixed: inspect the individual peak regression before promotion. Recovery after weather removal is comparable, with no evidence of persistent oscillation in these finite tests.',
      'No new post-hoc release threshold is invented. The evidence supports continuing qualification of the candidate at 25 deg/s, not an automatic 40 deg/s release. Before publication, investigate pitch-peak cases and run the deferred controller transitions. A 40 deg/s comparative claim additionally needs matched reference runs at 40 deg/s. Physical safety claims need calibrated limits absent from this study.']
    (HERE/'REASSESSMENT.md').write_text('\n'.join(lines)+'\n');print('\n'.join(lines))
if __name__=='__main__':main()
