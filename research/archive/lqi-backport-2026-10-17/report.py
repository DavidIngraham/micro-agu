"""Backport acceptance is evaluated within each unchanged-gain/observer cohort."""
import importlib.util
import json
from pathlib import Path
import numpy as np
from score import analyze_phase
HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('mission_report',HERE.parent/'lqi-balanced-2026-10-17/report.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)

def main():
    training=analyze_phase(HERE/'training');valid=analyze_phase(HERE/'validation')
    missions=[m.mission_metrics(p) for p in (HERE/'missions').glob('*/*.json') if p.stem.isdigit()]
    (HERE/'missions/metrics.json').write_text(json.dumps(missions,indent=2))
    result={};lines=['# Original-observer LQI reference-filter backport','',
      'AI-assisted. Existing gains, original observer, no pitot or canopy sensing. Rate limits remain ±0.5 m/s. Only the climb-reference filter changes from 0 to 0.7 s.','']
    for cohort in ['local','deployed']:
        reasons=[];comparisons={}
        for phase,rows,count in [('training',training,3),('validation',valid,8)]:
            off={r['case']:r for r in rows if r['variant']==cohort+'_off'}
            on={r['case']:r for r in rows if r['variant']==cohort+'_on'}
            if len(off)!=count or set(on)!=set(off):reasons.append(phase+' missing pairs');continue
            if any(r['score'] is None for r in list(off.values())+list(on.values())):
                reasons.append(phase+' operational failure');continue
            changes={k:float(np.median([100*(fn(on[i])/fn(off[i])-1) for i in off])) for k,fn in {
                'score':lambda r:r['score'], 'energy':lambda r:r['shaft_Wh_per_ground_km'],
                'acquisition_time':lambda r:r['steps']['climb']['rise_10_90_s'],
                'climb_peak':lambda r:r['command_peak_rates_deg_s'][0],
                'descent_peak':lambda r:r['command_peak_rates_deg_s'][1],
                **{task:(lambda r,k=task:r['tasks'][k]) for task in ['acquisition','descent','updraft','crosswind']}}.items()}
            comparisons[phase]=changes
            if changes['score']> -5:reasons.append(phase+' score improves less than 5%')
            if changes['energy']>5:reasons.append(phase+' energy regression >5%')
            if changes['acquisition_time']>10:reasons.append(phase+' acquisition regression >10%')
            for task in ['acquisition','descent','updraft','crosswind']:
                if changes[task]>20:reasons.append(phase+' '+task+' regression >20%')
        for mode in ['AUTO','CRUISE']:
            off={r['case']:r for r in missions if r['variant']==cohort+'_off' and r['mode']==mode}
            on={r['case']:r for r in missions if r['variant']==cohort+'_on' and r['mode']==mode}
            if len(off)!=4 or set(on)!=set(off):reasons.append(mode+' missing pairs');continue
            if any(not r['complete'] or r.get('screen_failures') for r in list(off.values())+list(on.values())):
                reasons.append(mode+' operational failure');continue
            for metric in ['altitude_rms_m','xtrack_rms_m']:
                ratio=np.median([on[i]['disturbed'][metric]/max(off[i]['disturbed'][metric],1) for i in off])
                if ratio>1.2:reasons.append(mode+' '+metric+' regression >20%')
            if np.median([on[i]['shaft_W']/off[i]['shaft_W'] for i in off])>1.05:reasons.append(mode+' shaft power regression >5%')
        result[cohort]=dict(accepted=not reasons,reasons=reasons,comparisons=comparisons)
        lines += [f'## {cohort}', '', 'Acceptance: '+('PASS' if not reasons else 'FAIL: '+ '; '.join(reasons)), '',
          '| Variant | Complete / screens pass | Score | Climb relative pitch-rate peak °/s | Descent relative pitch-rate peak °/s | Climb settling s | Updraft recovery s | Shaft Wh/km |',
          '|---|---:|---:|---:|---:|---:|---:|---:|']
        for suffix in ['off','on']:
            rr=[r for r in valid if r['variant']==cohort+'_'+suffix];good=[r for r in rr if r['complete']]
            values=[]
            for fn in [lambda r:r['score'],lambda r:r['command_peak_rates_deg_s'][0],lambda r:r['command_peak_rates_deg_s'][1],
                       lambda r:r['steps']['climb']['settling_s'],lambda r:r['gusts']['vertical']['recovery_after_off_s'],lambda r:r['shaft_Wh_per_ground_km']]:
                vals=[fn(r) for r in good];vals=[v for v in vals if v is not None]
                values.append(f'{np.median(vals):.3f}' if vals else 'n/a')
            lines.append(f'| {suffix} | {len(good)}/{sum(r["score"] is not None for r in rr)} | '+' | '.join(values)+' |')
        lines += ['', 'Median paired changes (%):', '', '```json',json.dumps(comparisons,indent=2),'```','',
          'Validation paired change distribution (negative is lower; eight cases):','',
          '| Metric | Minimum | Median | Maximum | Improved cases |',
          '|---|---:|---:|---:|---:|']
        off={r['case']:r for r in valid if r['variant']==cohort+'_off' and r['score'] is not None}
        on={r['case']:r for r in valid if r['variant']==cohort+'_on' and r['score'] is not None}
        for label,fn in [('Score',lambda r:r['score']),('Shaft Wh/km',lambda r:r['shaft_Wh_per_ground_km']),
                         ('Climb rate peak',lambda r:r['command_peak_rates_deg_s'][0]),
                         ('Descent rate peak',lambda r:r['command_peak_rates_deg_s'][1])]:
            delta=[100*(fn(on[i])/fn(off[i])-1) for i in off.keys() & on.keys()]
            if delta:lines.append(f'| {label} | {min(delta):+.2f}% | {np.median(delta):+.2f}% | {max(delta):+.2f}% | {sum(x<0 for x in delta)}/{len(delta)} |')
        lines += ['', 'Mission medians (four cases per mode and variant):','',
          '| Mode | Filter | Altitude RMS m | Cross-track RMS m | Relative rate RMS °/s | Shaft W | Screens passed |',
          '|---|---|---:|---:|---:|---:|---:|']
        for mode in ['AUTO','CRUISE']:
            for suffix in ['off','on']:
                rr=[r for r in missions if r['variant']==cohort+'_'+suffix and r['mode']==mode and r['complete']]
                if not rr:continue
                vals=[np.median([r['disturbed'][k] for r in rr]) for k in ['altitude_rms_m','xtrack_rms_m','relative_rate_rms_deg_s']]
                vals.append(np.median([r['shaft_W'] for r in rr]))
                lines.append(f'| {mode} | {suffix} | '+' | '.join(f'{v:.3f}' for v in vals)+f' | {sum(not r["screen_failures"] for r in rr)}/{len(rr)} |')
        lines.append('')
    lines+=['## Limits','','Finite scenario screens are not formal stability margins. The two cohorts are separately controlled experiments: local includes pre-existing zero-trim and EKF work, deployed uses published AP base revision 010ada6 plus only this filter patch. No enhanced-state gains or extra sensing are transferred. Shaft power is a quasi-steady model, not measured electrical consumption.', '',
        'Acceptance uses medians of paired ratios; these need not equal ratios of the tabulated medians. Mission tracking uses a 1 m denominator floor. Individual regressions remain visible in [gust details](GUSTS.md) and raw metrics. The score weights task error 65%, motion 20%, actuator use 10%, and modeled energy 5%; it is a comparison index, not a measured safety margin.', '',
        '![All validation pairs](validation-pairs.png)']
    (HERE/'acceptance.json').write_text(json.dumps(result,indent=2))
    (HERE/'RESULTS.md').write_text('\n'.join(lines)+'\n');print(json.dumps(result,indent=2))

if __name__=='__main__':main()
