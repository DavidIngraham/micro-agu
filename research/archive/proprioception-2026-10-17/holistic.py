"""Retrospective, AI-assisted flight-quality assessment. No controller selection.

Run with the existing venv-ardupilot Python. Reads immutable campaign outputs.
Thresholds are engineering screening assumptions, not certified requirements.
"""
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import detrend, welch
from scipy.spatial.transform import Rotation as R

HERE = Path(__file__).resolve().parent
OUT = HERE / 'holistic'
TRAIN = {0, 1, 6, 9}
DT = .1
WEIGHTS = np.array([.35, .30, .20, .10, .05])
COMPONENTS = ['tracking', 'recovery', 'internal_motion', 'actuator', 'energy']
NAMES = {'standard': 'Standard + pitot', 'sensor': 'LQI + both sensors',
         'damping': 'Retuned LQI + both', 'airspeed': 'LQI + pitot only'}


def rms(x):
    return float(np.sqrt(np.mean(np.asarray(x)**2)))


def longest(mask):
    """Duration on the uniform sample grid, including terminal intervals."""
    edges = np.diff(np.r_[False, mask, False].astype(int))
    starts, ends = np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)
    return float(max(ends-starts, default=0)*DT)


def settling(t, good, start=100., hold=10.):
    """First full hold interval in tolerance; None means right-censored."""
    n = round(hold/DT)
    window = np.convolve(good.astype(int), np.ones(n, dtype=int), 'valid')
    eligible = np.flatnonzero((t[:len(window)] >= start) & (window == n))
    return float(t[eligible[0]]-start) if len(eligible) else None


def errors(x):
    return dict(rms=rms(x), p95=float(np.quantile(abs(x), .95)),
                peak=float(np.max(abs(x))), bias=float(np.mean(x)))


def band_rms(x):
    f, p = welch(detrend(x), fs=1/DT, nperseg=min(256, len(x)))
    mask = (f >= .1) & (f <= 2)
    return float(np.sqrt(np.trapezoid(p[mask], f[mask])))


def analyze(path, variant, old):
    d = json.loads(path.read_text())
    samples = d['samples']
    ts = np.array([s['time'] for s in samples])
    assert np.all(np.diff(ts) > 0) and np.max(np.diff(ts)) < .3
    assert ts[0] <= .2 and ts[-1] >= 179.8
    t = np.arange(.2, 179.8, DT)
    def grid(x):
        a = np.asarray(x)
        if a.ndim == 1:
            return np.interp(t, ts, a)
        return np.stack([np.interp(t, ts, c) for c in a.T], axis=1)
    state = np.array([s['truth'] for s in samples])
    rp, rc = R.from_quat(state[:, 3:7]), R.from_quat(state[:, 16:20])
    attitude = grid(rp.as_euler('xyz', degrees=True))
    relative = grid((rp.inv()*rc).as_rotvec()*180/np.pi)
    rel_rate = grid(rp.inv().apply(state[:, 23:26]-state[:, 10:13])*180/np.pi)
    rates = grid(rp.inv().apply(state[:, 10:13])*180/np.pi)
    altitude = grid(-state[:, 2])
    alt = altitude-grid([s['targetAltitude'] for s in samples])
    xtrack = grid([s['crossTrack'] for s in samples])
    speed = grid(np.linalg.norm(state[:, 7:10]-np.array([s['wind'] for s in samples]), axis=1))
    scale = np.array([1300, 1500])*np.pi*55/360*.1
    commands = grid(np.array([s['command'] for s in samples]) / np.r_[1, scale])
    brakes = grid(np.array([s['brakes'] for s in samples])/scale)
    periods = dict(disturbed=(t >= 20) & (t < 100), later_disturbed=(t >= 60) & (t < 100), recovery=(t >= 100) & (t < 140),
                   late=(t >= 150), active=(t >= 20))
    phases = {}
    for name, m in periods.items():
        phases[name] = dict(altitude=errors(alt[m]), xtrack=errors(xtrack[m]),
            pitch_band_deg=band_rms(attitude[m, 1]), pitch_rate_deg_s=rms(rates[m, 1]),
            relative_pitch_rate_deg_s=rms(rel_rate[m, 1]),
            relative_rate_vector_rms_deg_s=float(np.sqrt(np.mean(np.sum(rel_rate[m]**2, axis=1)))),
            relative_angle_ac_vector_deg=float(np.sqrt(np.mean(np.sum(detrend(relative[m], axis=0)**2, axis=1)))),
            relative_pitch_band_deg=band_rms(relative[m, 1]),
            mean_speed=float(np.mean(speed[m])), min_speed=float(np.min(speed[m])))
    m = periods['active']
    brake_high = np.any(brakes > .99, axis=1)
    throttle_high = commands[:, 0] >= .594  # recorded THR_MAX=60%, 99% of its authority
    travel = np.sum(abs(np.diff(commands[m], axis=0)), axis=0)/(t[m][-1]-t[m][0])
    # Rate limit avoids declaring quiet attitude while the canopy is swinging.
    n = 20
    rate_rms_2s = np.sqrt(np.convolve(rel_rate[:, 1]**2, np.ones(n)/n, 'full')[:len(t)])
    quiet = rate_rms_2s <= 2
    recovered = (abs(alt) <= 2) & quiet
    r = dict(variant=variant, id=d['spec']['id'], mode=d['spec']['mode'],
             heldout=d['spec']['id'] not in TRAIN, complete=d['complete'], failure=d['failure'],
             input=str(path.relative_to(HERE)), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
             phases=phases, altitude_recovery_s=settling(t, recovered),
             altitude_error_at_weather_onset_m=float(np.interp(20, t, alt)),
             canopy_quiet_s=settling(t, quiet),
             altitude_peak_after_removal_m=float(np.max(abs(alt[periods['recovery']]))),
             brake_high_fraction=float(np.mean(brake_high[m])),
             brake_high_longest_s=longest(brake_high & m),
             throttle_high_fraction=float(np.mean(throttle_high[m])),
             throttle_high_longest_s=longest(throttle_high & m),
             travel_per_s=travel.tolist(),
             max_roll_deg=float(np.max(abs(attitude[m, 0]))),
             max_pitch_deg=float(np.max(abs(attitude[m, 1]))), min_altitude_m=float(np.min(altitude[m])),
             shaft_W=old['shaft_W'], shaft_Wh_per_ground_km=old['shaft_Wh_per_ground_km'],
             final_mission=samples[-1]['mission'])
    failures = []
    if not d['complete'] or d['failure']:
        failures.append('harness stop')
    if r['max_roll_deg'] > 60 or r['max_pitch_deg'] > 45 or r['min_altitude_m'] < 3:
        failures.append('truth envelope')
    if r['brake_high_longest_s'] > 5:
        failures.append('brake upper saturation >5 s')
    late = phases['late']
    if late['relative_pitch_band_deg'] > 1 and late['relative_pitch_rate_deg_s'] > 2:
        failures.append('late longitudinal motion exceeds screen')
    r['screen_failures'] = failures
    p, rec = phases['disturbed'], phases['recovery']
    # Fixed dimensional reference scales. All component scores are squared losses;
    # 1 represents the stated scale, not a physical safety boundary.
    r['components'] = dict(
        tracking=float(np.mean([(p['altitude']['rms']/2)**2, (p['altitude']['p95']/5)**2,
                                (p['xtrack']['rms']/5)**2, (p['xtrack']['p95']/10)**2])),
        recovery=float(np.mean([(rec['altitude']['rms']/2)**2,
                                (rec['relative_pitch_rate_deg_s']/5)**2])),
        internal_motion=float(np.mean([(p['relative_angle_ac_vector_deg']/5)**2,
                                       (p['relative_rate_vector_rms_deg_s']/20)**2])),
        actuator=float(np.mean([(travel[0]/.2)**2, (np.mean(travel[1:])/.2)**2,
                                (r['brake_high_fraction']/.05)**2,
                                (r['throttle_high_fraction']/.1)**2])),
        energy=float((r['shaft_Wh_per_ground_km']/5)**2))
    r['score'] = float(np.dot(WEIGHTS, list(r['components'].values()))) if not failures else None
    later = phases['later_disturbed']
    lc = dict(r['components'])
    lc['tracking'] = float(np.mean([(later['altitude']['rms']/2)**2, (later['altitude']['p95']/5)**2,
                                    (later['xtrack']['rms']/5)**2, (later['xtrack']['p95']/10)**2]))
    lc['internal_motion'] = float(np.mean([(later['relative_angle_ac_vector_deg']/5)**2,
                                          (later['relative_rate_vector_rms_deg_s']/20)**2]))
    r['later_window_components'] = lc
    r['later_window_score'] = float(np.dot(WEIGHTS, list(lc.values()))) if not failures else None
    return r, (t, alt, relative[:, 1], rel_rate[:, 1])


def distribution(x):
    x = np.asarray(x)
    return dict(median=float(np.median(x)), p90=float(np.quantile(x, .9)),
                worst_quartile_mean=float(np.mean(np.sort(x)[-max(1, int(np.ceil(len(x)/4))):])),
                worst=float(np.max(x)), values=x.tolist())


def main():
    OUT.mkdir(exist_ok=True)
    records, traces = [], {}
    configs = [('fixed', v) for v in ['standard', 'sensor', 'airspeed', 'sensorless', 'angles', 'full']]
    configs += [('damping', 'sensor'), ('efficient', 'sensor')]
    for group, source in configs:
        old = {r['id']: r for r in json.loads((HERE/group/'analysis.json').read_text()) if r['variant'] == source}
        variant = source if group == 'fixed' else group
        for p in sorted((HERE/group/source/'results').glob('*.json')):
            i = int(p.name.split('-')[0])
            r, trace = analyze(p, variant, old[i])
            records.append(r)
            traces[variant, i] = trace
    assert len(records) == 92
    lookup = {(r['variant'], r['id']): r for r in records}
    comparisons = {}
    rng = np.random.default_rng(20261017)
    random_weights = rng.dirichlet(WEIGHTS*40, 10000)
    random_weights = random_weights[random_weights[:, 4] <= .15]
    scale_weights = WEIGHTS / np.exp(rng.uniform(np.log(.5), np.log(2), (10000, 5)))**2
    # Also test deliberate alternatives, including no efficiency contribution.
    profiles = dict(proposed=WEIGHTS.tolist(), no_energy=[.37,.32,.21,.10,0],
                    tracking_first=[.60,.20,.10,.05,.05], recovery_first=[.20,.50,.20,.05,.05],
                    motion_first=[.20,.20,.50,.05,.05])
    for a, b in [('sensor','standard'), ('damping','standard'), ('sensor','airspeed'), ('damping','sensor')]:
        for mode in ['AUTO', 'CRUISE']:
            pairs = [(r, lookup[b,r['id']]) for r in records if r['variant'] == a and r['mode'] == mode
                     and r['heldout'] and (b,r['id']) in lookup]
            delta = np.array([[x['components'][k]-y['components'][k] for k in COMPONENTS] for x,y in pairs])
            changes = {}
            getters = dict(alt_rms=lambda r:r['phases']['disturbed']['altitude']['rms'],
                alt_p95=lambda r:r['phases']['disturbed']['altitude']['p95'],
                xtrack_rms=lambda r:r['phases']['disturbed']['xtrack']['rms'],
                relative_angle=lambda r:r['phases']['disturbed']['relative_angle_ac_vector_deg'],
                relative_rate=lambda r:r['phases']['disturbed']['relative_rate_vector_rms_deg_s'],
                recovery_alt_rms=lambda r:r['phases']['recovery']['altitude']['rms'],
                pitch_band=lambda r:r['phases']['disturbed']['pitch_band_deg'],
                energy=lambda r:r['shaft_Wh_per_ground_km'])
            for k, getter in getters.items():
                changes[k] = distribution([100*(getter(x)/max(getter(y),1e-9)-1) for x,y in pairs])
            qualified = all(not x['screen_failures'] and not y['screen_failures'] for x,y in pairs)
            comparisons[f'{a}_vs_{b}_{mode}'] = dict(n=len(pairs), qualified=qualified,
                paired_changes_pct=changes,
                profiles={k:dict(median_delta=float(np.median(delta@w)), wins=int(np.sum(delta@w < 0)))
                          for k,w in profiles.items()},
                median_paired_score_change_pct=float(np.median([100*(x['score']/y['score']-1) for x,y in pairs]))
                if qualified else None,
                later_window_paired_score_change_pct=float(np.median([100*(x['later_window_score']/y['later_window_score']-1) for x,y in pairs]))
                if qualified else None,
                scale_sensitivity_fraction_favoring_a=float(np.mean(np.median(delta@scale_weights.T, axis=0)<0))
                if qualified else None,
                sampled_weight_fraction_favoring_a=float(np.mean(np.median(delta@random_weights.T, axis=0)<0))
                if qualified else None)
    groups = {}
    for mode in ['AUTO','CRUISE']:
        for variant in NAMES:
            rr = [r for r in records if r['variant']==variant and r['mode']==mode and r['heldout']]
            groups[f'{variant}_{mode}'] = dict(score=distribution([r['score'] for r in rr if r['score'] is not None]),
                components={k:distribution([r['components'][k] for r in rr]) for k in COMPONENTS})
    result = dict(weights=dict(zip(COMPONENTS, WEIGHTS.tolist())), profiles=profiles,
                  cases=records, heldout_comparisons=comparisons, heldout_groups=groups)
    (OUT/'metrics.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    lines = ['# Holistic sensor-equipped LQI assessment', '',
        'AI-assisted retrospective analysis of the 92 retained runs. No new flight runs or gain selection.', '',
        '**Answer:** the realistic sensor-equipped LQI has a useful CRUISE tradeoff: better altitude '
        'tracking/recovery and less relative angular-rate motion and brake activity. In AUTO, its quicker '
        'recovery does not compensate for substantially worse altitude capture and path tracking under the proposed index. '
        'Both controllers pass the finite-horizon stability screen; neither demonstrates a decisive stability '
        'advantage in these cases. Additional angle sensing contributes little beyond the pitot with these gains. '
        'The previous pitch-penalty retune improves some recovery/rate and tail metrics, but is not a consistent '
        'winner under this broader index. No controller is promoted.', '',
        'Primary comparisons below use only the twelve cases excluded from the original tuning screen: '
        'six AUTO and six CRUISE. The new metrics themselves are retrospective, so this is not a preregistered validation.', '',
        '## Stability screening', '',
        '| Configuration | Runs | Harness failures | New screen failures | Max roll / pitch ° | Worst late relative-pitch band RMS ° | Longest full-brake episode s |',
        '|---|---:|---:|---:|---:|---:|---:|']
    for variant in ['standard','sensorless','airspeed','sensor','damping','efficient','angles','full']:
        rr = [r for r in records if r['variant']==variant]
        lines.append(f'| {variant} | {len(rr)} | {sum(not r["complete"] for r in rr)} | '
            f'{sum(bool(r["screen_failures"]) for r in rr)} | '
            f'{max(r["max_roll_deg"] for r in rr):.2f} / {max(r["max_pitch_deg"] for r in rr):.2f} | '
            f'{max(r["phases"]["late"]["relative_pitch_band_deg"] for r in rr):.3f} | '
            f'{max(r["brake_high_longest_s"] for r in rr):.1f} |')
    lines += ['', '## Held-out flight quality', '',
        'Entries are scenario medians; parentheses give the worst of six cases. '
        'Altitude is truth height minus the recorded target; cross-track is onboard navigation telemetry.', '',
        '| Mode | Configuration | Alt RMS m | Alt P95 m | Xtrack RMS m | Relative angle AC RMS ° | Relative rate RMS °/s | Recovery alt RMS m |',
        '|---|---|---:|---:|---:|---:|---:|---:|']
    for mode in ['AUTO','CRUISE']:
        for variant in NAMES:
            rr = [r for r in records if r['variant']==variant and r['mode']==mode and r['heldout']]
            getters = [lambda r:r['phases']['disturbed']['altitude']['rms'],
                lambda r:r['phases']['disturbed']['altitude']['p95'],
                lambda r:r['phases']['disturbed']['xtrack']['rms'],
                lambda r:r['phases']['disturbed']['relative_angle_ac_vector_deg'],
                lambda r:r['phases']['disturbed']['relative_rate_vector_rms_deg_s'],
                lambda r:r['phases']['recovery']['altitude']['rms']]
            vals = [[get(r) for r in rr] for get in getters]
            lines.append(f'| {mode} | {NAMES[variant]} | '+' | '.join(f'{np.median(v):.2f} ({max(v):.2f})' for v in vals)+' |')
    lines += ['', '## Capture confound and later disturbed flight', '',
        'AUTO was not settled when weather began at 20 s: LQI was still 10–14 m below target in the held-out cases, '
        'versus roughly 1–5 m for standard. The primary score intentionally includes this slower capture, '
        'but it must not be interpreted as pure gust rejection. The following 60–100 s window reduces that '
        'confound; mission phase and disturbance exposure are still not synchronized.', '',
        '| Mode | Configuration | Altitude error at 20 s, median m | Later altitude RMS m | Later xtrack RMS m | Later relative-rate RMS °/s |',
        '|---|---|---:|---:|---:|---:|']
    for mode in ['AUTO','CRUISE']:
        for variant in NAMES:
            rr = [r for r in records if r['variant']==variant and r['mode']==mode and r['heldout']]
            lines.append(f'| {mode} | {NAMES[variant]} | '
                f'{np.median([r["altitude_error_at_weather_onset_m"] for r in rr]):.2f} | '
                f'{np.median([r["phases"]["later_disturbed"]["altitude"]["rms"] for r in rr]):.2f} | '
                f'{np.median([r["phases"]["later_disturbed"]["xtrack"]["rms"] for r in rr]):.2f} | '
                f'{np.median([r["phases"]["later_disturbed"]["relative_rate_vector_rms_deg_s"] for r in rr]):.2f} |')
    lines += ['', '## Recovery, actuation, and aggregate index', '',
        'Recovery time: first ten continuous seconds within 2 m altitude error and 2 °/s '
        'relative pitch-rate (2 s RMS) after weather removal. Unrecovered cases are censored, not assigned zero. '
        'The last evaluable onset is about 70 s. This is a combined weather-removal response, not a modal damping estimate.', '',
        '| Mode | Configuration | Recovered / 6 | Median recovery s (recovered only) | Throttle ceiling % | Brake travel/s | Score median | Worst-quarter score | Worst score |',
        '|---|---|---:|---:|---:|---:|---:|---:|---:|']
    for mode in ['AUTO','CRUISE']:
        for variant in NAMES:
            rr = [r for r in records if r['variant']==variant and r['mode']==mode and r['heldout']]
            recovery = [r['altitude_recovery_s'] for r in rr if r['altitude_recovery_s'] is not None]
            score = distribution([r['score'] for r in rr if r['score'] is not None])
            lines.append(f'| {mode} | {NAMES[variant]} | {len(recovery)} | '
                f'{np.median(recovery) if recovery else float("nan"):.1f} | '
                f'{100*np.median([r["throttle_high_fraction"] for r in rr]):.2f} | '
                f'{np.median([np.mean(r["travel_per_s"][1:]) for r in rr]):.3f} | '
                f'{score["median"]:.3f} | {score["worst_quartile_mean"]:.3f} | {score["worst"]:.3f} |')
    lines += ['', '## Paired changes and preference sensitivity', '',
        'All percentages are median paired changes, not ratios of group medians. Negative is better. '
        'Weight sensitivity is a preference exploration, not a probability of superiority.', '',
        '| Comparison | Mode | Alt RMS % | Xtrack RMS % | Relative angle % | Relative rate % | Recovery alt RMS % | Weights favor first % | Wins / 6 (proposed) |',
        '|---|---|---:|---:|---:|---:|---:|---:|---:|']
    for name, c in comparisons.items():
        comparison, mode = name.rsplit('_',1)
        vals = [c['paired_changes_pct'][k]['median'] for k in ['alt_rms','xtrack_rms','relative_angle','relative_rate','recovery_alt_rms']]
        lines.append(f'| {comparison} | {mode} | '+' | '.join(f'{v:+.2f}' for v in vals)+
            f' | {100*c["sampled_weight_fraction_favoring_a"]:.1f} | {c["profiles"]["proposed"]["wins"]} |')
    lines += ['', '| Comparison | Mode | Paired score change % | Later-window score change % | Scale choices favor first % | No-energy wins / 6 |',
              '|---|---|---:|---:|---:|---:|']
    for name, c in comparisons.items():
        comparison, mode = name.rsplit('_',1)
        lines.append(f'| {comparison} | {mode} | {c["median_paired_score_change_pct"]:+.2f} | '
            f'{c["later_window_paired_score_change_pct"]:+.2f} | '
            f'{100*c["scale_sensitivity_fraction_favoring_a"]:.1f} | {c["profiles"]["no_energy"]["wins"]} |')
    lines += ['', '## Index definition and limits', '',
        'The five components use weights 35/30/20/10/5% for tracking/recovery/internal motion/actuation/energy. '
        'Each is an arithmetic mean of squared dimensional losses. These are provisional engineering scales, '
        'not user-approved requirements or safety limits:', '',
        '- Tracking: disturbed altitude RMS / 2 m, altitude P95 / 5 m, cross-track RMS / 5 m, cross-track P95 / 10 m.',
        '- Recovery: first 40 s after weather removal, altitude RMS / 2 m and relative pitch-rate RMS / 5 °/s.',
        '- Internal motion: disturbed detrended relative rotation-vector RMS / 5° and relative angular-rate RMS / 20 °/s.',
        '- Actuation: throttle travel / 0.2 fraction/s, mean brake travel / 0.2 fraction/s, '
        'upper-brake occupancy / 5%, throttle-ceiling occupancy / 10%. Released brakes are not penalized as saturation. '
        'The recorded throttle ceiling is 60%, not 100%.',
        '- Energy: shaft Wh per actual ground km / 5 Wh/km; still not equal achieved missions or electrical energy.', '',
        'Hard screens: recorded failures, truth roll >60°, pitch >45°, altitude <3 m, '
        'more than 5 s continuous applied upper-brake saturation, or late relative pitch band RMS >1° '
        'together with relative pitch-rate RMS >2 °/s. Failure disqualifies a score. These are finite-horizon '
        'operational screens, not proofs of asymptotic stability. Late windows can contain commanded AUTO turns.', '',
        'Sensitivity uses named tracking/recovery/motion-first profiles and no-energy profile, plus seeded '
        'Dirichlet samples centered on the proposed weights (concentration 40, energy <=15%). '
        'A separate scale sensitivity varies each component reference scale by 0.5–2x (log-uniform). '
        'These coefficients are not guaranteed percentages of the resulting total loss. '
        'All profile results, every run, input hashes, P90/worst-quarter/worst values and paired changes are in metrics.json.', '',
        '**Airspeed objective:** the standard controller uses the paraglider TECS branch, which controls '
        'height with throttle/pitch-rate damping and explicitly disables airspeed-based throttle control. '
        'It assumes nearly constant natural airspeed; ordinary TECS speed-error telemetry is not meaningful. '
        'The archived LQI clamps its true-speed target to 3.8–4.4 m/s. Thus achieved airspeed is retained, '
        'but an equal-reference airspeed tracking score '
        'cannot be reconstructed fairly and is deliberately excluded. Cross-track uses onboard estimates; '
        'AUTO path/mission phase is not synchronized across controllers. Altitude target telemetry is sampled/held. '
        'There are no isolated command-step tests, no identified closed-loop poles, no matched sensor-bias sweep, '
        'and no independently measured canopy physics uncertainty. Current results answer how these tunes flew '
        'this task suite, not the best possible sensor-equipped LQI.', '',
        '![Held-out component scores](components.png)', '', '![All held-out recovery traces](recovery.png)']
    (OUT/'REPORT.md').write_text('\n'.join(lines)+'\n')
    fig, axes = plt.subplots(2, 5, figsize=(15,6), constrained_layout=True)
    names = ['standard','sensor','damping']
    for i, mode in enumerate(['AUTO','CRUISE']):
        for j, component in enumerate(COMPONENTS):
            ax = axes[i,j]
            for k, v in enumerate(names):
                values = [r['components'][component] for r in records if r['variant']==v and r['mode']==mode and r['heldout']]
                ax.bar(k,np.median(values),color=['#c89337','#2b9a82','#4b80b5'][k])
                ax.scatter(k+np.linspace(-.12,.12,len(values)), values, s=12, color='black',alpha=.55)
            ax.set_title(mode+' '+component.replace('_',' '), fontsize=10)
            ax.set_xticks(range(3), ['Standard','LQI','Retuned'],fontsize=8)
            ax.grid(axis='y',alpha=.2)
    fig.suptitle('Held-out component losses: lower is better; dots are all six scenarios')
    fig.savefig(OUT/'components.png',dpi=150)
    plt.close(fig)
    fig, axes = plt.subplots(4,3,figsize=(13,10),constrained_layout=True)
    for ax, case in zip(axes.flat, sorted(set(range(16))-TRAIN)):
        for v,color in zip(names,['#c89337','#2b9a82','#4b80b5']):
            t,alt,rel,rate = traces[v,case]
            m=t>=95
            ax.plot(t[m]-100,alt[m],label=v,color=color,lw=.85)
        ax.axvline(0,color='gray',lw=.6)
        ax.axhspan(-2,2,color='gray',alpha=.1)
        ax.set_title(f'{case:02} '+('AUTO' if case%2==0 else 'CRUISE'))
        ax.set_ylabel('Truth altitude error (m)')
        ax.set_xlabel('Seconds after weather removal')
        ax.grid(alpha=.2)
    axes[0,0].legend(fontsize=8)
    fig.suptitle('Every held-out case: altitude recovery (shaded band ±2 m)')
    fig.savefig(OUT/'recovery.png',dpi=150)
    plt.close(fig)
    print('\n'.join(lines[:lines.index('## Index definition and limits')]))


if __name__ == '__main__':
    main()
