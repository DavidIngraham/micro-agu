"""Publish final L1 landing figures and bounded truth replays; raw logs stay in Mission Planner.

Run with the ArduPilot Python environment. The optional --run-preview executes the
native gentle-wind preview before export. Existing calm10 and landing10-* runs
are the completed regression study, not regenerated or relabeled holdout data.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from pymavlink import mavutil
from scipy.spatial.transform import Rotation, Slerp

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--root', type=Path, required=True)
parser.add_argument('--ardupilot', type=Path, default=Path('/home/david/repos/ardupilot-high-dof'))
parser.add_argument('--run-preview', action='store_true')
args = parser.parse_args()
repo = Path(__file__).resolve().parents[1]
root = args.root
old = root.parent / 'landing-refinement-2026-10-13'
out = repo / 'docs/assets/landing-guidance'
out.mkdir(parents=True, exist_ok=True)
expected = json.loads((args.ardupilot / 'Tools/simulation/paraglider/mission-controller-candidate.json').read_text())['parameters']
assert expected['NAVL1_PERIOD'] == 10 and expected['NAVL1_DAMPING'] == 1
assert expected['PG_TURN_FILT'] == 1
if args.run_preview:
    destination = root / 'gentle10'
    assert not (destination / 'completed.json').exists(), 'Preserve the existing completed preview'
    destination.mkdir(exist_ok=True)
    env = dict(os.environ, BUILDLOGS=str(destination), PG_CONTROLLER_CONFIG=str(root / 'candidate-10.json'),
               PG_GAINS='mission', PG_LANDING_CONFIG=str(old / 'preview-config.json'))
    env['PATH'] = str(Path(sys.executable).parent) + ':' + env['PATH']
    with (destination / 'run.log').open('w') as stream:
        run = subprocess.run([sys.executable, 'Tools/simulation/paraglider/landing_autotest.py',
                              '--no-check-parameter-leaks', 'test.Plane.ParagliderLandingStudy'],
                             cwd=args.ardupilot, env=env, stdout=stream, stderr=subprocess.STDOUT, timeout=1200)
    (destination / 'completed.json').write_text(json.dumps({'returncode': run.returncode}))
    assert run.returncode == 0


def read(directory, period=10):
    assert 'PASSED STEP: test.Plane.ParagliderLandingStudy' in (directory / 'run.log').read_text()
    land = json.loads((directory / 'landing.json').read_text())
    frames = json.loads((directory / 'viewer.json').read_text())
    log = mavutil.mavlink_connection(str(directory / 'flight.BIN'))
    params, pg, flare = {}, [], None
    while (msg := log.recv_match(type=['PARM', 'PGYR', 'PGFL'])) is not None:
        if msg.get_type() == 'PARM':
            params[msg.Name] = msg.Value
        elif msg.get_type() == 'PGYR':
            pg.append([msg.TimeUS / 1e6, msg.Tgt, msg.Out])
        elif msg.Active:
            if flare is None:
                flare = msg.TimeUS / 1e6
            assert 0 <= msg.Common <= 60.001 and 0 <= msg.Left <= 100 and 0 <= msg.Right <= 100
    assert flare is not None
    for key, value in expected.items():
        assert abs(params[key] - (period if key == 'NAVL1_PERIOD' else value)) < 1e-5, (directory, key)
    assert params['SCHED_LOOP_RATE'] == 50
    pg = np.array(pg)
    assert abs(np.median(np.diff(pg[:, 0])) - .02) < 1e-5
    selection = [f for f in frames if flare - 20 < f['time_s'] < flare]
    t = np.array([f['time_s'] for f in selection])
    state = np.array([f['state'] for f in selection])
    rot = Rotation.from_quat(state[:, 3:7])
    angles = rot.as_euler('xyz')
    body_rate = rot.inv().apply(state[:, 10:13])
    rate = np.rad2deg((body_rate[:, 1] * np.sin(angles[:, 0]) + body_rate[:, 2] * np.cos(angles[:, 0])) / np.cos(angles[:, 1]))
    target = np.interp(t, pg[:, 0], pg[:, 1])
    brake = np.interp(t, pg[:, 0], pg[:, 2])
    roll = np.rad2deg(angles[:, 0])
    stats = dict(miss_m=land['radial_error_m'], sink_m_s=land['contact']['velocity'][2],
                 groundspeed_m_s=float(np.linalg.norm(land['contact']['velocity'][:2])),
                 roll_90pct_span_deg=float(np.ptp(np.percentile(roll, [5, 95]))),
                 rate_error_rms_deg_s=float(np.sqrt(np.mean((rate-target)**2))),
                 parameters={k: params[k] for k in expected}, flare_time_s=flare,
                 log_sha256=hashlib.sha256((directory/'flight.BIN').read_bytes()).hexdigest())
    return land, frames, t-flare, roll, rate, target, brake, stats


def replay(data, label, filename):
    land, frames, *_, stats = data
    end = land['contact']['time']
    left = max((f for f in frames if f['time_s'] <= end), key=lambda f: f['time_s'])
    right = min((f for f in frames if f['time_s'] >= end), key=lambda f: f['time_s'])
    fraction = (end-left['time_s'])/(right['time_s']-left['time_s']) if right['time_s'] > left['time_s'] else 0
    endpoint = dict(left, time_s=end)
    state = np.array(left['state']) + fraction*(np.array(right['state'])-left['state'])
    for offset in [3, 16]:
        state[offset:offset+4] = Slerp([0, 1], Rotation.from_quat([left['state'][offset:offset+4], right['state'][offset:offset+4]]))([fraction]).as_quat()[0]
    endpoint['state'] = state.tolist()
    for key in ['rpm', 'throttle', 'voltage_v', 'brake_mm', 'wind_ned_m_s', 'joint_rotvec_rad', 'joint_rate_rad_s']:
        endpoint[key] = (np.array(left[key])+fraction*(np.array(right[key])-left[key])).tolist()
    clipped = [f for f in frames if end-22 <= f['time_s'] < end] + [endpoint]
    # Truth frames are sampled more slowly than native ground-crossing detection.
    assert abs(endpoint['state'][2]) < .001
    result = dict(frames=clipped, target_ned_m=land['target_ned_m'], result=land, selected_trial=1,
                  selection='Illustration only; selected 10 s L1 / damping 1 / 1 Hz measurement filter',
                  label=label, parameters=stats['parameters'], flare_time_s=stats['flare_time_s'],
                  flare_event_source='First active native PGFL sample; SITL simulation timestamp',
                  flare_log_sha256=stats['log_sha256'])
    (repo/'viewer/assets/recordings/landing'/filename).write_text(json.dumps(result, separators=(',', ':')))


calm_old = read(old/'preview-calm', 8)
calm = read(root/'calm10')
gentle = read(root/'gentle10')
assert all(calm[0]['config'][k] == v for k, v in gentle[0]['config'].items() if k not in ['wind_ned_m_s', 'gust_amplitude_m_s'])
assert calm[0]['config']['wind_ned_m_s'] == [0, 0, 0] and calm[0]['config']['gust_amplitude_m_s'] == 0
assert gentle[0]['config']['gust_amplitude_m_s'] == .25
for data, label, filename in [(calm, 'Calm air · L1 10 s', 'replay-calm.json'), (gentle, 'Gentle turbulence · L1 10 s', 'replay.json')]:
    replay(data, label, filename)

for name, series in [('calm-guidance', [(calm_old, 'L1 8 s'), (calm, 'L1 10 s')]),
                     ('preview-response', [(calm, 'Calm air'), (gentle, 'Gentle turbulence')])]:
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    for data, label in series:
        _, _, t, roll, rate, target, brake, _ = data
        line, = axes[0].plot(t, roll, label=label)
        axes[1].plot(t, rate, color=line.get_color(), label=label)
        axes[1].plot(t, target, '--', color=line.get_color(), alpha=.65)
        axes[2].plot(t, brake, color=line.get_color(), label=label)
    for ax, label in zip(axes, ['Payload roll (deg)', 'Heading rate (deg/s)', 'Differential brake demand']):
        ax.set_ylabel(label); ax.grid(alpha=.2); ax.legend(loc='upper right')
    axes[0].set_title('Final 20 seconds before flare · 1 Hz measurement filter · L1 damping 1.0')
    axes[1].set_title('Solid: simulator truth; dashed: controller target', fontsize=10)
    axes[-1].set_xlabel('Time relative to native flare entry (s)')
    fig.tight_layout(); fig.savefig(out/f'{name}.png', dpi=150); plt.close(fig)

runs = [read(root/f'landing10-{i:02d}') for i in range(12)]
errors = np.array([x[-1]['miss_m'] for x in runs])
sinks = np.array([x[-1]['sink_m_s'] for x in runs])
summary = dict(classification='Regression set reused during L1 tuning; not independent holdout validation',
               parameters=expected, within_5m=int(sum(errors <= 5)), count=len(runs),
               median_m=float(np.median(errors)), p90_m=float(np.percentile(errors, 90)), worst_m=float(max(errors)),
               median_sink_m_s=float(np.median(sinks)), p90_sink_m_s=float(np.percentile(sinks, 90)),
               median_groundspeed_m_s=float(np.median([x[-1]['groundspeed_m_s'] for x in runs])),
               trials=[dict(trial=i, **x[-1], config=x[0]['config']) for i, x in enumerate(runs)],
               previews=dict(calm=calm[-1], gentle=gentle[-1], calm_8s=calm_old[-1]))
(out/'final-results.json').write_text(json.dumps(summary, indent=2))
fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
for radius in [1, 2, 5]:
    axes[0].add_patch(plt.Circle((0, 0), radius, fill=False, color='.75'))
for x in runs:
    axes[0].scatter(x[0]['cross_track_error_m'], x[0]['along_track_error_m'], color='#218c9b')
axes[0].set_aspect('equal', adjustable='datalim')
axes[0].set(xlabel='Cross-track error (m)', ylabel='Along-track error (m)', title='Selected 10 s L1: 12 regression landings')
axes[1].scatter(errors, sinks, color='#218c9b')
axes[1].set(xlabel='Distance from target (m)', ylabel='Sink speed at crossing (m/s)', title='Placement and arrival speed')
axes[1].grid(alpha=.2)
fig.tight_layout(); fig.savefig(out/'landing-final.png', dpi=150); plt.close(fig)
for filename in ['mission-comparison.png', 'mission-results.json', 'case07-response.png']:
    shutil.copyfile(root/filename, out/filename)
print(json.dumps({k: v for k, v in summary.items() if k not in ['trials', 'parameters', 'previews']}, indent=2))
print('Preview results:', {k: {a: v[a] for a in ['miss_m','roll_90pct_span_deg']} for k,v in summary['previews'].items()})