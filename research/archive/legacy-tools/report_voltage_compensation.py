"""Plot matched native voltage-compensation runs; raw telemetry stays outside Git."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser()
parser.add_argument('--runs', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--include-auto', action='store_true')
a = parser.parse_args()
a.output.mkdir(parents=True, exist_ok=True)
metrics = json.loads((a.output / 'metrics.json').read_text())
cases = ['voltage_22_2_off', 'voltage_22_2_on', 'voltage_20_4_off', 'voltage_20_4_on']
labels = ['22.2 V, off', '22.2 V, on', '20.4 V, off', '20.4 V, on']
colors = ['gray', '#147c86', '#c15730', '#6d4cba']
checks = {}
fig, axes = plt.subplots(4, 2, figsize=(14, 12), sharex='col')
for case, label, color in zip(cases, labels, colors):
    folder = a.runs / case
    data = np.load(folder / 'signals.npz')
    events = json.loads((folder / 'study-events.json').read_text())['events']
    cfg = json.loads((folder / 'config.json').read_text())
    params = json.loads((folder / 'logged-parameters.json').read_text())
    check = json.loads((folder / 'voltage-check.json').read_text())
    battery = data['battery']
    active = battery[:, 0] >= events[0]['start']
    voltage_error = float(np.max(abs(battery[active, 1] - cfg['voltage'])))
    assert voltage_error < .01, (case, voltage_error)
    for key in ['FWD_BAT_VOLT_MAX', 'FWD_BAT_VOLT_MIN', 'FWD_BAT_IDX']:
        assert abs(params[key] - cfg['parameters'][key]) < 1e-4, (case, key)
    assert params['BATT_MONITOR'] == 3
    servo, tecs = data['servo'], data['tecs']
    settled = (tecs[:, 0] >= events[1]['end'] - 15) & (tecs[:, 0] < events[1]['end'])
    physical = np.interp(tecs[settled, 0], servo[:, 0], servo[:, 1])
    expected = np.minimum(tecs[settled, 5] * check['compensation_ratio'], check['physical_throttle_max'])
    scaling_error = float(np.median(abs(physical - expected)))
    # PWM quantization and the TECS/output scheduler have different sample times.
    assert scaling_error < .002, (case, scaling_error)
    checks[case] = check | dict(voltage_max_error=voltage_error, settled_scaling_median_error=scaling_error)
    for col, phase in enumerate(['climb', 'right_turn']):
        event = next(e for e in events if e['name'] == phase)
        start, end = event['start'], event['end'] + 35
        m = (tecs[:, 0] >= start - 5) & (tecs[:, 0] < end)
        n = (data['time'] >= start - 5) & (data['time'] < end)
        v = (servo[:, 0] >= start - 5) & (servo[:, 0] < end)
        axes[0, col].plot(tecs[m, 0] - start, tecs[m, 1] - tecs[m, 2], label=label, color=color)
        axes[1, col].plot(data['time'][n] - start, data['q'][n], color=color, lw=.8)
        axes[2, col].plot(tecs[m, 0] - start, 100 * tecs[m, 5], color=color)
        axes[3, col].plot(servo[v, 0] - start, 100 * servo[v, 1], color=color)
        axes[0, col].set_title(phase.replace('_', ' ').title() + ' and following capture / transition')
for col in range(2):
    for row, label in enumerate(['Height error (m)', 'Payload pitch rate (deg/s)', 'TECS throttle (%)', 'Motor command (%)']):
        axes[row, col].set_ylabel(label)
        axes[row, col].grid(alpha=.25)
        axes[row, col].axvline(0, color='black', lw=.5)
    axes[0, col].legend(fontsize=8)
    axes[-1, col].set_xlabel('Seconds from phase start')
fig.suptitle('Native ArduPlane voltage compensation: same controller, two battery voltages')
fig.tight_layout()
fig.savefig(a.output / 'voltage-response.png', dpi=150)
plt.close(fig)

fig, axes = plt.subplots(1, 2, figsize=(13, 5))
phases = ['level', 'climb', 'climb_capture', 'descent', 'descent_capture', 'right_turn', 'left_turn']
x = np.arange(len(phases))
for i, (case, label, color) in enumerate(zip(cases, labels, colors)):
    for ax, key in zip(axes, ['height_rmse', 'q_std']):
        ax.bar(x + (i - 1.5) * .19, [metrics[case][p][key] for p in phases], .19, label=label, color=color)
for ax in axes:
    ax.set_xticks(x, [p.replace('_', '\n') for p in phases], fontsize=8)
    ax.grid(axis='y', alpha=.2)
axes[0].set_ylabel('Phase height RMSE (m)')
axes[1].set_ylabel('Final 20 s pitch-rate standard deviation (deg/s)')
axes[0].legend(fontsize=8)
fig.suptitle('Tracking and settled motion; phase-wide height errors, final-window pitch variation')
fig.tight_layout()
fig.savefig(a.output / 'voltage-metrics.png', dpi=150)
plt.close(fig)
(a.output / 'verification.json').write_text(json.dumps(checks, indent=2) + '\n')

head = '''<!doctype html><meta charset="utf-8"><title>Native voltage compensation</title>
<style>body{font:17px/1.6 system-ui;max-width:1150px;margin:30px auto;padding:0 20px;color:#193742}img{width:100%}table{border-collapse:collapse}td,th{padding:8px;border-bottom:1px solid #ccd}pre{white-space:pre-wrap}</style>
<h1>Connecting the existing voltage compensation</h1><p>11 October 2026 · native ArduPlane + 9-DOF Trout Lake model</p>
<p>ArduPlane already compensates throttle for battery voltage. The previous study disabled the battery monitor, and the JSON backend did not send its physics voltage. This experiment connects the monitor and compares compensation off/on at 22.2 and 20.4 V. The mechanical model and controller gains are unchanged.</p>
<p>The reference is the existing 22.2 V calibration, not the pack's full-charge voltage. Defaults are FWD_BAT_VOLT_MAX=22.2, FWD_BAT_VOLT_MIN=18, FWD_BAT_IDX=0. A voltage-only analog monitor receives the JSON backend's fixed voltage; no current, discharge or capacity model is claimed.</p>
<pre>c(V) = 22.2 / clamp(V, 18, 22.2)
Motor throttle = clip(c(V) × TECS throttle, 0, floor(60 × c(V))/100)
At 20.4 V: c = 1.08824; physical throttle limit = 65%.</pre>
<p>Native servos.cpp also returns that 65% maximum to TECS. Consequently the climbing feedforward slope rises from 0.15 to 0.20 throttle fraction per m/s before output scaling, and vertical-speed PI gains scale with the new limit. The corrected linear/nonlinear analysis includes both effects. This is the existing firmware behavior, not a new voltage-aware feedforward controller.</p>
<p>At 20.4 V, climb height RMSE improves from 0.486 to 0.250 m and level-phase error from 0.633 to 0.016 m. Climb-capture RMSE increases from 0.096 to 0.169 m. At 22.2 V, results are effectively unchanged. Retain compensation, with the native feedforward-limit interaction recorded as a remaining capture refinement.</p><img src="voltage-response.png"><img src="voltage-metrics.png">
<table><tr><th>Case</th><th>Logged voltage</th><th>Climb height RMSE</th><th>Climb-capture height RMSE</th><th>Right/left turn-rate RMSE</th></tr>'''
for case, label in zip(cases, labels):
    m = metrics[case]
    head += f"<tr><td>{label}</td><td>{checks[case]['logged_voltage']:.3f} V</td><td>{m['climb']['height_rmse']:.3f} m</td><td>{m['climb_capture']['height_rmse']:.3f} m</td><td>{m['right_turn']['turn_error_rms']:.3f} / {m['left_turn']['turn_error_rms']:.3f} deg/s</td></tr>"
head += '''</table><p>All four runs completed the existing armed FBWB sequence. Completion is separate from these tracking metrics. The voltage checks verify the logged battery and configured compensation parameters against each run's physics configuration, and compare settled native TECS output with final PWM scaling. Saturation is counted against the voltage-adjusted output ceiling.</p>
<p><a href="metrics.json">Phase metrics</a> · <a href="verification.json">Voltage and output checks</a> · <a href="../classical-longitudinal-compensated/report.html">Corrected longitudinal linear analysis</a></p>
<p>Run Tools/simulation/paraglider/run_longitudinal_study.py with study-voltage-compensation.json, then analyze_longitudinal_study.py with --cases and --output. Run research/archive/legacy-tools/report_voltage_compensation.py --runs NATIVE_RUN_DIRECTORY --output THIS_DIRECTORY. Raw BIN and plant traces remain in their existing native run folders; final figures and plotting scripts are archived in tinkering.</p>'''
(a.output / 'report.html').write_text(head)
print(json.dumps(checks, indent=2))
for case in cases:
    print(case, {p: round(metrics[case][p]['height_rmse'], 4) for p in phases})
if a.include_auto:
    summary = {}
    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    for case, label, color in [('voltage_auto_22_2', '22.2 V compensated', '#147c86'), ('voltage_auto_20_4', '20.4 V compensated', '#6d4cba')]:
        folder = a.runs / case
        data = np.genfromtxt(folder / 'ParagliderAutoMission.csv', delimiter=',', names=True)
        meta = json.loads((folder / 'ParagliderAutoMission.csv.json').read_text())
        backend = json.loads((folder / 'high-dof-backend.json').read_text())
        assert not backend['errors'], backend
        assert np.isfinite(np.load(folder / 'high-dof-trace.npz')['samples']).all()
        from pymavlink import mavutil
        log = mavutil.mavlink_connection(str(folder / 'flight.BIN'))
        logged_params, volts = {}, []
        while True:
            message = log.recv_match(type=['PARM', 'BAT'])
            if message is None:
                break
            if message.get_type() == 'PARM':
                logged_params[message.Name] = message.Value
            elif message.Inst == 0 and message.TimeUS * 1e-6 >= data['time_s'][0]:
                volts.append(message.Volt)
        assert logged_params['BATT_MONITOR'] == 3
        assert abs(logged_params['FWD_BAT_VOLT_MAX'] - 22.2) < 1e-4
        assert abs(logged_params['FWD_BAT_VOLT_MIN'] - 18) < 1e-4
        assert logged_params['FWD_BAT_IDX'] == 0
        assert volts and max(abs(v - backend['voltage']) for v in volts) < .01
        t = data['time_s'] - data['time_s'][0]
        north = np.deg2rad(data['latitude_deg'] - meta['home_lat']) * 6371000
        east = np.deg2rad(data['longitude_deg'] - meta['home_lng']) * 6371000 * np.cos(np.deg2rad(meta['home_lat']))
        axes[0, 0].plot(east, north, color=color, label=label)
        axes[0, 1].plot(t, data['relative_alt_m'], color=color, label=label)
        phase = data[data['seq'] == 3]
        phase = phase[phase['time_s'] > phase['time_s'][-1] - 30]
        axes[1, 0].plot(phase['time_s'] - phase['time_s'][-1], np.rad2deg(phase['roll_rad']), color=color, label=label)
        axes[1, 1].plot(phase['time_s'] - phase['time_s'][-1], phase['target_distance_m'], color=color, label=label)
        m = meta['metrics']
        waypoint_error = [abs(m[str(i)]['altitude_at_closest_m'] - meta['route'][i-1][3]) for i in [1,2,4,5,6,7,9,10,11] if m[str(i)]['altitude_at_closest_m'] is not None]
        overshoot = [v['next_track_overshoot_m'] for v in m.values() if v['next_track_overshoot_m'] is not None]
        summary[case] = dict(voltage=backend['voltage'], logged_voltage=float(np.median(volts)), duration_s=float(t[-1]), max_waypoint_altitude_error_m=max(waypoint_error), max_corner_overshoot_m=max(overshoot), first_loiter_bank_std_deg=float(np.std(np.rad2deg(phase['roll_rad']))), first_loiter_radius_m=float(np.median(phase['target_distance_m'])))
    axes[0, 0].set(xlabel='East (m)', ylabel='North (m)', title='Extended AUTO route', aspect='equal')
    axes[0, 1].set(xlabel='Seconds after AUTO start', ylabel='Relative height (m)', title='Mission altitude')
    axes[1, 0].set(xlabel='Seconds before leaving first loiter', ylabel='Payload bank (deg)', title='Final 30 s of first loiter')
    axes[1, 1].set(xlabel='Seconds before leaving first loiter', ylabel='Loiter radius (m)', title='First loiter tracking')
    axes[1, 1].axhline(25, color='gray', ls='--', label='25 m demand')
    for ax in axes.flat:
        ax.grid(alpha=.25)
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(a.output / 'voltage-auto.png', dpi=150)
    plt.close(fig)
    (a.output / 'auto-summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    with (a.output / 'report.html').open('a') as report:
        report.write('<h2>Extended native AUTO regression</h2><p>Both voltages completed the original extended AUTO test with its acceptance checks unchanged. Each uses the selected candidate with compensation enabled.</p><img src="voltage-auto.png"><table><tr><th>Voltage</th><th>Max waypoint altitude error</th><th>Max corner overshoot</th><th>First-loiter bank standard deviation</th></tr>')
        for row in summary.values():
            report.write(f"<tr><td>{row['voltage']:.1f} V</td><td>{row['max_waypoint_altitude_error_m']:.3f} m</td><td>{row['max_corner_overshoot_m']:.3f} m</td><td>{row['first_loiter_bank_std_deg']:.3f} deg</td></tr>")
        report.write('</table><p><a href="auto-summary.json">AUTO metrics</a></p>')
    print('AUTO', json.dumps(summary, indent=2))
