"""Summarize tracking, recovery, power and onboard common-brake contributions."""
import argparse
import json
from pathlib import Path
import re
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
ZERO = HERE.parent / 'zero-brake-2026-10-16'
sys.path.insert(0, str(ZERO))
from compare import assess, metrics


def arrays(path):
    result = {}
    for name, value in re.findall(r'static constexpr float (\w+)(?:\[\d+\])+ = (.*);', path.read_text()):
        result[name] = np.array(json.loads(value.replace('{', '[').replace('}', ']').replace('f', '')))
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('trial')
    args = parser.parse_args()
    folder = HERE / args.trial
    gains = arrays(folder / 'AP_CustomControl_LQI_gains.h')
    report = []
    for path in sorted((folder / 'results').glob('*-lqi.json')):
        d = json.loads(path.read_text())
        row = assess(path)
        row['early_recovery'] = metrics([s for s in d['samples'] if 105 <= s['time'] < 135])
        row['previous'] = assess(ZERO / 'campaign/results' / path.name)
        late = [r['values'] for r in d.get('diagnostics', []) if 150 <= r['time'] < 180]
        if late:
            v = np.array(late)
            assert v.shape[1] in (42, 43), v.shape
            ref, state, integral = v[:, 1:5], v[:, 16:32], v[:, 32:35]
            target = ref @ gains['XREF'].T
            terms = {
                'feedforward': gains['US'][1]*(ref @ gains['UREF'][1]),
                'state_feedback': -gains['US'][1]*((state-target) @ gains['K'][1, :16]),
                'integral_feedback': -gains['US'][1]*(integral @ gains['K'][1, 16:19]),
                'memory_feedback': -gains['US'][1]*gains['K'][1, 19]*v[:, 41],
                'requested': v[:, 36], 'applied': v[:, 39],
                'airspeed_measured': v[:, 5], 'airspeed_target': v[:, 6],
                'height_integral': integral[:, 0], 'turn_integral': integral[:, 1],
                'speed_integral': integral[:, 2],
                'speed_integral_common': -gains['US'][1]*gains['K'][1, 18]*integral[:, 2],
                'height_integral_common': -gains['US'][1]*gains['K'][1, 16]*integral[:, 0],
                'turn_integral_common': -gains['US'][1]*gains['K'][1, 17]*integral[:, 1],
                'air_vector_speed': np.linalg.norm(v[:, 9:12]*gains['XS'][5:8]+gains['VELOCITY'], axis=1),
            }
            samples = [s for s in d['samples'] if 150 <= s['time'] < 180]
            terms['truth_airspeed'] = np.array([np.linalg.norm(np.array(s['truth'][7:10])-s['wind']) for s in samples])
            row['diagnostic_late_means'] = {k: float(np.mean(x)) for k, x in terms.items()}
            if v.shape[1] == 43:
                row['airspeed_sensor_use_fraction'] = float(np.mean([r['values'][42] for r in d['diagnostics']]))
            reconstruction = (terms['feedforward']+terms['state_feedback']+
                              terms['integral_feedback']+terms['memory_feedback']+gains['TRIM'][1])
            row['late_reconstruction_max_error'] = float(np.max(abs(reconstruction-v[:, 36])))
            assert row['late_reconstruction_max_error'] < 1e-5, 'Diagnostic command reconstruction mismatch'
        report.append(row)
    (folder / 'analysis.json').write_text(json.dumps(report, indent=2)+'\n')
    for r in report:
        if not r['complete']:
            print(r['id'], r['failure']); continue
        print(r['id'], r['mode'], 'alt/xtrack/pitch/common/W',
              *[round(x, 3) for x in [r['disturbed']['altitude_rms_m'], r['disturbed']['xtrack_rms_m'],
                                     r['late']['pitch_ac_deg'], 100*r['late']['additional_common_fraction'],
                                     r['combined_power']['estimated_shaft_W']]])
        print('  diagnostic', r.get('diagnostic_late_means', 'MISSING'))
    for mode in ['AUTO', 'CRUISE']:
        group = [r for r in report if r['mode'] == mode and r['complete']]
        for phase, key in [('disturbed', 'altitude_rms_m'), ('disturbed', 'xtrack_rms_m'),
                           ('combined_power', 'estimated_shaft_W')]:
            def val(r): return r[phase][key]
            print(mode, key, 'new/previous medians', np.median([val(r) for r in group]),
                  np.median([val(r['previous']) for r in group]))


if __name__ == '__main__':
    main()
