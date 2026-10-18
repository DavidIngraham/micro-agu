"""Offline evidence for low-speed/sideslip assumptions; no EKF fusion-state inference."""
import json
from pathlib import Path
import re
import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent

def arrays(path):
    return {name: np.array(json.loads(value.replace('{', '[').replace('}', ']').replace('f', '')))
            for name, value in re.findall(r'static constexpr float (\w+)(?:\[\d+\])+ = (.*);', path.read_text())}

rows = []
for folder in [HERE.parent / 'filtered-brake-2026-10-16/diagnostic-zero', HERE / 'eas-zero']:
    gains = arrays(folder / 'AP_CustomControl_LQI_gains.h')
    for path in sorted((folder / 'results').glob('*-lqi.json')):
        d = json.loads(path.read_text())
        for window, lo, hi in [('disturbed', 20, 100), ('late', 150, 180)]:
            s = [s for s in d['samples'] if lo <= s['time'] < hi]
            state = np.array([s['truth'] for s in s])
            wind = np.array([s['wind'] for s in s])
            rp = Rotation.from_quat(state[:, 3:7])
            rc = Rotation.from_quat(state[:, 16:20])
            payload_air = rp.inv().apply(state[:, 7:10]-wind)
            canopy_air = rc.inv().apply(state[:, 20:23]-wind)
            beta = np.degrees(np.arctan2(payload_air[:, 1], np.hypot(payload_air[:, 0], payload_air[:, 2])))
            canopy_beta = np.degrees(np.arctan2(canopy_air[:, 1], np.hypot(canopy_air[:, 0], canopy_air[:, 2])))
            attitude = Rotation.from_euler('xyz', [[x['roll'], x['pitch'], x['yaw']] for x in s])
            attitude_error = np.degrees((attitude.inv()*rp).magnitude())
            v = np.array([r['values'] for r in d['diagnostics'] if lo <= r['time'] < hi])
            air = v[:, 9:12]*gains['XS'][5:8]+gains['VELOCITY']
            estimated_beta = np.degrees(np.arctan2(air[:, 1], np.hypot(air[:, 0], air[:, 2])))
            # The EKF gate uses delayed internal states; this is only an output-level proxy.
            rows.append(dict(configuration=folder.name, id=d['spec']['id'], mode=d['spec']['mode'], window=window,
                             payload_beta_mean=float(beta.mean()), payload_beta_rms=float(np.sqrt(np.mean(beta**2))),
                             payload_beta_abs_p95=float(np.quantile(abs(beta), .95)),
                             canopy_beta_rms=float(np.sqrt(np.mean(canopy_beta**2))),
                             payload_forward_mean=float(payload_air[:, 0].mean()),
                             estimated_forward_mean=float(air[:, 0].mean()),
                             estimated_lateral_mean=float(air[:, 1].mean()),
                             estimated_beta_mean=float(estimated_beta.mean()),
                             truth_tas_mean=float(np.linalg.norm(payload_air, axis=1).mean()),
                             estimated_tas_mean=float(v[:, 5].mean()),
                             estimated_forward_over_5_fraction=float(np.mean(air[:, 0] > 5)),
                             groundspeed_over_5_fraction=float(np.mean(np.linalg.norm(state[:, 7:9], axis=1) > 5)),
                             attitude_error_rms=float(np.sqrt(np.mean(attitude_error**2))),
                             vertical_wind_rms=float(np.sqrt(np.mean(wind[:, 2]**2)))))
(HERE / 'estimator-diagnosis.json').write_text(json.dumps(rows, indent=2)+'\n')
for configuration in ['diagnostic-zero', 'eas-zero']:
    for window in ['disturbed', 'late']:
        group = [r for r in rows if r['configuration'] == configuration and r['window'] == window]
        keys = ['payload_beta_rms', 'payload_beta_abs_p95', 'canopy_beta_rms', 'attitude_error_rms',
                'estimated_forward_over_5_fraction', 'groundspeed_over_5_fraction']
        print(configuration, window, {k: round(float(np.median([r[k] for r in group])), 4) for k in keys})
for r in rows:
    if r['configuration'] == 'diagnostic-zero' and r['window'] == 'late':
        print(r['id'], {k: round(v, 4) for k, v in r.items() if isinstance(v, float)})
