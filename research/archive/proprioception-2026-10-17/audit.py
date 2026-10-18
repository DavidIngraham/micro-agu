"""Check fixed-gain equivalence, sensor activation and shaft-power calculation."""
import importlib.util
import json
from pathlib import Path
import re
import sys
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
spec = importlib.util.spec_from_file_location('proprio_analysis', HERE/'analyze.py')
analysis = importlib.util.module_from_spec(spec); spec.loader.exec_module(analysis)


def arrays(path):
    return {name: np.asarray(json.loads(value.replace('{', '[').replace('}', ']').replace('f', '')))
            for name, value in re.findall(r'static constexpr float (\w+)(?:\[\d+\])+ = (.*);', path.read_text())}


original = arrays(HERE/'original/3')
fixed = arrays(HERE/'fixed-design/AP_CustomControl_LQI_gains.h')
differences = {k: float(np.max(abs(v-fixed[k]))) for k, v in original.items()}
assert max(differences.values()) < 1e-8, differences
native_path = next((ROOT/'simulation/.venv/lib/python3.12/site-packages/microagu_sim').glob('_native*.so'))
spec = importlib.util.spec_from_file_location('_native', native_path)
native = importlib.util.module_from_spec(spec); spec.loader.exec_module(native)
errors = []
for rpm in [1000., 5000., 9000., 14000.]:
    for axial in [0., 2., 4., 8.]:
        expected = native.mapped_torque(rpm, axial)*rpm*2*np.pi/60
        actual = analysis.shaft_power(np.array([rpm]), np.array([axial]))[0]
        errors.append(abs(actual-expected))
assert max(errors) < 1e-8, errors
checks = []
expected_modes = dict(sensorless=0, airspeed=0, angles=1, full=2, sensor=3, stress=3)
for group in ['fixed', 'efficient', 'damping']:
    for path in (HERE/group).glob('*/results/*-lqi.json'):
        result = json.loads(path.read_text())
        variant = path.parents[1].name
        diagnostics = result['diagnostics']
        if not result['complete']:
            checks.append(dict(path=str(path.relative_to(HERE)), complete=False))
            continue
        active = [r['values'] for r in diagnostics if 20 <= r['time'] < 180]
        assert active and all(len(r) == 43 for r in active), path
        use = np.mean([r[42] for r in active])
        assert use == (0 if variant == 'sensorless' else 1), (path, use)
        modes = {int(s.split(',')[2]) for s in result['logs'] if s.startswith('LQIP,')}
        assert modes == {expected_modes[variant]}, (path, modes)
        checks.append(dict(path=str(path.relative_to(HERE)), complete=True,
                           airspeed_use=float(use), mode=expected_modes[variant]))
report = dict(fixed_gain_max_absolute_difference=max(differences.values()),
              shaft_power_native_max_error=max(errors), runs=checks)
(HERE/'audit.json').write_text(json.dumps(report, indent=2)+'\n')
print('Fixed gains match, power matches native kernel, sensor modes verified:', len(checks), 'runs')
