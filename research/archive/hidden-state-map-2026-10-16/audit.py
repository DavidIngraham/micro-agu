"""Numerical resolution, input provenance and result-completeness checks."""
import hashlib
import json
from pathlib import Path
import numpy as np
from run import HERE, ROOT, est, prior, segment


def main():
    report = {'integration': {}, 'fixed_fits': 0, 'held_out_forecasts': 0}
    for flight, start in [('hood_river', 300), ('trout_lake', 600)]:
        s = segment(flight, start)
        x = np.load(HERE/f'fixed-{s.name}-nominal.npz')['x']
        a = est.Graph([s], substeps=32).residual(x)
        b = est.Graph([s], substeps=64).residual(x)
        error = float(np.max(abs(a-b)))
        assert error < 1e-3
        report['integration'][s.name] = dict(max_normalized_residual_change=error)
    for path in HERE.glob('fixed-*.json'):
        result = json.loads(path.read_text())
        assert result['success'], path
        assert np.isfinite(np.load(path.with_suffix('.npz'))['x']).all(), path
        report['fixed_fits'] += 1
    assert report['fixed_fits'] == 40
    for path in HERE.glob('forecast-*.npz'):
        result = np.load(path)
        assert np.isfinite(result['prediction']).all(), path
        assert len(result['time']) == 751
        report['held_out_forecasts'] += 1
    assert report['held_out_forecasts'] == 16
    for kind, expected in [('prefix', 16), ('joint', 3), ('diagnostic', 3)]:
        paths = list(HERE.glob(f'{kind}-*.json'))
        assert len(paths) == expected, (kind, len(paths))
        assert all(json.loads(p.read_text())['success'] for p in paths)
        report[f'{kind}_successful_fits'] = len(paths)
    for seed in [0, 1]:
        result = json.loads((HERE/f'synthetic-{seed}-joint-truth.json').read_text())
        assert result['wind_rmse'] < .15 and result['angle_rmse_deg'] < 2
        assert np.max(abs(np.array(result['fitted_damping'])/[2., 4.]-1)) < .2
    # Verify the entire prior input/native/runtime baseline, excluding its
    # research code (which is independently hashed below).
    old = json.loads((HERE.parent/'wind-calibration-2026-10-16/audit.json').read_text())['hashes']
    paths = [Path(p) for p in old if p.endswith(('.npz', '.so', 'as-built-calibration.json'))]
    for path in paths:
        assert hashlib.sha256(path.read_bytes()).hexdigest() == old[str(path)], path
    paths += list(HERE.glob('*.py')) + [Path(est.__file__), Path(prior.__file__),
                                      Path(prior.prior.__file__), Path(prior.study.__file__)]
    report['hashes'] = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    report['runtime_and_source_inputs_unchanged'] = True
    report['geometry'] = {flight: dict(parameters=segment(flight, starts[0]).parameters,
                                       mass=segment(flight, starts[0]).mass,
                                       hinge=segment(flight, starts[0]).hinge.tolist(),
                                       imu=segment(flight, starts[0]).imu.tolist())
                          for flight, (_, starts) in prior.study.FLIGHTS.items()}
    (HERE/'audit.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k: v for k, v in report.items() if k not in ['hashes', 'geometry']}, indent=2))


if __name__ == '__main__':
    main()
