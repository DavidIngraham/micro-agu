"""Check retained campaign completeness and packaged source provenance."""
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
AP = ROOT.parent / 'ardupilot-browser-9dof'


def main():
    count = 0
    for name, expected in [('diagnostic-zero', set(range(16))),
                            ('memory-1000', {0, 1, 3, 5, 6, 9, 13, 15}),
                            ('memory-3000', set(range(16)))]:
        rows = json.loads((HERE / name / 'analysis.json').read_text())
        assert {r['id'] for r in rows} == expected
        for r in rows:
            assert r['complete'] and r['failure'] is None
            assert r['minimum_applied_brake_mm'] >= 0
            assert r['late_reconstruction_max_error'] < 1e-5
        count += len(rows)
    report = json.loads((ROOT / 'simulation/data/control/mimo/design.json').read_text())
    assert report['brake_memory_weight'] == 0 and not report['diagnostics']
    manifest = json.loads((ROOT / 'viewer/assets/runtime/build.json').read_text())
    for path, expected in manifest['controller_source_sha256'].items():
        assert hashlib.sha256((AP / path).read_bytes()).hexdigest() == expected, path
    assert (ROOT / 'simulation/data/control/mimo/AP_CustomControl_LQI_gains.h').read_bytes() == (
        AP / 'libraries/AP_CustomControl/AP_CustomControl_LQI_gains.h').read_bytes()
    print(f'{count} complete flights; all diagnostic reconstructions and source hashes verified.')


if __name__ == '__main__':
    main()
