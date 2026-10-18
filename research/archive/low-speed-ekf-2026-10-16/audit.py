"""Check the retained evidence and final non-diagnostic local build."""
import hashlib
import json
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
gains = []
for name in ['baseline', 'adaptive']:
    folder = HERE / name
    gains.append(hashlib.sha256((folder / 'AP_CustomControl_LQI_gains.h').read_bytes()).hexdigest())
    files = sorted((folder / 'results').glob('*-lqi.json'))
    assert len(files) == 16
    for path in files:
        d = json.loads(path.read_text())
        assert d['complete'] and d['failure'] is None, path
        v = np.array([r['values'] for r in d['diagnostics']])
        assert len(v) >= 850 and v.shape[1] == 43, (path, v.shape)
        assert np.isfinite(v).all() and (v[:, 42] == 0).all(), path
        if name == 'adaptive':
            assert any(s.startswith('N3LD,') for s in d['logs']), path
assert gains[0] == gains[1], 'Controller gains differ'
ap = ROOT.parent / 'ardupilot-browser-9dof'
assert (HERE / 'production-gains.h').read_bytes() == (ap / 'libraries/AP_CustomControl/AP_CustomControl_LQI_gains.h').read_bytes()
firmware = (ROOT / 'viewer/generated/ardupilot/arduplane.wasm').read_bytes()
assert b'N3LD,' not in firmware and b'LQID,' not in firmware, 'Diagnostics remain in final firmware'
print('PASS: 32 complete sensorless flights, finite diagnostics, identical gains, production gains restored, final debug streams absent')
