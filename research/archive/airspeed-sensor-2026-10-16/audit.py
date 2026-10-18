"""Verify sensor use, completed runs and restored normal firmware provenance."""
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
AP = ROOT.parent / 'ardupilot-browser-9dof'

for name in ['eas-zero', 'eas-3000']:
    paths = sorted((HERE / name / 'results').glob('*-lqi.json'))
    assert len(paths) == 16
    ids = set()
    for path in paths:
        d = json.loads(path.read_text())
        ids.add(d['spec']['id'])
        assert d['complete'] and d['failure'] is None
        assert d['spec']['parameters']['ARSPD_TYPE'] == 100
        assert d['spec']['parameters']['SIM_ARSPD_RND'] == 2
        assert len(d['diagnostics']) >= 850
        assert all(len(v['values']) == 43 and v['values'][42] == 1 for v in d['diagnostics'])
    assert ids == set(range(16))
manifest = json.loads((ROOT / 'viewer/assets/runtime/build.json').read_text())
for path, expected in manifest['controller_source_sha256'].items():
    assert hashlib.sha256((AP / path).read_bytes()).hexdigest() == expected
gains = (AP / 'libraries/AP_CustomControl/AP_CustomControl_LQI_gains.h').read_bytes()
assert gains == (ROOT / 'simulation/data/control/mimo/AP_CustomControl_LQI_gains.h').read_bytes()
assert b'#define MICROAGU_LQI_DIAGNOSTICS 0' in gains
print('32/32 corrected flights complete; sensor used in every active diagnostic; normal gains/source hashes restored.')
