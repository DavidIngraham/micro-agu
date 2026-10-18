"""Verify restoration, clean patch reproduction, gains and telemetry exclusion."""
import json,subprocess,tempfile,os
from pathlib import Path
from build import HERE,AP,BASE,SOURCE,sha
D=HERE/os.environ.get('RELEASE_DIR','deployment-v2');runtime=D/'runtime';limit=int(os.environ.get('RELEASE_RATE','25'))
manifest=json.loads((D/'original/manifest.json').read_text())
assert all(sha(Path(r['path']))==r['sha256'] for r in manifest)
assert sha(runtime/'AP_CustomControl_LQI_gains.h')==sha(SOURCE/'AP_CustomControl_LQI_gains.h')
cpp=(runtime/'AP_CustomControl_LQI.cpp').read_text()
assert 'TURNID' not in cpp and 'OBSID' not in cpp and 'printf(' not in cpp
assert f'_turn_max <= {limit}' in cpp and '_handover_time/2.0f' in cpp
files=['AP_CustomControl.cpp','AP_CustomControl_LQI.cpp','AP_CustomControl_LQI.h','AP_CustomControl_LQI_gains.h']
with tempfile.TemporaryDirectory(prefix='microagu-release-patch-') as td:
    root=Path(td);dest=root/'libraries/AP_CustomControl';dest.mkdir(parents=True)
    for name in files:
        (dest/name).write_bytes(subprocess.check_output(['git','show',BASE+':libraries/AP_CustomControl/'+name],cwd=AP))
    subprocess.run(['git','apply','--check',str(runtime/'source.patch')],cwd=root,check=True)
    subprocess.run(['git','apply',str(runtime/'source.patch')],cwd=root,check=True)
    assert all((dest/name).read_bytes()==(runtime/name).read_bytes() for name in files)
out=dict(restored_files=len(manifest),patch_reproduces_source=True,gains_unchanged=True,research_telemetry_absent=True,maximum_rate_parameter=limit,handover_blend_seconds=2,firmware_sha256=sha(runtime/'arduplane.wasm'))
(D/'audit.json').write_text(json.dumps(out,indent=2));print(json.dumps(out))
