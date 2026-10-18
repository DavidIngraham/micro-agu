"""Retain the accepted local backport after the campaign restoration audit."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2];AP=ROOT.parent/'ardupilot-browser-9dof'
assert json.loads((HERE/'acceptance.json').read_text())['local']['accepted']
assert json.loads((HERE/'audit.json').read_text())['runs']==76
assert not (HERE/'local-promotion.json').exists()
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
for r in json.loads((HERE/'original/manifest.json').read_text()):
    assert sha(Path(r['path']))==r['sha256'],r['path']
patch=HERE/'local-filter.patch'
subprocess.run(['git','apply','--check',str(patch)],cwd=AP,check=True)
subprocess.run(['git','apply',str(patch)],cwd=AP,check=True)
parms=ROOT/'simulation/mission-controller-candidate.parm'
assert 'CP3_HRTC' not in parms.read_text()
parms.write_text(parms.read_text()+'\n# Original-observer LQI reference filter; validated 2026-10-17.\nCP3_HRTC 0.7\n')
config_path=ROOT/'viewer/assets/runtime/highdof.json'
config=json.loads(config_path.read_text());assert 'CP3_HRTC' not in config['defaults']
config['defaults']+='\n# LQI reference shaping; existing gains and observer retained\nCP3_HRTC 0.7\n'
config_path.write_text(json.dumps(config,indent=2)+'\n')
runtime=HERE/'runtime/local'
for name in ['arduplane.js','arduplane.wasm']:
    shutil.copyfile(runtime/name,ROOT/'viewer/generated/ardupilot'/name)
meta=json.loads((runtime/'build.json').read_text())
meta['sha256']['highdof.json']=sha(config_path)
meta['controller_reference_filter']={'CP3_HRTC':.7,'climb_limit_m_s':.5,'descent_limit_m_s':.5,
    'canopy_sensor':False,'pitot_sensor':False,'validation':'research/archive/lqi-backport-2026-10-17; local cohort, separate from deployed cohort'}
meta['local_research_build']=True
meta['controller_source_sha256']['libraries/AP_CustomControl/AP_CustomControl.cpp']=sha(runtime/'AP_CustomControl.cpp')
(ROOT/'viewer/assets/runtime/build.json').write_text(json.dumps(meta,indent=2)+'\n')
for name in ['AP_CustomControl.cpp','AP_CustomControl_LQI.cpp','AP_CustomControl_LQI.h','AP_CustomControl_LQI_gains.h']:
    assert sha(AP/'libraries/AP_CustomControl'/name)==sha(runtime/name),name
(HERE/'local-promotion.json').write_text(json.dumps(dict(patch_sha256=sha(patch),
    firmware_sha256=sha(runtime/'arduplane.wasm'),config_sha256=sha(config_path),
    note='Local research source and runtime retained; website uses the independently tested deployed cohort.'),indent=2)+'\n')
