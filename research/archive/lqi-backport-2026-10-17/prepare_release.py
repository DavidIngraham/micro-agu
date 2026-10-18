"""Prepare the verified publisher input; retain the deployed frontend and physics."""
import hashlib
import json
from pathlib import Path
import shutil

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
SITE=ROOT.parent/'microagu-pages-publish/assets/microagu-viewer'
OUT=HERE/'release'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
acceptance=json.loads((HERE/'acceptance.json').read_text())
assert acceptance['deployed']['accepted'],acceptance['deployed']
assert not OUT.exists(), 'Refuse to overwrite a prepared release'
manifest=json.loads((SITE/'simulator-manifest.json').read_text())
before=json.loads((SITE/'wasm/build.json').read_text())
live=json.loads((HERE/'live-before.json').read_text())
assert before['sha256']==live['sha256'], 'Published baseline differs from tested cohort'
for rel,expected in manifest['files'].items():
    assert sha(SITE/rel)==expected,rel
    dest=OUT/rel;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(SITE/rel,dest)
runtime=HERE/'runtime/deployed'
for name in ['arduplane.js','arduplane.wasm']:shutil.copyfile(runtime/name,OUT/'wasm'/name)
config=json.loads((OUT/'wasm/highdof.json').read_text())
assert 'CP3_HRTC' not in config['defaults']
config['defaults']+='\n# LQI reference shaping; existing gains and observer retained\nCP3_HRTC 0.7\n'
(OUT/'wasm/highdof.json').write_text(json.dumps(config,indent=2)+'\n')
patch='wasm/lqi-reference-filter.patch'
shutil.copyfile(HERE/'deployed-filter.patch',OUT/patch)
meta=before.copy();meta['sha256']=dict(before['sha256'])
for name in ['arduplane.js','arduplane.wasm','highdof.json']:meta['sha256'][name]=sha(OUT/'wasm'/name)
meta['working_tree_dirty']=True
meta['controller_source_sha256']={f'libraries/AP_CustomControl/{name}':sha(runtime/name) for name in
    ['AP_CustomControl.cpp','AP_CustomControl_LQI.cpp','AP_CustomControl_LQI.h','AP_CustomControl_LQI_gains.h']}
meta['controller_source_patch']={'path':'lqi-reference-filter.patch','sha256':sha(OUT/patch),
    'base_revision':before['revision'],'description':'AI-assisted configurable reference filter; apply this patch to base_revision. Gains and original observer unchanged.'}
meta['controller_reference_filter']={'CP3_HRTC':.7,'climb_limit_m_s':.5,'descent_limit_m_s':.5,
    'canopy_sensor':False,'pitot_sensor':False,'validation':'Original-observer backport study, 2026-10-17; paired commands/gusts and AUTO/CRUISE missions.'}
(OUT/'wasm/build.json').write_text(json.dumps(meta,indent=2)+'\n')
manifest['files'][patch]=sha(OUT/patch);manifest['assets'].append(patch)
changed=[]
for rel in manifest['files']:
    value=sha(OUT/rel)
    if not (SITE/rel).exists() or value!=sha(SITE/rel):changed.append(rel)
    manifest['files'][rel]=value
assert set(changed)<=set(['wasm/arduplane.js','wasm/arduplane.wasm','wasm/highdof.json','wasm/build.json',patch]),changed
(OUT/'applet-manifest.json').write_text(json.dumps(dict(schema='microagu.applets.v1',applets={'simulator':manifest}),indent=2)+'\n')
(HERE/'release-diff.json').write_text(json.dumps(dict(changed=changed,files=manifest['files']),indent=2))
print(json.dumps(dict(changed=changed,firmware=meta['sha256']['arduplane.wasm']),indent=2))
