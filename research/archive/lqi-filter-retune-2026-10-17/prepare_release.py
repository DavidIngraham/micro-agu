"""Prepare only the qualified controller change, retaining published physics/UI."""
import hashlib,json,shutil
from pathlib import Path
from urllib.request import urlopen
HERE=Path(__file__).resolve().parent;BACK=HERE.parent/'lqi-backport-2026-10-17'
ROOT=HERE.parents[2];SITE=ROOT.parent/'microagu-pages-publish/assets/microagu-viewer';OUT=HERE/'release'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
acceptance=json.loads((HERE/'acceptance.json').read_text());assert acceptance['accepted'],acceptance
assert json.loads((HERE/'audit.json').read_text())['runtime_hashes_verified']
chosen=acceptance['selected'];assert not OUT.exists()
manifest=json.loads((SITE/'simulator-manifest.json').read_text());before=json.loads((SITE/'wasm/build.json').read_text())
with urlopen('https://davidingraham.github.io/assets/microagu-viewer/wasm/build.json',timeout=30) as r:live=r.read()
(HERE/'live-before.json').write_bytes(live)
assert before['sha256']==json.loads(live)['sha256']
assert before['revision']=='010ada6a4f0b26910fc18ecca176d2e65c5ed9d7'
for rel,expected in manifest['files'].items():
    assert sha(SITE/rel)==expected,rel
    dest=OUT/rel;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(SITE/rel,dest)
runtime=BACK/'runtime/deployed' if chosen=='filtered' else HERE/'designs'/chosen/'runtime'
for name in ['arduplane.js','arduplane.wasm']:shutil.copyfile(runtime/name,OUT/'wasm'/name)
config=json.loads((OUT/'wasm/highdof.json').read_text());assert 'CP3_HRTC' not in config['defaults']
config['defaults']+='\n# LQI reference shaping; original sensing retained\nCP3_HRTC 0.7\n'
(OUT/'wasm/highdof.json').write_text(json.dumps(config,indent=2)+'\n')
patch='wasm/lqi-filter-tune.patch';design='wasm/lqi-filter-design.json'
shutil.copyfile(BACK/'deployed-filter.patch' if chosen=='filtered' else HERE/'designs'/chosen/'published-source.patch',OUT/patch)
details=dict(selected=chosen,filter_tau_s=.7,study='lqi-filter-retune-2026-10-17',validation=acceptance,
    weights=None if chosen=='filtered' else json.loads((HERE/'designs'/chosen/'design.json').read_text())['weights'])
(OUT/design).write_text(json.dumps(details,indent=2)+'\n')
meta=before.copy();meta['sha256']=dict(before['sha256'])
for name in ['arduplane.js','arduplane.wasm','highdof.json']:meta['sha256'][name]=sha(OUT/'wasm'/name)
meta['working_tree_dirty']=True
meta['controller_source_sha256']={f'libraries/AP_CustomControl/{name}':sha(runtime/name) for name in ['AP_CustomControl.cpp','AP_CustomControl_LQI.cpp','AP_CustomControl_LQI.h','AP_CustomControl_LQI_gains.h']}
meta['controller_source_patch']=dict(path=Path(patch).name,sha256=sha(OUT/patch),base_revision=before['revision'],description='AI-assisted reference-filter tuning; apply patch to base_revision. Original observer and sensing retained.')
meta['controller_reference_filter']=dict(CP3_HRTC=.7,climb_limit_m_s=.5,descent_limit_m_s=.5,canopy_sensor=False,pitot_sensor=False,selected=chosen,validation='Corrected lateral gust incidence; fresh Monte Carlo commands and AUTO/CRUISE missions, 2026-10-17')
(OUT/'wasm/build.json').write_text(json.dumps(meta,indent=2)+'\n')
for name in [patch,design]:manifest['files'][name]=sha(OUT/name);manifest['assets'].append(name)
changed=[]
for rel in manifest['files']:
    value=sha(OUT/rel)
    if not (SITE/rel).exists() or value!=sha(SITE/rel):changed.append(rel)
    manifest['files'][rel]=value
assert set(changed)<=set(['wasm/arduplane.js','wasm/arduplane.wasm','wasm/highdof.json','wasm/build.json',patch,design])
(OUT/'applet-manifest.json').write_text(json.dumps(dict(schema='microagu.applets.v1',applets={'simulator':manifest}),indent=2)+'\n')
(HERE/'release-diff.json').write_text(json.dumps(dict(changed=changed,files=manifest['files']),indent=2))
print(json.dumps(dict(changed=changed,firmware=meta['sha256']['arduplane.wasm']),indent=2))
