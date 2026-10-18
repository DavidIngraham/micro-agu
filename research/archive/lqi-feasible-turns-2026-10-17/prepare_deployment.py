"""Stage only the qualified browser controller and its reproducible source."""
import hashlib,json,shutil,os
from pathlib import Path
from urllib.request import urlopen
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2]
LIMIT=int(os.environ.get('RELEASE_RATE','25'))
SITE=ROOT.parent/'microagu-pages-publish/assets/microagu-viewer';OUT=HERE/os.environ.get('RELEASE_OUTPUT','release');DEPLOY=HERE/os.environ.get('RELEASE_DIR','deployment-v2');RUNTIME=DEPLOY/'runtime'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    accepted=json.loads((DEPLOY/'qualification.json').read_text());assert accepted['accepted'],accepted
    fresh=json.loads((DEPLOY/('disturbance.json' if LIMIT==35 else 'fresh/qualification.json')).read_text());assert fresh['accepted'],fresh
    assert accepted['firmware_sha256']==sha(RUNTIME/'arduplane.wasm')
    assert not json.loads((DEPLOY/'restoration.json').read_text())['conflicts']
    manifest=json.loads((SITE/'simulator-manifest.json').read_text());before=json.loads((SITE/'wasm/build.json').read_text())
    with urlopen('https://davidingraham.github.io/assets/microagu-viewer/wasm/build.json',timeout=30) as r:live=json.load(r)
    assert before['sha256']==live['sha256']
    for rel,expected in manifest['files'].items():
        assert sha(SITE/rel)==expected,rel
        dest=OUT/rel;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(SITE/rel,dest)
    for name in ['arduplane.js','arduplane.wasm']:shutil.copyfile(RUNTIME/name,OUT/'wasm'/name)
    config=json.loads((OUT/'wasm/highdof.json').read_text())
    if LIMIT==35:
        config['defaults']='\n'.join(line for line in config['defaults'].splitlines() if not line.split() or line.split()[0] not in ('CP3_RMAX','PG_TURN_RMAX'))+'\n'
        config['defaults']+='\n# Qualified 35 deg/s browser turn-reference envelope\nCP3_RMAX 35\nPG_TURN_RMAX 35\n'
    else:
        config['defaults']+='\n# Qualified browser-only feasible-turn LQI, original K/L and EKF sensing\nCP3_RMAX 25\nCP3_AWMODE 1\nCP3_TMAP 1\nCP3_OBSTRIM 1\n'
    (OUT/'wasm/highdof.json').write_text(json.dumps(config,indent=2)+'\n')
    # Complete patch against the same public AP base, not a patch stacked onto the old one.
    patch='wasm/lqi-feasible-turns.patch';details='wasm/lqi-feasible-turns-validation.json'
    shutil.copyfile(RUNTIME/'source.patch',OUT/patch)
    (OUT/details).write_text(json.dumps(dict(ai_assisted=True,qualification=accepted,additional_validation=fresh,
        comparison=json.loads((HERE/'reassessment.json').read_text())['paired_25'],
        scope=f'Browser simulation only. {LIMIT} deg/s cap; unchanged K/L, as-built geometry and EKF sensing. Original disturbance screens retained; paired analysis is not a physical safety certification.',
        limits='Pitch peaks are mixed; 40 deg/s is not deployed. PG + TECS remains the default.'),indent=2)+'\n')
    meta=before.copy();meta['sha256']=dict(before['sha256'])
    for name in ['arduplane.js','arduplane.wasm','highdof.json']:meta['sha256'][name]=sha(OUT/'wasm'/name)
    meta['controller_source_sha256']={f'libraries/AP_CustomControl/{name}':sha(RUNTIME/name) for name in ['AP_CustomControl.cpp','AP_CustomControl_LQI.cpp','AP_CustomControl_LQI.h','AP_CustomControl_LQI_gains.h']}
    meta['controller_source_patch']=dict(path=Path(patch).name,sha256=sha(OUT/patch),base_revision=before['revision'],description='AI-assisted feasible trim map, anti-windup correction and observer equilibrium recentering. Complete patch against base revision; no research telemetry.')
    meta['controller_turn_envelope']=dict(CP3_RMAX=LIMIT,PG_TURN_RMAX=LIMIT,CP3_AWMODE=1,CP3_TMAP=1,CP3_OBSTRIM=1,handover_blend_seconds=2,feedback_gains_unchanged=True,observer_gains_unchanged=True,validation=Path(details).name)
    (OUT/'wasm/build.json').write_text(json.dumps(meta,indent=2)+'\n')
    for name in [patch,details]:
        manifest['files'][name]=sha(OUT/name)
        if name not in manifest['assets']:manifest['assets'].append(name)
    changed=[]
    for rel in manifest['files']:
        value=sha(OUT/rel)
        if not (SITE/rel).exists() or value!=sha(SITE/rel):changed.append(rel)
        manifest['files'][rel]=value
    assert set(changed)<=set(['wasm/arduplane.js','wasm/arduplane.wasm','wasm/highdof.json','wasm/build.json',patch,details])
    (OUT/'applet-manifest.json').write_text(json.dumps(dict(schema='microagu.applets.v1',applets={'simulator':manifest}),indent=2)+'\n')
    (DEPLOY/'release-diff.json').write_text(json.dumps(dict(changed=changed,firmware=meta['sha256']['arduplane.wasm']),indent=2));print(changed)
if __name__=='__main__':main()
