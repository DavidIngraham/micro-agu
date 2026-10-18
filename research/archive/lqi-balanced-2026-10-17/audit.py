"""Verify restoration, retained binary provenance and actual sensor/governor use."""
import hashlib
import json
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    manifest=json.loads((HERE/'original/manifest.json').read_text())
    for item in manifest:assert sha(Path(item['path']))==item['sha256'],item['path']
    runs=[]
    for phase in ['training','validation','missions']:
        for record in (HERE/phase).glob('*/runs.json'):
            for r in json.loads(record.read_text()):
                runtime=HERE/'designs'/('fixed' if r['variant']=='standard' else r['variant'])/'runtime'
                assert r['firmware_sha256']==sha(runtime/'arduplane.wasm')
                path=record.parent/f'{r["case"]:02}.json';d=json.loads(path.read_text())
                if phase!='training':assert path.stat().st_mtime >= (HERE/'selection.json').stat().st_mtime
                if d['complete'] and r['variant']!='standard':
                    diag=np.array([x['values'] for x in d['diagnostics']]);assert diag.shape[1]==43
                    assert np.all(diag[:,42]==1)
                    modes={int(s.split(',')[2]) for s in d['logs'] if s.startswith('LQIP,')}
                    assert modes=={3}
                    assert max(abs(diag[:,4]))<=1.00001
                assert not any('Unknown parameter' in s for s in d['logs'])
                runs.append(dict(**r,complete=d['complete'],failure=d['failure'],sha256=sha(path)))
    sources={}
    for name in ['campaign.py','score.py','report.py','case.ts','mission.ts','governor.patch','standard-reference-only.patch']:
        sources[name]=sha(HERE/name)
    assets={}
    for p in [ROOT/'viewer/assets/runtime/highdof.json',ROOT/'viewer/generated/physics/sim_wasm_bg.wasm']:
        assets[str(p.relative_to(ROOT))]=sha(p)
    assert assets==json.loads((HERE/'physics-input-hashes.json').read_text())
    for n in ['balanced-case.ts','balanced-mission.ts']:assert not (ROOT/'viewer/tests/integration'/n).exists()
    out=dict(restored_files=len(manifest),run_count=len(runs),complete=sum(r['complete'] for r in runs),
        runs=runs,study_sources=sources,unchanged_physics_assets=assets,
        note='Research source files include the documented correction after preliminary trials. Candidate source/binary hashes are retained separately.')
    (HERE/'audit.json').write_text(json.dumps(out,indent=2)+'\n')
    print(json.dumps({k:out[k] for k in ['restored_files','run_count','complete']}))


if __name__=='__main__':main()
