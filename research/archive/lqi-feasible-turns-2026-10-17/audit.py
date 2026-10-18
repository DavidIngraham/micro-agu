"""Verify experimental provenance and restoration without touching source trees."""
import hashlib,json
from pathlib import Path
import numpy as np
from build import HERE,VIEW,SOURCE,sha
from local_analysis import arrays
def main():
    manifest=json.loads((HERE/'original/manifest.json').read_text())
    assert all(sha(Path(r['path']))==r['sha256'] for r in manifest)
    assert sha(HERE/'runtime/AP_CustomControl_LQI_gains.h')==sha(SOURCE/'AP_CustomControl_LQI_gains.h')
    published=sha(VIEW/'generated/ardupilot/arduplane.wasm');assert published=='b6acedf6ed9f9729af8a125c7f5a555d854893f693b2ea38dc6609d747acd167'
    physics=sha(VIEW/'generated/physics/sim_wasm_bg.wasm');assert physics=='f5d479506baad4065fbfa082e985ab2d930018d7bba700b0cc184102b71a636e'
    z=arrays((HERE/'runtime/AP_CustomControl_LQI_gains.h').read_text());base=np.load(HERE/'base.npz')
    assert np.max(abs(base['u']-z['TRIM']))<1e-8
    trims=json.loads((HERE/'trims.json').read_text());used=[r for r in trims if r['feasible'] and r['residual']<1e-6 and abs(r['rate'])<=40]
    assert all(0<=r['u'][0]<=.6 and r['u'][1]>=0 and r['u'][1]+abs(r['u'][2])<=1.000001 for r in used)
    records=[]
    for p in HERE.glob('*/*/*/*-metrics.json'):
        r=json.loads(p.read_text());records.append(r)
    out=dict(restored_files=len(manifest),unchanged_feedback_observer_gains=True,unchanged_published_runtime=True,
        modeled_nodes=len(trims),accepted_table_nodes=len(used),max_table_residual=max(r['residual'] for r in used),
        runs=len(records),completed=sum(r['complete'] for r in records),research_firmware_sha256=sha(HERE/'runtime/arduplane.wasm'),physics_sha256=physics,
        generator_sha256=sha(HERE/'trim_grid.py'),controller_transform_sha256=sha(HERE/'modify.py'),harness_sha256=sha(HERE/'case.mts'))
    (HERE/'audit.json').write_text(json.dumps(out,indent=2));print(json.dumps(out))
if __name__=='__main__':main()
