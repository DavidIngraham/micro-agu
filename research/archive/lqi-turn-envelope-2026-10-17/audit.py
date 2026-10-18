"""Check restoration and unchanged deployed controller design/physics."""
import json
from build import HERE,VIEW,SOURCE,sha
def main():
    original=json.loads((HERE/'original/manifest.json').read_text())
    from pathlib import Path
    restored=[r['path'] for r in original if sha(Path(r['path']))==r['sha256']]
    assert len(restored)==len(original)
    assert sha(HERE/'runtime/AP_CustomControl_LQI_gains.h')==sha(SOURCE/'AP_CustomControl_LQI_gains.h')
    assert sha(VIEW/'generated/ardupilot/arduplane.wasm')=='b6acedf6ed9f9729af8a125c7f5a555d854893f693b2ea38dc6609d747acd167'
    results=[]
    for cohort in ['screen','nominal','diagnostic','comparison','validation']:
        for p in (HERE/cohort).glob('*/*-metrics.json'):
            result=json.loads(p.read_text());results.append(result)
            raw=json.loads(p.with_name(p.name.replace('-metrics','')).read_text())
            if raw['complete'] and result['scheme']=='lqi':
                a=[s['diagnostic'] for s in raw['samples'] if len(s['diagnostic'])==15]
                assert a and max(abs(s[14]-result['rate']) for s in a)<.01
    record=dict(restored_files=len(restored),unchanged_gains_observer_model=True,published_firmware_unchanged=True,
        total_runs=len(results),completed=sum(r['complete'] for r in results),research_firmware_sha256=sha(HERE/'runtime/arduplane.wasm'),
        physics_sha256={p.name:sha(p) for p in (VIEW/'generated/physics').glob('*.wasm')},
        harness_sha256=sha(HERE/'case.mts'),gates_sha256=sha(HERE/'score.py'),configuration_sha256=sha(VIEW/'assets/runtime/highdof.json'))
    (HERE/'audit.json').write_text(json.dumps(record,indent=2));print(json.dumps(record))
if __name__=='__main__':main()
