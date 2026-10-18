"""Replay frozen validation with physical pose recording; assert unchanged metrics."""
import json
import sys
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import numpy as np
import experiment as ex
from imu_study import final_job

if __name__ == '__main__':
    old=json.loads((ex.HERE/'iteration2/final-validation.json').read_text())
    selected=json.loads((ex.HERE/'iteration2/refined-codesign-selection.json').read_text())
    cases=[]
    for c in ex.scenarios():
        c=dict(c);c['seed']+=3000;c['gust']*=1.2
        if c['name']=='maneuver':c.update(maneuver_height=1.5,maneuver_turn=8.)
        cases.append(c)
    jobs=[(c,selected['config'],f,selected['selected']['variant'],o) for c in cases for o in [False,True] for f in ['baseline','direct']]
    with ProcessPoolExecutor(max_workers=4) as pool:rows=list(pool.map(final_job,jobs))
    target=Path(sys.argv[1]);target.mkdir(exist_ok=True,parents=True)
    for r in rows:
        original=next(x for x in old if all(x[k]==r[k] for k in ['case','family','observer']))
        for k in ['height_rms','pitch_rms','heading_rms']:assert abs(r[k]-original[k])<1e-10,(r['case'],k)
        p=np.load(ex.HERE/'traces'/f'{r["case"]}-{r["controller"]}.npz')['poses']
        assert p.shape==(801,32) and np.isfinite(p).all()
        for k in [4,17]:assert np.max(abs(np.linalg.norm(p[:,k:k+4],axis=1)-1))<1e-6
        name=f'{r["case"]}-{"observer" if r["observer"] else "truth"}-{r["family"]}.json'
        (target/name).write_text(json.dumps(np.round(p,7).tolist(),separators=(',',':')))
    print('32 pose recordings match frozen validation metrics; normalized quaternions verified')
