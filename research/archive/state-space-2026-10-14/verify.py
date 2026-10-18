"""Numerical consistency checks for the retained research, not firmware unit tests."""
import json
import re
from pathlib import Path
import numpy as np
from study import HERE, DT, BASE, plants, controller, output_matrix, performance, save

def main():
    cfg=json.loads((HERE/'optimization.json').read_text())['candidate'];checks=[]
    for p in plants():
        for name,c in [('baseline',BASE),('candidate',cfg)]:
            predicted=performance(p,c);assert predicted is not None
            F,G,_,U,_=controller(p,c);Y=output_matrix(p,U)
            x=-np.linalg.solve(np.eye(24)-F,G@np.diag([1.,10.]));total=0.
            for _ in range(round(300/DT)):
                total+=DT*np.sum((Y@x)**2);x=F@x
            err=abs(total-predicted['step'])/predicted['step'];assert err<1e-6,(p['name'],name,err)
            checks.append(dict(case=p['name'],controller=name,lyapunov_vs_time_sum_relative_error=float(err)))
        if p['name']!='dynamic_climb':
            b=performance(p,BASE);c=performance(p,cfg)
            assert c['gust']<=b['gust']*(1+1e-6)
            assert all(c['step_components'][i]<=b['step_components'][i]*(1+1e-6) for i in [0,1])
            assert c['pole_real']<=.8*b['pole_real']+1e-6
    v=json.loads((HERE/'linear-validation.json').read_text())
    assert len(v['rows'])==4
    assert max(max(r['relative_l2_errors']) for r in v['rows'])<.001
    s=json.loads((HERE/'robustness.json').read_text());assert len(s)==1296
    assert all(r['max_real']<0 for r in s)
    oracle=json.loads((HERE/'oracle-screen.json').read_text());assert oracle['evaluated']==72
    article=HERE.parents[1]/'docs/controller-state-space.md';missing=[]
    for target in re.findall(r'\]\(([^)]+)\)',article.read_text()):
        if '://' in target or target.startswith('#'):continue
        path=(article.parent/target.split('#')[0]).resolve()
        if not path.exists():missing.append(str(path))
    assert not missing,missing
    native=[]
    if (HERE/'native-results.json').exists():
        for r in json.loads((HERE/'native-results.json').read_text()):
            assert r['returncode']==0 and 'metrics' in r
            actual=r['metrics']['parameters'];want=json.loads((HERE/f'native-{r["controller"]}.json').read_text())['parameters']
            tested=[]
            for key,value in want.items():
                if key in actual:
                    assert np.isclose(actual[key],value,rtol=2e-6,atol=2e-6),(key,actual[key],value)
                    tested.append(key)
            assert not r['metrics']['backend']['errors']
            assert r['metrics']['backend']['clipped_packets']==0
            native.append(dict(controller=r['controller'],condition=r['condition'],audited_parameters=tested))
    save('verification.json',dict(linear_cost_checks=checks,small_signal_checks=v['rows'],
         robustness_cases=len(s),local_links_valid=True,native_audits=native))
    print('PASS:',len(checks),'independent cost checks;',len(v['rows']),'nonlinear equivalence checks;',len(s),'stable uncertainty cases;',len(native),'native parameter audits')

if __name__=='__main__':main()
