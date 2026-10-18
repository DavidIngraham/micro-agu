"""AI-assisted exact logged control-law decomposition; no new simulation required."""
import json
from pathlib import Path
import numpy as np
import re

HERE = Path(__file__).resolve().parent
OLD = HERE.parent/'lqi-balanced-2026-10-17'

def decompose(path, design):
    d = json.loads(path.read_text())
    z = np.load(design/'design.npz')
    a = np.array([x['values'] for x in d['diagnostics']])
    t = np.array([x['time'] for x in d['diagnostics']])
    refs, state, integral = a[:,1:5], a[:,16:32], a[:,32:35]
    k, us = z['K'], z['US']
    header=(design/'AP_CustomControl_LQI_gains.h').read_text()
    def array(name):
        match=re.search(r'static constexpr float '+name+r'((?:\[\d+\])+) = ([^;]+);',header)
        shape=list(map(int,re.findall(r'\d+',match[1])))
        return np.array(json.loads(match[2].replace('{','[').replace('}',']').replace('f',''))).reshape(shape)
    # References enter BOTH equilibrium state feedback and direct feedforward.
    parts = dict(reference=(refs@array('UREF').T + (refs@array('XREF').T)@k[:,:16].T)*us,
                 state=-(state@k[:,:16].T)*us,
                 integral=-(integral@k[:,16:19].T)*us,
                 brake_memory=-a[:,41,None]*k[:,19]*us)
    residual = a[:,35:38]-z['command']-sum(parts.values())
    assert np.max(abs(residual[t>10]))<1e-4, 'Logged control law reconstruction mismatch'
    events = {}
    for name, onset in [('climb',20),('descent',120)]:
        window=np.where((t>=onset)&(t<onset+3))[0]
        after=window[np.argmax(abs(np.diff(a[:,4],prepend=a[0,4])[window]))]
        before=after-1
        events[name] = dict(time=float(t[after]),
            first_step={n:(v[after]-v[before]).tolist() for n,v in parts.items()},
            requested_step=(a[after,35:38]-a[before,35:38]).tolist(),
            applied_step=(a[after,38:41]-a[before,38:41]).tolist(),
            max_allocation_residual_first_3s=np.max(abs(a[(t>=onset)&(t<onset+3),35:38]-a[(t>=onset)&(t<onset+3),38:41]),axis=0).tolist())
    return dict(events=events, max_unexplained_after_10s=np.max(abs(residual[t>10]),axis=0).tolist())

if __name__ == '__main__':
    out={name:decompose(OLD/f'training/{name}/01.json',OLD/f'designs/{name}') for name in ['fixed','responsive']}
    (HERE/'decomposition.json').write_text(json.dumps(out,indent=2)+'\n')
    print(json.dumps(out,indent=2))
    if (HERE/'selection.json').exists():
        name=json.loads((HERE/'selection.json').read_text())['selected']
        compared={n:decompose(HERE/f'training/{n}/01.json',HERE/f'designs/{n}') for n in ['fixed',name]}
        (HERE/'decomposition-compared.json').write_text(json.dumps(compared,indent=2)+'\n')
