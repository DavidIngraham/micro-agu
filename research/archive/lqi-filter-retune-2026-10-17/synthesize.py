"""AI-assisted gain-only synthesis from the exact published controller matrices."""
import ast
import hashlib
import json
import re
from pathlib import Path
import numpy as np
from scipy.linalg import solve_discrete_are
HERE=Path(__file__).resolve().parent
BACK=HERE.parent/'lqi-backport-2026-10-17'
BASE=(BACK/'runtime/deployed/AP_CustomControl_LQI_gains.h').read_text()
def arrays(text):
    return {m[1]:np.array(ast.literal_eval(m[2].replace('{','[').replace('}',']').replace('f','')))
            for m in re.finditer(r'static constexpr float (\w+)(?:\[\d+\])+ = ([^;]+);',text)}
Z=arrays(BASE)
H=np.zeros((3,16));H[0,15]=1
roll,pitch,_=Z['EULER'];H[1,9:11]=np.array([np.sin(roll),np.cos(roll)])/np.cos(pitch)*Z['XS'][9:11]/np.deg2rad(10)
H[2,5:8]=Z['VELOCITY']/np.linalg.norm(Z['VELOCITY'])*Z['XS'][5:8]/.5
A=np.eye(19);A[:16,:16]=Z['A'];A[16:,:16]=.02*H
B=np.zeros((19,3));B[:16]=Z['B']
CANDIDATES={
 'damped':dict(pitch=90),
 'responsive':dict(pitch=90,height=4,integral=12,throttle=500),
 'efficient':dict(pitch=90,common=3000,throttle=700,integral=6),
 'relative':dict(pitch=30,relative_rate=120,relative_angle=10),
 'balanced':dict(pitch=60,relative_rate=60,vertical=4,height=2,integral=6,throttle=700,common=2000),
}
def solve(cfg):
    q=np.diag(np.r_[np.full(16,.02),[cfg.get('integral',3),3,cfg.get('speed_integral',3)]])
    q[:16,:16]+=H.T@np.diag([cfg.get('height',1),1,cfg.get('speed',1)])@H
    q[9,9]+=cfg.get('pitch',30);q[12,12]+=cfg.get('pitch',30)
    rel=np.zeros(19);rel[9]=-1;rel[12]=1
    q+=cfg.get('relative_rate',0)*np.outer(rel,rel)
    q[3,3]+=cfg.get('relative_angle',0);q[7,7]+=cfg.get('vertical',0)
    effort=np.diag([cfg.get('throttle',1000),cfg.get('common',1000),1000])
    p=solve_discrete_are(A,B,q,effort);k=np.linalg.solve(effort+B.T@p@B,B.T@p@A)
    poles=np.linalg.eigvals(A-B@k);assert max(abs(poles))<1
    return k,np.linalg.inv(Z['US'][:,None]*k[:,16:]),poles,q,effort
def main():
    original,*_=solve({});error=float(np.max(abs(original-Z['K'])))
    assert error<1e-6,error
    (HERE/'synthesis-audit.json').write_text(json.dumps(dict(original_gain_reconstruction_max_error=error,
       base_header_sha256=hashlib.sha256(BASE.encode()).hexdigest(),unchanged=['A','B','L','TRIM','XREF','UREF','XS','US']),indent=2))
    for name,cfg in CANDIDATES.items():
        k,aw,poles,q,r=solve(cfg);header=BASE
        def val(x):return '{'+', '.join(val(v) for v in x)+'}' if np.ndim(x) else f'{float(x):.9e}f'
        for key,value in dict(K=k,AW=aw).items():
            header,n=re.subn(r'(static constexpr float '+key+r'(?:\[\d+\])+ = )[^;]+;',lambda m:m[1]+val(value)+';',header);assert n==1
        out=HERE/'designs'/name;out.mkdir(parents=True,exist_ok=True)
        (out/'AP_CustomControl_LQI_gains.h').write_text(header)
        np.savez(out/'design.npz',A=A,B=B,H=H,K=k,AW=aw,Q=q,R=r,poles=poles)
        (out/'design.json').write_text(json.dumps(dict(weights=cfg,max_local_pole=float(max(abs(poles))),
          filter_tau_s=.7,note='Feedback and antiwindup only; observer, trim and reference maps unchanged. Local poles are not nonlinear stability certification.'),indent=2))
if __name__=='__main__':main()
