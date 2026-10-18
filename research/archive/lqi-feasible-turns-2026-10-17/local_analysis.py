"""Local fixed-gain poles at nonlinear trims; not a nonlinear stability proof."""
import ast,json,re
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from build import HERE

def arrays(text):
    return {m[1]:np.array(ast.literal_eval(m[2].replace('{','[').replace('}',']').replace('f',''))) for m in re.finditer(r'static constexpr float (\w+)(?:\[\d+\])+ = ([^;]+);',text)}
def main():
    z=arrays((HERE/'runtime/AP_CustomControl_LQI_gains.h').read_text());K=z['K'];L=z['L'];C=np.eye(16)[[0,1,5,6,7,8,9,10,15]];records=[]
    for r in json.loads((HERE/'trims.json').read_text()):
        if not r['feasible'] or r['residual']>=1e-6:continue
        n=np.load(HERE/f'trim-{r["rate"]}-{r["speed"]:.1f}.npz');a=n['A'];b=n['B'];x=n['x'];
        def output(x):
            roll,pitch=z['EULER'][:2]+x[:2]*z['XS'][:2];rates=z['RATES']+x[8:11]*z['XS'][8:11];vel=z['VELOCITY']+x[5:8]*z['XS'][5:8]
            return np.array([x[15],(rates[1]*np.sin(roll)+rates[2]*np.cos(roll))/np.cos(pitch)/np.deg2rad(10),np.linalg.norm(vel)/.5])
        H=np.column_stack([(output(x+step)-output(x-step))/2e-5 for step in np.eye(16)*1e-5])
        aa=np.eye(19);aa[:16,:16]=a;aa[16:,:16]=.02*H;bb=np.zeros((19,3));bb[:16]=b
        perfect=float(max(abs(np.linalg.eigvals(aa-bb@K))))
        # Unconstrained fixed observer + plant + integral dynamics. Reference
        # scheduling is frozen at the operating point for this local comparison.
        U=np.c_[np.zeros((3,16)),-K]
        plant=np.c_[a,np.zeros((16,19))]+b@U
        prediction=np.c_[np.zeros((16,16)),z['A'],np.zeros((16,3))]+z['B']@U
        observer=(np.eye(16)-L@C)@prediction+L@C@plant
        actualH=H.copy();actualH[0]=0;estimatedH=np.zeros_like(H);estimatedH[0]=H[0]
        integral=np.c_[.02*actualH,.02*estimatedH,np.eye(3)]
        coupled=float(max(abs(np.linalg.eigvals(np.r_[plant,observer,integral]))))
        records.append(dict(rate=r['rate'],speed=r['speed'],perfect_state_radius=perfect,fixed_observer_coupled_radius=coupled))
    (HERE/'local-poles.json').write_text(json.dumps(records,indent=2))
    print(json.dumps(dict(nodes=len(records),perfect_state_unstable=[r for r in records if r['perfect_state_radius']>=1],fixed_observer_unstable=[r for r in records if r['fixed_observer_coupled_radius']>=1]),indent=2))
if __name__=='__main__':main()
