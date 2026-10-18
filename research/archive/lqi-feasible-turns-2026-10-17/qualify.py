"""Predeclared browser qualification of the final non-instrumented firmware."""
import json,subprocess,os
from concurrent.futures import ThreadPoolExecutor
import numpy as np
from scipy.spatial.transform import Rotation
from run import HERE,VIEW,ENV,specs
from build import sha

def main():
    limit=int(os.environ.get('RELEASE_RATE','25'))
    out=HERE/os.environ.get('RELEASE_DIR','deployment');params={'CP3_RMAX':limit,'PG_TURN_RMAX':limit,'CP3_AWMODE':1,'CP3_TMAP':1,'CP3_OBSTRIM':1}
    runtime=out/'runtime';fresh=os.environ.get('QUALIFY_FRESH')=='1'
    if fresh:out=out/'fresh';out.mkdir(exist_ok=True)
    jobs=[]
    cases=[specs(True)[1],specs(True)[4]] if fresh else [specs()[0],specs()[1],specs(True)[0],specs(True)[6]]
    for i,s in enumerate(cases):
        for scheme in ['baseline','lqi']:
            jobs.append(dict(s,id=i,rate=limit,scheme=scheme,parameters=params,runtime=str(runtime),transitions=True,kind='transition'))
    for rate in ([] if fresh else sorted({10,25,limit})):
        for s in specs()[:2]:jobs.append(dict(s,rate=rate,scheme='lqi',parameters=dict(params,CP3_RMAX=rate,PG_TURN_RMAX=rate),runtime=str(runtime),kind='turn'))
    protocol=dict(firmware_sha256=sha(runtime/'arduplane.wasm'),jobs=jobs,
        criteria='All flights complete, finite, altitude above 15 m, roll below 60 and pitch below 45 deg, correct ownership within 5 s. These are simulation regression guards, not physical safety certification. Final calm turns must pass the original motion/tracking/recovery gates excluding only unavailable research diagnostics. Transition candidate peak relative pitch rate and altitude excursion must not exceed matched baseline by more than 20% plus 2 deg/s or 1 m respectively. No gain retuning on these cases. Investigate failures; never silently relax this protocol.')
    pp=out/'qualification-protocol.json'
    if pp.exists():assert json.loads(pp.read_text())==protocol
    else:pp.write_text(json.dumps(protocol,indent=2))
    def run(s):
        folder=out/'qualification'/f"{s['kind']}-{s['scheme']}-{s['rate']}";folder.mkdir(parents=True,exist_ok=True)
        p=folder/f"{s['id']:03}.json";sp=p.with_name(p.stem+'-spec.json');sp.write_text(json.dumps(s,indent=2))
        if not p.exists():
            with p.with_suffix('.log').open('w') as log:subprocess.run([str(VIEW/'node_modules/.bin/tsx'),'--tsconfig',str(VIEW/'tsconfig.json'),str(HERE/'case.mts'),str(sp),str(p)],cwd=VIEW,env=ENV,stdout=log,stderr=subprocess.STDOUT,timeout=400)
        d=json.loads(p.read_text());r=dict(kind=s['kind'],scheme=s['scheme'],case=s['id'],rate=s['rate'],complete=d['complete'],failure=d['failure'],source=str(p.relative_to(HERE)))
        if d['complete']:
            x=np.array([a['truth'] for a in d['samples']]);rot=Rotation.from_quat(x[:,3:7]);relative=np.rad2deg(rot.inv().apply(x[:,23:26]-x[:,10:13])[:,1])
            r.update(pitch_peak=float(np.max(abs(relative))),alt_peak=float(np.max(abs(-x[:,2]-60))))
            if s['kind']=='turn':
                from score import assess
                m=assess(p);r['original_reasons']=m['reasons'];r['release_reasons']=[v for v in m['reasons'] if v!='Missing controller diagnostics'];r['passed']=not r['release_reasons']
        print(json.dumps(r),flush=True);return r
    with ThreadPoolExecutor(max_workers=3) as pool:rows=list(pool.map(run,jobs))
    for r in rows:
        if not r['complete']:r['passed']=False
        elif r['kind']=='transition':
            b=next(x for x in rows if x['kind']=='transition' and x['case']==r['case'] and x['scheme']=='baseline')
            r['passed']=b['complete'] and r['pitch_peak']<=1.2*b['pitch_peak']+2 and r['alt_peak']<=1.2*b['alt_peak']+1
    result=dict(accepted=all(r['passed'] for r in rows),rows=rows,firmware_sha256=protocol['firmware_sha256'])
    (out/'qualification.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
if __name__=='__main__':main()
