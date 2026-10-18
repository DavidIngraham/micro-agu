"""Freeze and run paired 35-deg/s disturbance qualification before release."""
import json,subprocess
from concurrent.futures import ThreadPoolExecutor
import numpy as np
from run import HERE,VIEW,ENV
from build import sha
from reassess import metrics
from score import assess
D=HERE/'deployment35'
def main():
    rng=np.random.default_rng(2026101901);cases=[]
    for i in range(6):
        cases.append(dict(id=200+i,sign=1 if i%2==0 else -1,scales={k:float(rng.uniform(.7,1.3)) for k in ['Cmq','joint_c_pitch','joint_c_roll','joint_c_yaw','CLbrake','CDbrake']},brakeRateScale=float(rng.uniform(.7,1.1)),wind=float(rng.uniform(0,2)),turbulence=float(rng.uniform(.1,.6)),seed=int(rng.integers(1,2**32-1)),weatherOffRecovery=True))
    params={'CP3_AWMODE':1,'CP3_TMAP':1,'CP3_OBSTRIM':1}
    plan=dict(cases=cases,firmware=sha(D/'runtime/arduplane.wasm'),
      criteria={'all_completed':True,'quiet_recovery_heading_rms':3,'quiet_recovery_pitch_rms':3,'quiet_recovery_altitude_rms':1.5,'minimum_speed':2,
        'vs_classic35_median_tracking_rms':'<=1.2*classic+0.5 deg/s','vs_classic35_median_pitch_rms':'<=1.2*classic+0.5 deg/s','vs_classic35_median_alt_rms':'<=1.2*classic+0.3 m',
        'vs_lqi25_median_pitch_rms':'<=1.25*lqi25+0.5 deg/s','vs_lqi25_individual_pitch_peak':'<=1.5*lqi25+5 deg/s','vs_lqi25_median_shaft_power':'<=1.2*lqi25'},
      interpretation='Browser regression qualification, not physical safety certification. All original turbulent screens are retained separately, not used as binary release gate. Paired seeds/coefficients; trajectories differ. Rate35 vs rate25 is a workload comparison, not matched tracking demand. No tune changes permitted on this validation set.')
    pp=D/'disturbance-protocol.json'
    if pp.exists():assert json.loads(pp.read_text())==plan
    else:pp.write_text(json.dumps(plan,indent=2))
    jobs=[dict(s,variant=name,rate=rate,scheme=scheme,parameters=dict(params,CP3_RMAX=rate,PG_TURN_RMAX=rate),runtime=str(D/'runtime')) for s in cases for name,rate,scheme in [('lqi35',35,'lqi'),('classic35',35,'baseline'),('lqi25',25,'lqi')]]
    def run(s):
        folder=D/'disturbance'/s['variant'];folder.mkdir(parents=True,exist_ok=True);p=folder/f"{s['id']:03}.json";sp=p.with_name(p.stem+'-spec.json')
        if not p.exists():
            sp.write_text(json.dumps(s,indent=2))
            with p.with_suffix('.log').open('w') as log:subprocess.run([str(VIEW/'node_modules/.bin/tsx'),'--tsconfig',str(VIEW/'tsconfig.json'),str(HERE/'case.mts'),str(sp),str(p)],cwd=VIEW,env=ENV,stdout=log,stderr=subprocess.STDOUT,timeout=400)
        raw=json.loads(p.read_text());r=dict(case=s['id'],variant=s['variant'],complete=raw['complete'],failure=raw['failure'])
        if raw['complete']:r.update(metrics(p));r['original_screens']=assess(p)
        print(json.dumps({k:v for k,v in r.items() if k!='original_screens'}),flush=True);return r
    with ThreadPoolExecutor(max_workers=3) as pool:rows=list(pool.map(run,jobs))
    checks={'all_completed':all(r['complete'] for r in rows)}
    if checks['all_completed']:
        group={name:[r for r in rows if r['variant']==name] for name in ['lqi35','classic35','lqi25']}
        def med(name,key):return float(np.median([r[key] for r in group[name]]))
        a=group['lqi35'];checks.update(quiet_recovery=all(r['quiet_heading_rms']<=3 and r['quiet_pitch_rms']<=3 and r['quiet_alt_rms']<=1.5 for r in a),minimum_speed=all(r['min_speed']>=2 for r in a))
        for key,margin in [('tracking_rms',.5),('pitch_rms',.5),('alt_rms',.3)]:checks['classic35_'+key]=med('lqi35',key)<=1.2*med('classic35',key)+margin
        checks['lqi25_pitch_rms']=med('lqi35','pitch_rms')<=1.25*med('lqi25','pitch_rms')+.5
        checks['lqi25_peak']=all(r['pitch_peak']<=1.5*next(b['pitch_peak'] for b in group['lqi25'] if b['case']==r['case'])+5 for r in a)
        checks['lqi25_power']=med('lqi35','shaft_W')<=1.2*med('lqi25','shaft_W')
    result=dict(accepted=all(checks.values()),checks=checks,rows=rows,protocol=plan)
    (D/'disturbance.json').write_text(json.dumps(result,indent=2));print(json.dumps(dict(accepted=result['accepted'],checks=checks)),flush=True)
if __name__=='__main__':main()
