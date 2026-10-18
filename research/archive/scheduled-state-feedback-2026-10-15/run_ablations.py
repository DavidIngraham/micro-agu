from concurrent.futures import ProcessPoolExecutor
from experiment import scenarios,run_case,save

def task(args):return run_case(*args)

if __name__=='__main__':
    jobs=[(c,n) for c in scenarios() if c['name'] in ['voltage_ramp','speed_transition']
          for n in ['scheduled_no_voltage','scheduled_no_speed']]
    with ProcessPoolExecutor(max_workers=4) as pool:rows=list(pool.map(task,jobs))
    save('ablation-results.json',rows)
