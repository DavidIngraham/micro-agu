from concurrent.futures import ProcessPoolExecutor
from experiment import scenarios,run_case,save

def task(args):return run_case(*args)

if __name__=='__main__':
    jobs=[(c,n+'_observer') for c in scenarios() if c['name'] in ['maneuver','gust','voltage_ramp','soft_roll_gust']
          for n in ['baseline','candidate','scheduled']]
    with ProcessPoolExecutor(max_workers=4) as pool:rows=list(pool.map(task,jobs))
    save('observer-results.json',rows)
