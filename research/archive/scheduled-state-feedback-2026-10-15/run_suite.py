"""Independent numerical simulations, not separate authoring agents."""
from concurrent.futures import ProcessPoolExecutor
from experiment import scenarios, run_case, save

def task(args):
    return run_case(*args)

if __name__=='__main__':
    jobs=[(case,name) for case in scenarios() for name in ['baseline','candidate','scheduled']]
    with ProcessPoolExecutor(max_workers=4) as pool:
        rows=list(pool.map(task,jobs))
    save('results.json',rows)
