"""Fixed-geometry damping calibration across independently reconstructed winds.

AI-assisted research. Does not overwrite the runtime model or prior studies.
"""
import os
os.environ['OPENBLAS_NUM_THREADS'] = '1'
from concurrent.futures import ProcessPoolExecutor
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time
import numpy as np
from scipy.spatial.transform import Rotation
from microagu_sim.compat import dynamics

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
WIND = HERE.parent / 'wind-smoother-2026-10-16'
spec = importlib.util.spec_from_file_location('prior_calibration', HERE.parent / 'as-built-simulator-2026-10-15/calibrate.py')
prior = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prior)
study = prior.study
VARIANTS = ['original', 'smoothed', 'slow_wind', 'fast_wind', 'beta_minus3', 'beta_plus3',
            'yaw_minus5', 'yaw_plus5', 'down_minus1', 'down_plus1', 'offset_tight', 'offset_loose']
GRID = [1., 1.5, 2., 2.5, 3.]
METRICS = [('roll_rate', 1.), ('pitch_rate', 1.), ('yaw_rate', 1.), ('bank', .25), ('climb', .25)]
CACHE = {}


def inputs(flight):
    if flight not in CACHE:
        day = study.FLIGHTS[flight][0]
        a = dict(np.load(study.DATA / f'{day}-causal-input.npz'))
        b = dict(np.load(study.DATA / f'{day}-propulsion-9x45.npz'))
        w = dict(np.load(WIND / f'{flight}-0.npz'))
        assert np.max(abs(a['t']-b['t'])) < 1e-6
        CACHE[flight] = a, b, w
    return CACHE[flight]


def replay(job):
    flight, start, variant, damping, seconds, alpha, legacy, substeps = job[:8]
    force_step = len(job) > 8 and job[8]
    a, b, w = inputs(flight)
    i = int(np.argmin(abs(a['t']-start)))
    sl = slice(i, i+round(seconds/.04)+1)
    t, truth = a['t'][sl], a['Y'][sl]
    if variant == 'original':
        wind = np.tile(a['wind'][i], (len(t), 1))
    else:
        assert t[0] >= w['t'][0] and t[-1] <= w['t'][-1], (flight, start)
        horizontal = w['alternatives'][VARIANTS.index(variant)-1]
        wind = np.column_stack([np.interp(t, w['t'], horizontal[:, j]) for j in range(2)] + [np.zeros(len(t))])
        if variant == 'down_minus1':
            wind[:, 2] = -1.
        elif variant == 'down_plus1':
            wind[:, 2] = 1.
    sim, _ = prior.migrated_model(flight, 'legacy' if legacy else 'routed',
                                  (damping, 1., 1. if legacy else .0743/.066), substeps=substeps)
    # Reuse the exact historical initialization, with the candidate initial wind.
    local = dict(a)
    local['wind'] = a['wind'].copy()
    local['wind'][i] = wind[0]
    y0 = study.init(sim, local, b, i, alpha)
    commands = np.column_stack([a['U'][sl, 0], a['brake_mm'][sl]])
    voltage, rpm = float(b['voltage'][i]), float(b['rpm'][i])
    row = dict(flight=flight, start=start, variant=variant, damping=damping,
               seconds=seconds, alpha=alpha, legacy=legacy, substeps=substeps,
               valid_fraction=float(a['valid'][sl].mean()))
    try:
        if variant == 'original' and not force_step:
            state, _ = sim.replay(y0, commands, voltage=voltage, rpm=rpm, wind=wind[0])
        else:
            # Reuse one Rust kernel; update wind each logged 40 ms interval.
            kernel = dynamics._kernel(sim.parameters, sim.geometry['mass'])
            u = np.array([sim.controls(*command) for command in commands])
            state = np.zeros((len(t), 26)); state[0] = y0
            for j in range(len(t)-1):
                step = kernel.replay_numpy(np.ascontiguousarray(state[j]), np.ascontiguousarray(u[j:j+2].ravel()),
                                           np.ascontiguousarray(wind[j]), voltage, rpm, .04, substeps, True)
                state[j+1], rpm = step[-1, :26], float(step[-1, 26])
        assert np.isfinite(state).all()
        rotation = Rotation.from_quat(state[:, 3:7])
        rates = rotation.inv().apply(state[:, 10:13])
        nav = state[:, 7:10]+np.cross(state[:, 10:13], rotation.apply(sim.geometry['imu']))
        rms = lambda x: float(np.sqrt(np.mean(x*x)))
        wrap = lambda x: np.arctan2(np.sin(x), np.cos(x))
        metrics = {k: rms(np.rad2deg(rates[:, j]-truth[:, j+3])) for j, k in enumerate(['roll_rate', 'pitch_rate', 'yaw_rate'])}
        metrics.update(bank=rms(np.rad2deg(wrap(rotation.as_euler('xyz')[:, 0]-truth[:, 0]))),
                       course=rms(np.rad2deg(wrap(np.arctan2(nav[:, 1], nav[:, 0])-np.arctan2(truth[:, 7], truth[:, 6])))),
                       climb=rms(nav[:, 2]-truth[:, 8]))
        return row | dict(finite=True, metrics=metrics)
    except (FloatingPointError, ValueError, AssertionError) as error:
        return row | dict(finite=False, error=str(error))


def batch(jobs, filename):
    path = HERE / filename
    if path.exists():
        rows = json.loads(path.read_text())
        assert len(rows) == len(jobs), 'Existing partial output; inspect before restarting'
        return rows
    rows = []
    with ProcessPoolExecutor(max_workers=8) as pool:
        for row in pool.map(replay, jobs):
            rows.append(row)
            if len(rows) % 40 == 0:
                print(filename, len(rows), '/', len(jobs), flush=True)
    path.write_text(json.dumps(rows, indent=2)+'\n')
    return rows


def main():
    fit_windows = [(f, t) for f, (_, starts) in study.FLIGHTS.items() for t in starts[::2]]
    check_windows = [(f, t) for f, (_, starts) in study.FLIGHTS.items() for t in starts[1::2]]
    refs = batch([(f, t, 'original', 1., 10, .35, True, 32) for f, t in fit_windows], 'references.json')
    assert all(r['finite'] for r in refs)
    ref = {(r['flight'], r['start']): r['metrics'] for r in refs}
    fits = batch([(f, t, variant, damping, 10, .35, False, 32)
                  for variant in VARIANTS for damping in GRID for f, t in fit_windows], 'fit.json')
    ranked = {}
    for variant in VARIANTS:
        ranking = []
        for damping in GRID:
            group = [r for r in fits if r['variant'] == variant and r['damping'] == damping]
            score = float(np.mean([sum(weight*r['metrics'][k]/ref[r['flight'], r['start']][k]
                                      for k, weight in METRICS)/3.5 for r in group])) if all(r['finite'] for r in group) else None
            ranking.append(dict(damping=damping, score=score))
        ranked[variant] = sorted(ranking, key=lambda r: r['score'] if r['score'] is not None else float('inf'))
    (HERE / 'ranked.json').write_text(json.dumps(ranked, indent=2)+'\n')
    jobs = [(f, t, variant, damping, 30, alpha, False, 32)
            for variant in VARIANTS for damping in sorted({3., ranked[variant][0]['damping']})
            for f, t in check_windows for alpha in [.25, .35, .45]]
    checks = batch(jobs, 'checks.json')
    print('selected', {k: v[0] for k, v in ranked.items()}, flush=True)
    print('finite', sum(r['finite'] for r in fits+checks), '/', len(fits+checks), flush=True)


def extended():
    fit_windows = [(f, t) for f, (_, starts) in study.FLIGHTS.items() for t in starts[::2]]
    check_windows = [(f, t) for f, (_, starts) in study.FLIGHTS.items() for t in starts[1::2]]
    refs = json.loads((HERE / 'references.json').read_text())
    ref = {(r['flight'], r['start']): r['metrics'] for r in refs}
    extra = batch([(f, t, variant, damping, 10, .35, False, 32)
                   for variant in VARIANTS for damping in [4., 6., 8.] for f, t in fit_windows], 'fit-extended.json')
    fits = json.loads((HERE / 'fit.json').read_text())+extra
    rankings = {}
    for variant in VARIANTS:
        rows = []
        for damping in GRID+[4., 6., 8.]:
            group = [r for r in fits if r['variant'] == variant and r['damping'] == damping]
            score = float(np.mean([sum(weight*r['metrics'][k]/ref[r['flight'], r['start']][k]
                                      for k, weight in METRICS)/3.5 for r in group])) if all(r['finite'] for r in group) else None
            rows.append(dict(damping=damping, score=score))
        rankings[variant] = sorted(rows, key=lambda r: r['score'] if r['score'] is not None else float('inf'))
    (HERE / 'ranked-extended.json').write_text(json.dumps(rankings, indent=2)+'\n')
    jobs = [(f, t, variant, rankings[variant][0]['damping'], 30, alpha, False, 32)
            for variant in VARIANTS if rankings[variant][0]['damping'] > 3
            for f, t in check_windows for alpha in [.25, .35, .45]]
    checks = batch(jobs, 'checks-extended.json')
    print('extended selections', {k: v[0] for k, v in rankings.items()}, flush=True)
    print('extended finite', sum(r['finite'] for r in extra+checks), '/', len(extra+checks), flush=True)


if __name__ == '__main__':
    extended() if '--extended' in sys.argv else main()
