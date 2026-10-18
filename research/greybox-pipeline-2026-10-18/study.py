"""Frozen comparison: legacy versus direct response, with/without nuisance fit."""

import os

os.environ["OPENBLAS_NUM_THREADS"] = "1"
import hashlib
import json
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
from pathlib import Path

import numpy as np
from microagu_sim.sysid.data import load_session
from microagu_sim.sysid.fit import FitOptions, fit
from microagu_sim.sysid.metrics import (
    metrics,
    residual_diagnostics,
    response_residual,
    summarize,
    timing,
)
from microagu_sim.sysid.model import DEFAULT, replay

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
DATA = Path(
    os.environ.get(
        "MICROAGU_REPLAY_DATA",
        "/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/longitudinal-2026-10-09/turn-model-spike",
    )
)


def save(name, value):
    (OUT / name).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def build_session():
    specifications = [
        ("train", "manual-266", [280, 310]),
        ("train", "manual-529", [535, 555]),
        ("train", "manual-1985", [1988]),
        ("validation", "manual-371", [380, 400]),
        ("validation", "manual-435", [440]),
        ("validation", "nonmanual-1440", [1440, 1470, 1500]),
        ("validation", "nonmanual-2130", [2130, 2160, 2190]),
        ("validation", "nonmanual-2760", [2760, 2790, 2820]),
    ]
    segments = [
        {
            "name": f"tl-{start}",
            "episode": episode,
            "role": role,
            "flight": "trout_lake",
            "start_s": start,
            "duration_s": 20,
            "causal_input": str(DATA / "2026-02-15-causal-input.npz"),
            "propulsion_input": str(DATA / "2026-02-15-propulsion-9x45.npz"),
        }
        for role, episode, starts in specifications
        for start in starts
    ]
    cfg = {"schema_version": 1, "segments": segments}
    save("session.json", cfg)
    return cfg


def evaluate(job):
    s, theta, alpha, family = job
    pred, _y, _ = replay(s, theta, alpha=alpha)
    obs = np.rad2deg(s.observation[:, 3:6])
    row = dict(
        name=s.name,
        episode=s.episode,
        start=float(s.time[0]),
        alpha=alpha,
        family=family,
        **metrics(obs, pred),
        command_response=timing(obs, pred, s.commands),
        uncentered_response_loss=float(
            response_residual(obs, pred) @ response_residual(obs, pred)
        ),
        diagnostics=residual_diagnostics(obs, pred, s.commands),
    )
    if alpha == 0.35:
        np.savez_compressed(
            OUT / f"{family}-{s.name}.npz",
            time=s.time,
            observed=obs,
            predicted=pred,
            commands=s.commands,
        )
    return row


def main():
    build_session()
    _, segments, hashes = load_session(OUT / "session.json")
    # Establish that this reusable adapter reproduces the prior research replay.
    old = json.loads(
        (
            ROOT / "research/manual-coupling-fit-2026-10-18/baseline-training.json"
        ).read_text()
    )
    reproduction = []
    for s in [s for s in segments if s.role == "train"]:
        r = evaluate((s, DEFAULT, 0.35, "baseline-training"))
        reference = next(v for v in old if abs(v["start"] - s.time[0]) < 0.021)
        delta = abs(reference["score"] - r["score"])
        assert delta < 1e-10, (s.name, delta)
        reproduction.append({"name": s.name, "score_delta": delta})
    save("baseline-reproduction.json", reproduction)
    save(
        "protocol.json",
        {
            "variants": ["legacy", "response", "response_nuisance"],
            "max_replays_per_start": 2000,
            "training_episodes": sorted(
                {s.episode for s in segments if s.role == "train"}
            ),
            "selection": "Training objective only; all fits frozen before validation; no validation nuisance fitting",
            "evidence": "Reused historical flight and previously studied evaluation windows; not independent prospective validation",
            "improvement_criteria": "No scalar promotion gate: report uncentered rate/acceleration, centered local, spectral amplitudes, inter-axis and command phase independently. A different objective alone cannot establish overall model fidelity.",
            "nuisance": "Per training segment only: canopy initial angle +/-0.1 rad and constant north/east wind corrections +/-2 m/s, penalized by stated priors. No arbitrary latent forces.",
            "production_changed": False,
        },
    )
    save("input-hashes.json", hashes)
    results = {}
    for variant in ["legacy", "response", "response_nuisance"]:
        options = FitOptions(
            objective="legacy" if variant == "legacy" else "response",
            fit_nuisance=variant == "response_nuisance",
            max_replays=2000,
        )
        print("Fitting", variant, flush=True)
        result = fit(segments, options)
        results[variant] = result
        save(f"fit-{variant}.json", dict(options=asdict(options), **result))
        print("Selected", variant, result["selection"], flush=True)
    save("frozen-selection.json", {k: v["selection"] for k, v in results.items()})
    rows = []
    with ProcessPoolExecutor(max_workers=4) as pool:
        for family, theta in [
            ("baseline", DEFAULT),
            *[(k, v["selection"]["theta"]) for k, v in results.items()],
        ]:
            jobs = [
                (s, theta, alpha, family)
                for s in segments
                if s.role == "validation"
                for alpha in [0.25, 0.35, 0.45]
            ]
            rows.extend(pool.map(evaluate, jobs))
            print("Validated", family, flush=True)
    save("validation.json", rows)
    summary = {}
    for variant in results:
        summary[variant] = {}
        for kind in ["manual", "nonmanual"]:
            base = [
                r
                for r in rows
                if r["family"] == "baseline" and r["episode"].startswith(kind + "-")
            ]
            new = [
                r
                for r in rows
                if r["family"] == variant and r["episode"].startswith(kind + "-")
            ]
            summary[variant][kind] = dict(
                summarize(base, new),
                uncentered_response_ratio=float(
                    np.mean(
                        [
                            n["uncentered_response_loss"]
                            / b["uncentered_response_loss"]
                            for n, b in zip(new, base)
                        ]
                    )
                ),
            )
    save("summary.json", summary)
    save(
        "source-hashes.json",
        {
            str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in [
                Path(__file__),
                *list(
                    (
                        ROOT / "simulation/crates/sim-python/python/microagu_sim/sysid"
                    ).glob("*.py")
                ),
            ]
        },
    )
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == "__main__":
    main()
