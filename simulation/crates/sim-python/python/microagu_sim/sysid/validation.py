"""Free-running reserved-episode validation; never estimates evaluation states."""

from concurrent.futures import ProcessPoolExecutor

import numpy as np

from .data import validate_split
from .metrics import metrics, residual_diagnostics, response_residual, summarize, timing
from .model import DEFAULT, replay


def evaluate(job):
    segment, theta, alpha, family = job
    predicted, _, _ = replay(segment, theta, alpha=alpha)
    observed = np.rad2deg(segment.observation[:, 3:6])
    r = response_residual(observed, predicted)
    return dict(
        name=segment.name,
        episode=segment.episode,
        start=float(segment.time[0]),
        alpha=alpha,
        family=family,
        **metrics(observed, predicted),
        uncentered_response_loss=float(r @ r),
        command_response=timing(observed, predicted, segment.commands),
        diagnostics=residual_diagnostics(observed, predicted, segment.commands),
    )


def validate(
    segments, theta, workers=4, alphas=(0.25, 0.35, 0.45), training_episodes=()
):
    validate_split(segments)
    reserved = [s for s in segments if s.role == "validation"]
    if not reserved:
        raise ValueError("No reserved validation episodes")
    if set(training_episodes) & {s.episode for s in reserved}:
        raise ValueError("Frozen fit trained on a requested validation episode")
    if not alphas or not np.isfinite(alphas).all():
        raise ValueError("Finite initialization angles required")
    rows = []
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for family, parameters in [("baseline", DEFAULT), ("candidate", theta)]:
            rows.extend(
                pool.map(
                    evaluate,
                    [(s, parameters, a, family) for s in reserved for a in alphas],
                )
            )
    by_episode = {}
    for episode in sorted({s.episode for s in reserved}):
        base = [
            r for r in rows if r["family"] == "baseline" and r["episode"] == episode
        ]
        new = [
            r for r in rows if r["family"] == "candidate" and r["episode"] == episode
        ]
        by_episode[episode] = dict(
            summarize(base, new),
            uncentered_response_ratio=float(
                np.mean(
                    [
                        n["uncentered_response_loss"] / b["uncentered_response_loss"]
                        for b, n in zip(base, new)
                    ]
                )
            ),
        )
    return {
        "schema_version": 1,
        "rows": rows,
        "by_episode": by_episode,
        "production_changed": False,
        "promotion": "Not evaluated; review separate physical metrics and uncertainty",
    }
