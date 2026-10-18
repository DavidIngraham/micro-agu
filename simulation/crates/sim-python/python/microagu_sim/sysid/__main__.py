"""Command line entry point: python -m microagu_sim.sysid fit SESSION OUTPUT."""

import argparse
import hashlib
import json
from dataclasses import asdict
from pathlib import Path

import numpy as np

from .data import load_session
from .fit import FitOptions, fit


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=["fit", "validate"])
    parser.add_argument("session", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--fit-result", type=Path, help="Frozen fit.json for validate")
    args = parser.parse_args()
    if args.action == "validate" and args.fit_result is None:
        parser.error("validate requires --fit-result")
    if args.output.exists():
        parser.error("Output directory exists; choose a new run directory")
    cfg, segments, hashes = load_session(args.session)
    options = FitOptions(**cfg.get("fit", {}))
    args.output.mkdir(parents=True)
    manifest = {
        "session": cfg,
        "input_hashes": hashes,
        "options": asdict(options),
        "package_sources": {
            p.name: hashlib.sha256(p.read_bytes()).hexdigest()
            for p in Path(__file__).parent.glob("*.py")
        },
    }
    from importlib.metadata import version

    from . import model

    package_root = Path(__file__).resolve().parent.parent
    manifest["runtime_versions"] = {
        name: version(name) for name in ["numpy", "scipy", "microagu-sim"]
    }
    manifest["physics_sources"] = {
        str(p.relative_to(package_root)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in package_root.rglob("*")
        if p.is_file()
        and "__pycache__" not in p.parts
        and (
            p.suffix in {".so", ".json", ".npz"}
            or (p.parent.name == "compat" and p.suffix == ".py")
        )
    }
    manifest["aircraft_profiles"] = {
        flight: {
            "parameters": sim.parameters,
            "geometry": {k: np.asarray(v).tolist() for k, v in sim.geometry.items()},
        }
        for flight in sorted({s.flight for s in segments})
        for sim in [model.make_simulator(flight, profile="as_built")]
    }
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    if args.action == "validate":
        from .validation import validate

        fitted = json.loads(args.fit_result.read_text())
        result = validate(
            segments,
            fitted["selection"]["theta"],
            workers=options.workers,
            training_episodes=fitted["training_episodes"],
        )
        manifest["fit_result_sha256"] = hashlib.sha256(
            args.fit_result.read_bytes()
        ).hexdigest()
        (args.output / "manifest.json").write_text(
            json.dumps(manifest, indent=2) + "\n"
        )
        (args.output / "validation.json").write_text(
            json.dumps(result, indent=2, allow_nan=False) + "\n"
        )
        print(json.dumps(result["by_episode"], indent=2))
        return
    result = fit(segments, options, cfg.get("starts"))
    (args.output / "fit.json").write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n"
    )
    print(
        json.dumps({k: result[k] for k in ["selection", "training_episodes"]}, indent=2)
    )


if __name__ == "__main__":
    main()
