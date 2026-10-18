"""Build the current Rust wheel; historical candidate packages stay immutable."""
import argparse
from pathlib import Path
import subprocess

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'simulation/Cargo.toml').exists())
DEFAULT_OUTPUT = ROOT/'simulation/target/research-handoff'


def build(output=DEFAULT_OUTPUT):
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    subprocess.run(['uv', 'run', '--project', str(ROOT/'simulation'), 'maturin',
                    'build', '--release', '--locked', '-m',
                    str(ROOT/'simulation/crates/sim-python/Cargo.toml'),
                    '--out', str(output)], check=True)
    return max(output.glob('microagu_sim-*.whl'), key=lambda p: p.stat().st_mtime)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    print(build(parser.parse_args().output))
