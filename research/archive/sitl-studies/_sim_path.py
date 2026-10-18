"""Locate the reusable simulator for study entry points, from any working directory."""
from pathlib import Path
import os
SIMULATION=Path(os.environ.get('MICROAGU_SIM_ROOT',Path(__file__).resolve().parents[3]/'simulation')).resolve()
