from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[3]
MODEL=Path('/home/david/repos/ardupilot-high-dof/Tools/simulation/paraglider')
SOURCE=Path('/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/longitudinal-2026-10-09/turn-model-spike')
RAW=Path('/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/spiral-authority-2026-10-13')
OUT=Path(__file__).parent
RAW.mkdir(exist_ok=True)
sys.path.insert(0,str(MODEL))
