"""Expose the exact built header next to each synthesized design archive."""
from pathlib import Path
for runtime in (Path(__file__).resolve().parent/'designs').glob('*/runtime'):
    (runtime.parent/'AP_CustomControl_LQI_gains.h').write_bytes((runtime/'AP_CustomControl_LQI_gains.h').read_bytes())
