"""Include unchanged filtered gains in training selection before any held-out result."""
import time
from pathlib import Path
from evaluate import select
HERE=Path(__file__).resolve().parent
while not (HERE/'selection.json').exists():time.sleep(1)
assert not any(p.stem.isdigit() for p in (HERE/'validation').glob('*/*.json')),'Validation already observed'
select('selection-with-filtered.json')
