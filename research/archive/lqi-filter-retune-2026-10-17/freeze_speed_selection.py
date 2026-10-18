"""Freeze corrected speed-task selection on training before any hold-out result."""
import time,json
from pathlib import Path
from evaluate import select
HERE=Path(__file__).resolve().parent
initial=(HERE/'selection.json').stat().st_mtime_ns
while (HERE/'selection.json').stat().st_mtime_ns==initial:time.sleep(.5)
assert not any(p.stem.isdigit() for p in (HERE/'validation').glob('*/*.json')),'Validation already observed'
(HERE/'speed-task-gate.json').write_text(json.dumps(dict(
    correction='Do not reject an increase toward the 4.4 m/s target merely because baseline was slow. The former symmetric speed-ratio check did this on training case 13.',
    gate='No >10% mean true-speed drop versus baseline in any climb/descent; no mean true-speed above 4.84 m/s (10% over fixed maximum target).',
    timing='Training only; no held-out result exists.'),indent=2))
select('selection-with-speed-task.json')
