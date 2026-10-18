import json
from pathlib import Path
from score import analyze_phase
from evaluate import compare
HERE=Path(__file__).resolve().parent
rows=analyze_phase(HERE/'validation')
chosen=json.loads((HERE/'selection-with-speed-task.json').read_text())['selected']
print(json.dumps({base:compare(rows,chosen,base,12) for base in ['published','local']},indent=2))
