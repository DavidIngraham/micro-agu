"""Finish the frozen study and stage a release only after all gates pass."""
import json,subprocess,sys,time
from pathlib import Path
HERE=Path(__file__).resolve().parent
while not (HERE/'stage2-restoration.json').exists():time.sleep(1)
assert not json.loads((HERE/'stage2-restoration.json').read_text())['conflicts']
def run(name):
    print('START '+name,flush=True)
    subprocess.run([sys.executable,str(HERE/name)],check=True)
    print('DONE '+name,flush=True)
run('complete_selection.py')
chosen=json.loads((HERE/'selection.json').read_text())['selected']
if chosen:run('endurance.py')
run('results.py')
if chosen:run('audit.py')
if json.loads((HERE/'acceptance.json').read_text())['accepted']:run('prepare_release.py')
print('STUDY COMPLETE',flush=True)
