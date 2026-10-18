"""Final dramatic verification after correcting the heading-wrap trigger."""
import json,sys,subprocess
from pathlib import Path
r=Path(__file__).parent
prior=json.loads((r/'styles.json').read_text());c=json.loads((r/'configs/dev08.json').read_text());c.pop('name')
prior['styles']['dramatic']=c;prior['selection']='Soft frozen after dev07; dramatic frozen after dev08 corrected heading-wrap check. Old dramatic-calm/gust excluded.'
(r/'styles.json').write_text(json.dumps(prior,indent=2))
for condition,variation in prior['conditions'].items():
 name=f'dramatic2-{condition}';p=r/'configs'/f'{name}.json';p.write_text(json.dumps(c|variation|dict(name=name),indent=2))
 subprocess.run([sys.executable,str(r/'run.py'),str(p)],check=True)
 subprocess.run([sys.executable,str(r/'analyze.py'),name],check=True)
selection={'replays':{'soft':'soft-calm','dramatic':'dramatic2-calm'},'validation':[f'{style}-{condition}' for style in ['soft','dramatic2'] for condition in prior['conditions']],
           'excluded_schedule_bug':['dev06','dramatic-calm','dramatic-gust'],'development_runs':8}
(r/'selection.json').write_text(json.dumps(selection,indent=2))
subprocess.run([sys.executable,str(r/'publish.py')],check=True)