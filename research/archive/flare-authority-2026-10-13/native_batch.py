import subprocess,sys
from pathlib import Path
p=Path(__file__).parent
for name in sys.argv[1:]:
 subprocess.run([sys.executable,str(p/'run.py'),str(p/'configs'/f'{name}.json')],check=True)
