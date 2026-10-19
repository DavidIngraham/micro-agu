import hashlib,json,tarfile
from pathlib import Path
P=Path(__file__).resolve().parent
files=sorted((P/'runs').glob('*.json'))+[P/n for n in ['calibration-basis.json','calibration-verified.json','fixed-mode.json','fixed-boundary.json','numerics.json','smoke.json']]
archive=P/'evidence.tar.gz'
with tarfile.open(archive,'w:gz',compresslevel=6) as tar:
 for f in files:tar.add(f,arcname=str(f.relative_to(P)))
manifest=dict(archive=archive.name,sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),files={str(f.relative_to(P)):hashlib.sha256(f.read_bytes()).hexdigest() for f in files})
(P/'evidence-manifest.json').write_text(json.dumps(manifest,indent=2))
print('Archived',len(files),'files;',archive.stat().st_size,'bytes')
