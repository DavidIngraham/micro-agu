from pathlib import Path
import json,tarfile,hashlib
HERE=Path(__file__).resolve().parent
sha=lambda x:hashlib.sha256(x).hexdigest()
files=sorted((HERE/"runs").glob("*.json"))
with tarfile.open(HERE/"evidence.tar.gz","w:gz") as t:
    for p in files:t.add(p,arcname=str(p.relative_to(HERE)))
(HERE/"evidence-manifest.json").write_text(json.dumps(dict(
    sha256=sha((HERE/"evidence.tar.gz").read_bytes()),
    files={str(p.relative_to(HERE)):sha(p.read_bytes()) for p in files}),indent=2)+"\n")
