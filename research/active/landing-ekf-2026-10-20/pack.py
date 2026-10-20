from pathlib import Path
import json,tarfile,hashlib
HERE=Path(__file__).resolve().parent
sha=lambda x:hashlib.sha256(x).hexdigest()
files=sorted((HERE/"runs").glob("*.json"))
with tarfile.open(HERE/"evidence.tar.gz","w:gz") as t:
    for p in files:t.add(p,arcname=str(p.relative_to(HERE)))
    diagnostic=HERE/"preliminary-runs/head-ground.json"
    if diagnostic.exists():t.add(diagnostic,arcname="diagnostics/headwind-ground-roll.json")
(HERE/"evidence-manifest.json").write_text(json.dumps(dict(
    sha256=sha((HERE/"evidence.tar.gz").read_bytes()),
    files={str(p.relative_to(HERE)):sha(p.read_bytes()) for p in files},
    diagnostics={"diagnostics/headwind-ground-roll.json":sha(diagnostic.read_bytes())} if diagnostic.exists() else {}),indent=2)+"\n")
