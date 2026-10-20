from pathlib import Path
import hashlib,json,tarfile
HERE=Path(__file__).resolve().parent
sha=lambda b:hashlib.sha256(b).hexdigest()
files=sorted((HERE/"runs").glob("*.json"))+sorted((HERE/"preliminary-runs").glob("*.json"))
with tarfile.open(HERE/"evidence.tar.gz","w:gz") as out:
    for p in files:out.add(p,arcname=str(p.relative_to(HERE)))
manifest=dict(sha256=sha((HERE/"evidence.tar.gz").read_bytes()),
              files={str(p.relative_to(HERE)):sha(p.read_bytes()) for p in files})
(HERE/"evidence-manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
print(len(files),"traces archived")
