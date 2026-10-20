from pathlib import Path
import hashlib
import json
import tarfile

HERE = Path(__file__).resolve().parent
sha = lambda data: hashlib.sha256(data).hexdigest()
files = sorted((HERE/"native-runs").glob("*.json")) + sorted((HERE/"ap-runs").glob("*.json"))
archive = HERE/"evidence.tar.gz"
with tarfile.open(archive, "w:gz", compresslevel=6) as target:
    for p in files:
        target.add(p, arcname=str(p.relative_to(HERE)))
manifest = dict(archive=archive.name, sha256=sha(archive.read_bytes()),
                files={str(p.relative_to(HERE)): sha(p.read_bytes()) for p in files},
                source_sha256={p.name: sha(p.read_bytes()) for pattern in ("*.py", "*.mts")
                               for p in HERE.glob(pattern)})
(HERE/"evidence-manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
print(len(files), "traces;", archive.stat().st_size, "bytes")
