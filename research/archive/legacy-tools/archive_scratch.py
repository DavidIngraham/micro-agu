#!/usr/bin/env python3
"""Create a lossless, hashed snapshot; compress raw logs and large artifacts."""
import concurrent.futures
import gzip
import hashlib
import json
import shutil
from collections import Counter
from pathlib import Path

SOURCE = Path('/workspace/scratch')
DESTINATION = Path(__file__).resolve().parents[1]
EXCLUDED_PARTS = {'__pycache__', '.pytest_cache', '.mypy_cache'}

def archive(path):
    relative = path.relative_to(SOURCE)
    if any(part in EXCLUDED_PARTS for part in relative.parts) or path.suffix == '.pyc':
        return {'excluded': str(relative), 'reason': 'Python runtime cache'}
    size = path.stat().st_size
    with path.open('rb') as stream:
        elf = stream.read(4) == b'\x7fELF'
    compressed = path.suffix.lower() in {'.bin', '.tlog'} or elf or (size > 5*1024**2 and path.suffix not in {'.zip', '.gz', '.png', '.pdf'})
    stored = Path('scratch') / relative
    if compressed:
        stored = Path(str(stored)+'.gz')
    destination = DESTINATION / stored
    destination.parent.mkdir(parents=True, exist_ok=True)
    digest = hashlib.sha256()
    if compressed:
        with path.open('rb') as source, destination.open('wb') as target:
            with gzip.GzipFile(filename='', mode='wb', fileobj=target, compresslevel=6, mtime=0) as encoded:
                while chunk := source.read(1024**2):
                    digest.update(chunk)
                    encoded.write(chunk)
    else:
        shutil.copy2(path, destination)
        with path.open('rb') as stream:
            while chunk := stream.read(1024**2):
                digest.update(chunk)
    return dict(original_path='scratch/'+str(relative), stored_path=str(stored), original_bytes=size,
                stored_bytes=destination.stat().st_size, sha256=digest.hexdigest(), compression='gzip' if compressed else None,
                original_mode=path.stat().st_mode & 0o777)

if __name__ == '__main__':
    paths=sorted(p for p in SOURCE.rglob('*') if p.is_file())
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        entries=list(pool.map(archive,paths))
    files=[e for e in entries if 'excluded' not in e]
    report=dict(source_root=str(SOURCE),snapshot_date='2026-10-09',files=files,
                excluded=[e for e in entries if 'excluded' in e],file_count=len(files),
                original_bytes=sum(e['original_bytes'] for e in files),stored_bytes=sum(e['stored_bytes'] for e in files))
    (DESTINATION/'provenance/artifact-manifest.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:report[k] for k in ['file_count','original_bytes','stored_bytes']},indent=2),flush=True)
