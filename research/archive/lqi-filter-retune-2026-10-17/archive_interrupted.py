"""Preserve the interrupted earth-fixed protocol; never reuse those runs."""
import hashlib,json
from pathlib import Path
HERE=Path(__file__).resolve().parent
assert not json.loads((HERE/'restoration.json').read_text())['conflicts']
for r in json.loads((HERE/'original/manifest.json').read_text()):
    assert hashlib.sha256(Path(r['path']).read_bytes()).hexdigest()==r['sha256']
dest=HERE/'interrupted-earth-fixed';dest.mkdir()
for name in ['original','training','protocol.json','restoration.json','campaign.log']:
    (HERE/name).rename(dest/name)
(dest/'README.txt').write_text('Interrupted before gain selection when the earth-fixed gust direction confound was identified. All 25 snapshotted files restored. These runs are excluded from retuning and validation.\n')
