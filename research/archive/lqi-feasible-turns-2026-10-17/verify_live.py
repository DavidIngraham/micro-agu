"""Verify ordinary public URLs against the exact prepared release manifest."""
import hashlib
import json
import os
from pathlib import Path
from urllib.request import urlopen
from concurrent.futures import ThreadPoolExecutor

HERE=Path(__file__).resolve().parent
BASE='https://davidingraham.github.io/assets/microagu-viewer/'
manifest=json.loads((HERE/os.environ.get('RELEASE_OUTPUT','release')/'applet-manifest.json').read_text())['applets']['simulator']
def check(item):
    name,expected=item
    with urlopen(BASE+name,timeout=45) as response:data=response.read()
    actual=hashlib.sha256(data).hexdigest()
    return dict(path=name,expected=expected,actual=actual,matched=actual==expected)
with ThreadPoolExecutor(max_workers=4) as pool:checks=list(pool.map(check,manifest['files'].items()))
result=dict(url=BASE+'simulator.html',passed=all(r['matched'] for r in checks),checks=checks)
(HERE/os.environ.get('HASH_OUTPUT','live-hashes.json')).write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(dict(passed=result['passed'],files=len(checks),mismatches=[r['path'] for r in checks if not r['matched']])))
assert result['passed'],'Public assets do not yet match the prepared release'
