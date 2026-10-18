"""Record embedded build identity separately from the current checkout revision."""
import argparse
import hashlib
import json
import subprocess
from pathlib import Path
from pymavlink import mavutil
from study import HERE,save

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--repo',type=Path,required=True);a=p.parse_args()
    rows=[]
    for run in json.loads((HERE/'native-results.json').read_text()):
        log=mavutil.mavlink_connection(str(Path(run['raw_directory'])/'flight.BIN'))
        ver=log.recv_match(type='VER');log.close()
        rows.append(dict(controller=run['controller'],condition=run['condition'],version=ver.to_dict() if ver else None))
    embedded=f"{rows[0]['version']['GH']:08x}"
    diff=subprocess.check_output(['git','diff','--stat',embedded,'HEAD','--','ArduPlane','libraries/AP_TECS'],cwd=a.repo,text=True)
    result=dict(runs=rows,checkout_revision=subprocess.check_output(['git','rev-parse','HEAD'],cwd=a.repo,text=True).strip(),
         binary_sha256=hashlib.sha256((a.repo/'build/sitl/bin/arduplane').read_bytes()).hexdigest(),
         source_difference_from_embedded_revision=diff,
         note='Executable build tree is not inferred from the checkout hash. The BIN version records, settings and binary hash identify the tested executable. The article documents the source-difference assessment for this study.')
    save('native-build-audit.json',result)
    save('native-provenance.json',{k:result[k] for k in ['checkout_revision','binary_sha256','note']})
