"""Install a generated experimental gain set, or archive the active artifact."""
import json
import hashlib
from pathlib import Path
import shutil
import sys
from prepare import AP, HERE, VIEW, arrays_header

name = sys.argv[1]
design = HERE/name
if len(sys.argv) > 2 and sys.argv[2] == 'archive':
    destination = design/'runtime'
    destination.mkdir(exist_ok=True)
    for source in [VIEW/'generated/ardupilot/arduplane.js', VIEW/'generated/ardupilot/arduplane.wasm',
                   VIEW/'assets/runtime/build.json', VIEW/'assets/runtime/highdof.json',
                   AP/'libraries/AP_CustomControl/AP_CustomControl_LQI.cpp',
                   AP/'libraries/AP_CustomControl/AP_CustomControl_LQI_gains.h',
                   AP/'libraries/SITL/SIM_JSON.cpp', AP/'libraries/SITL/SIM_JSON.h']:
        shutil.copyfile(source, destination/source.name)
    hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in destination.iterdir() if p.is_file()}
    (destination/'hashes.json').write_text(json.dumps(hashes, indent=2)+'\n')
else:
    (AP/'libraries/AP_CustomControl/AP_CustomControl_LQI_gains.h').write_text(
        (design/'AP_CustomControl_LQI_gains.h').read_text()+arrays_header(design))
