import json,hashlib
from pathlib import Path
from pymavlink import DFReader
BASE=Path("/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/logs/FIXED_WING/1")
out={}
for name in ["2026-01-03 15-41-42","2026-02-15 15-38-50"]:
    path=BASE/(name+".bin")
    reader=DFReader.DFReader_binary(str(path))
    counts={reader.formats[i].name:int(v) for i,v in enumerate(reader.counts) if i in reader.formats and reader.formats[i].name in ["ISBH","ISBD","GYR","ACC","IMU","ESC","VIBE"]}
    out[name]=dict(counts=counts,sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    print(name,counts,flush=True)
Path(__file__).with_name("binary-inventory.json").write_text(json.dumps(out,indent=2))
