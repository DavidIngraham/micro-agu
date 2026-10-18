"""Export display traces; metrics retain the audited 50 Hz results."""
import json
import sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
out = []
for r in json.loads((HERE/'iteration2/final-validation.json').read_text()):
    d = np.load(HERE/'traces'/f'{r["case"]}-{r["controller"]}.npz')['trace'][::5]
    out.append(dict(case=r['case'], observer=r['observer'], family=r['family'],
        metrics={k:r[k] for k in ['height_rms','pitch_rms','heading_rms']},
        start=5 if r['case'] in ['maneuver','speed_transition','limits_recovery'] else 15,
        trace=np.round(np.column_stack([d[:,0],d[:,1]-d[:,2],d[:,7],d[:,5]-d[:,6],d[:,11]]),5).tolist()))
Path(sys.argv[1]).write_text(json.dumps(dict(source='dbd0f27', columns=['time_s','height_error_m','pitch_rate_deg_s','heading_rate_error_deg_s','voltage_V'], runs=out), separators=(',',':'))+'\n')
