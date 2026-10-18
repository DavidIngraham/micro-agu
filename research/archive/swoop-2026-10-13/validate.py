"""Freeze the two developed styles, then run an eight-case robustness set."""
import json,sys,subprocess
from pathlib import Path
r=Path(__file__).parent
conditions={'calm':{},'gust':dict(gust_amplitude_m_s=.25,gust_period_s=12,gust_phase_rad=.7),
 'crosswind':dict(wind_ned_m_s=[1.,0,0]),
 'entry':dict(cross_track_error_m=2.,height_error_m=1.,heading_error_deg=5.,speed_scale=1.05,actuator_rate_mm_s=150.,actuator_delay_s=.06)}
styles={}
for style,base in [('soft','dev07'),('dramatic','dev06')]:
 c=json.loads((r/'configs'/f'{base}.json').read_text());c.pop('name');styles[style]=c
(r/'styles.json').write_text(json.dumps(dict(selection='Frozen after seven native development runs, before this eight-case robustness set',styles=styles,conditions=conditions),indent=2))
for style,c in styles.items():
 for condition,variation in conditions.items():
  name=f'{style}-{condition}';path=r/'configs'/f'{name}.json'
  path.write_text(json.dumps(c|variation|dict(name=name),indent=2))
  subprocess.run([sys.executable,str(r/'run.py'),str(path)],check=True)
  subprocess.run([sys.executable,str(r/'analyze.py'),name],check=True)