"""Export bounded swoop replays and an evidence-focused local research report."""
import json,html,subprocess
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
r=Path(__file__).parent
raw=Path('/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/spiral-swoop-2026-10-13')
viewer=r.parents[1]/'viewer/assets/recordings/landing'
selection={'replays':{'spiral':'spiral06'}}
results=json.loads((r/'results.json').read_text())
for style,name in selection['replays'].items():
 d=raw/name;land=json.loads((d/'landing.json').read_text());frames=json.loads((d/'viewer.json').read_text());end=land['contact']['time']
 left=max((f for f in frames if f['time_s']<=end),key=lambda f:f['time_s']);right=min((f for f in frames if f['time_s']>=end),key=lambda f:f['time_s'])
 q=(end-left['time_s'])/(right['time_s']-left['time_s']) if right['time_s']>left['time_s'] else 0
 state=np.array(left['state'])+q*(np.array(right['state'])-left['state'])
 for offset in [3,16]:state[offset:offset+4]=Slerp([0,1],Rotation.from_quat([left['state'][offset:offset+4],right['state'][offset:offset+4]]))([q]).as_quat()[0]
 endpoint=dict(left,time_s=end,state=state.tolist())
 for key in ['rpm','throttle','voltage_v','brake_mm','wind_ned_m_s','joint_rotvec_rad','joint_rate_rad_s']:
  endpoint[key]=(np.array(left[key])+q*(np.array(right[key])-left[key])).tolist()
 clip=[f for f in frames if f['time_s']<end]+[endpoint]
 data=dict(frames=clip,target_ned_m=land['target_ned_m'],result=land,label='Descending spiral · native AUTO',
           flare_time_s=results[name]['flare_time'],parameters=results[name]['parameters'],
           selection='Frozen-style calm robustness run; first CG ground-plane crossing. High-brake physics is not flight validated.',
           flare_event_source='First active native PGFL sample')
 (viewer/'spiral.json').write_text(json.dumps(data,separators=(',',':')))


import shutil
for name in ['spiral06','spiral06-gust']:
 shutil.copy2(raw/name/'response.png',r/f'{name}.png')
attempts=[]
for cfg in sorted((r/'configs').glob('*.json')):
 d=raw/cfg.stem
 attempts.append(dict(name=cfg.stem,config=json.loads(cfg.read_text()),completion=json.loads((d/'completed.json').read_text()),contact=json.loads((d/'landing.json').read_text()).get('contact')))
(r/'attempts.json').write_text(json.dumps(attempts,indent=2))
md=(r/'README.md').read_text()
body=subprocess.check_output(['/home/david/.npm/_npx/52027bd8fc0022aa/node_modules/node/bin/node','-e',"const {marked}=require('/home/david/repos/DavidIngraham.github.io/assets/vendor/marked.js');process.stdout.write(marked.parse(require('fs').readFileSync(0,'utf8')));"],input=md,text=True)
body=body.replace('<h2>Results</h2>','<iframe title="Spiral landing replay" src="../../viewer/dist/landing.html?case=spiral"></iframe><h2>Results</h2>')
(r/'report.html').write_text('<!doctype html><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Spiral landing</title><style>body{max-width:1100px;margin:30px auto;padding:0 20px;font:18px/1.6 system-ui;background:#faf9f5;color:#203440}img{max-width:100%}iframe{width:100%;height:1100px;border:0}table{display:block;overflow:auto;border-collapse:collapse;font-size:15px}td,th{padding:8px;border-bottom:1px solid #bbb}</style>'+body)
