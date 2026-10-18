"""Publish final landing Monte Carlo results and annotate replay from native PGFL."""
import argparse
import hashlib
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from pymavlink import mavutil

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('suite', type=Path)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
rows = json.loads((args.suite/'results.json').read_text())
good = [r for r in rows if r['completed_contact']]
landing = [r['landing'] for r in good]
radial = np.array([r['radial_error_m'] for r in landing])
sink = np.array([r['contact']['velocity'][2] for r in landing])
speed = np.array([np.linalg.norm(r['contact']['velocity'][:2]) for r in landing])
cross = [r['cross_track_error_m'] for r in landing]
along = [r['along_track_error_m'] for r in landing]
args.output.mkdir(parents=True, exist_ok=True)
summary = dict(attempted=len(rows), completed=len(good), within_5m=int(sum(radial<=5)),
               median_m=float(np.median(radial)), p90_m=float(np.percentile(radial,90)),
               worst_m=float(max(radial)), median_sink_m_s=float(np.median(sink)),
               p90_sink_m_s=float(np.percentile(sink,90)), median_groundspeed_m_s=float(np.median(speed)))
previous = json.loads((args.output/'summary.json').read_text())
summary['log_audits'] = [r for r in previous['audits'] if r['suite']=='heldout-candidate']
(args.output/'final-results.json').write_text(json.dumps(summary,indent=2))
(args.output/'final-conditions.json').write_text((args.suite/'manifest.json').read_text())
fig, axes = plt.subplots(1,3,figsize=(14,4.5))
for radius in [1,2,5]:
 axes[0].add_patch(plt.Circle((0,0),radius,fill=False,color='.75'))
 axes[0].text(radius,0,str(radius)+' m',fontsize=8)
axes[0].scatter(cross,along,c=speed,cmap='viridis',edgecolors='black',s=55)
axes[0].scatter(0,0,marker='+',s=110,color='black')
axes[0].set(xlabel='Cross-track error (m)',ylabel='Along-track error (m; negative = short)',title='Final landing footprint',xlim=(-6,6),ylim=(-6,6))
axes[0].set_aspect('equal')
axes[1].step(np.r_[0,np.sort(radial),6],np.r_[0,np.arange(1,len(radial)+1)/len(rows),len(radial)/len(rows)],where='post',color='#228d8a')
axes[1].axvline(5,color='.7',ls='--')
axes[1].set(xlabel='Distance from target (m)',ylabel='Fraction of attempted trials',title='12 final randomized conditions',ylim=(0,1.05))
pts=axes[2].scatter(radial,sink,c=speed,cmap='viridis',edgecolors='black',s=55)
axes[2].set(xlabel='Distance from target (m)',ylabel='Downward speed at crossing (m/s)',title='Arrival speeds')
fig.colorbar(pts,ax=axes[2],label='Ground speed (m/s)')
fig.tight_layout();fig.savefig(args.output/'landing-final.png',dpi=170)
replay_path=args.output/'replay.json'
replay=json.loads(replay_path.read_text())
logpath=args.suite/('moderate-%02d'%replay['selected_trial'])/'flight.BIN'
log=mavutil.mavlink_connection(str(logpath))
while True:
 msg=log.recv_match(type='PGFL')
 if msg is None: raise RuntimeError('Missing native flare activation')
 if msg.Active:
  replay['flare_time_s']=msg.TimeUS*1e-6
  replay['flare_event_source']='First active native PGFL sample; SITL simulation timestamp'
  replay['flare_log_sha256']=hashlib.sha256(logpath.read_bytes()).hexdigest()
  break
assert replay['frames'][0]['time_s']<replay['flare_time_s']<replay['frames'][-1]['time_s']
replay_path.write_text(json.dumps(replay,separators=(',',':')))
print(json.dumps(summary,indent=2))
