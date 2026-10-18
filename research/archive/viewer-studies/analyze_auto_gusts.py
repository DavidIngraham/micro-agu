"""Compare longitudinal motion in the published calm and gusty AUTO excerpts."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.signal import periodogram
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('root',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
fig,axes=plt.subplots(5,1,figsize=(12,12),sharex=True,layout='constrained'); summary={}
for name,folder,color in [('Calm','viewer-auto-mission','#697c8c'),('Synthetic gusts','viewer-auto-turbulence','#d96521')]:
 frames=[json.loads(s) for s in (a.root/folder/'auto-clip.jsonl').read_text().splitlines()]
 t=np.array([f['time_s'] for f in frames]);t-=t[0]; y=np.array([f['state'] for f in frames]);wind=np.array([f.get('wind_ned_m_s',[0,0,0]) for f in frames])
 pitch=Rotation.from_quat(y[:,3:7]).as_euler('xyz',degrees=True)[:,1];cpitch=Rotation.from_quat(y[:,16:20]).as_euler('xyz',degrees=True)[:,1]
 altitude=-y[:,2]; climb=-y[:,9]; throttle=np.array([f['throttle'] for f in frames])*100
 vals=[altitude-90,climb,pitch,throttle,-wind[:,2]]
 for ax,v in zip(axes,vals):ax.plot(t,v,color=color,label=name,linewidth=1)
 axes[2].plot(t,cpitch,color=color,ls='--',alpha=.6,label=name+' canopy')
 stats={'source_clip_sha256':hashlib.sha256((a.root/folder/'auto-clip.jsonl').read_bytes()).hexdigest()}
 for label,v in zip(['height_error_m','climb_m_s','payload_pitch_deg','throttle_pct','vertical_gust_up_m_s'],vals):
  freq,power=periodogram(v,fs=1/np.median(np.diff(t)),detrend='linear');sel=(freq>=.04)&(freq<=1)
  stats[label]=dict(min=float(v.min()),max=float(v.max()),rms=float(np.sqrt(np.mean(v*v))),std=float(v.std()),dominant_hz=float(freq[sel][np.argmax(power[sel])]))
 stats['payload_canopy_pitch_difference_deg']=dict(min=float((pitch-cpitch).min()),max=float((pitch-cpitch).max()))
 summary[name]=stats
for ax,label in zip(axes,['Height error (m)','Climb (m/s)','Pitch (deg)','Throttle (%)','Upward gust (m/s)']):ax.set_ylabel(label);ax.grid(alpha=.25)
axes[0].legend();axes[2].legend(ncol=2);axes[-1].set_xlabel('Seconds from first-loiter approach (independently aligned excerpts)')
fig.suptitle('AUTO loiter entry: calm versus synthetic gusts\nSame controller; distinct runs. Dashed pitch is canopy attitude; solid is payload.')
a.output.mkdir(parents=True,exist_ok=True);fig.savefig(a.output/'auto-longitudinal-gusts.png',dpi=150);(a.output/'auto-longitudinal-gusts.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2))
