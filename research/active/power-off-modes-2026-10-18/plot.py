import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=Path(__file__).resolve().parent
rows=json.loads((p/'results.json').read_text());flight=json.loads((p/'flight-windows.json').read_text())
fig,axes=plt.subplots(1,2,figsize=(11,4),layout='constrained')
for start in [3350,3366,3373,3459]:
 r=next(x for x in flight if x['start']==start)
 axes[0].semilogy(r['frequency'],np.array(r['rate_psd'])[:,2],label=f'{start} s: '+('powered' if start==3350 else 'prop stopped'))
axes[0].set(xlim=(.5,4),xlabel='Frequency (Hz)',ylabel='Yaw-rate PSD ((deg/s)²/Hz)',title='Trout Lake: six-second measured windows')
axes[0].legend(fontsize=8)
for gyro,style in [(True,'-'),(False,'--')]:
 for th,color in [(0.,'#287fa1'),(.4,'#bf6131')]:
  r=next(x for x in rows if x['flight']=='trout_lake' and x['gyro']==gyro and x['throttle']==th and x['dt']==.004)
  axes[1].plot(r['brake_response']['hz'],np.array(r['brake_response']['magnitude'])[:,2],style,color=color,label=f'Throttle {th:g}, gyro '+('on' if gyro else 'off'))
axes[1].axvspan(2.,2.2,color='gray',alpha=.15)
axes[1].set(xlabel='Frequency (Hz)',ylabel='Yaw-rate gain ((deg/s)/mm)',title='Model: differential-brake response, 12 mm trim')
axes[1].legend(fontsize=8)
fig.savefig(p/'comparison.png',dpi=180)
