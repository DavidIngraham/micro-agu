"""Plot the deployed feedback matrix in its design normalization."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import SymLogNorm
from synthesize import arrays

here = Path(__file__).resolve().parent
a = arrays((here/'designs/responsive_softspeed/AP_CustomControl_LQI_gains.h').read_text())
# Command percentage points for one positive normalized error unit.
c = -100*a['US'][:, None]*a['K']
labels = ['Payload roll\n10°', 'Payload pitch\n10°', 'Relative roll\n10°',
          'Relative pitch\n10°', 'Relative yaw\n10°', 'Air velocity x\n0.5 m/s',
          'Air velocity y\n0.5 m/s', 'Air velocity z\n0.5 m/s',
          'Payload roll rate\n30°/s', 'Payload pitch rate\n30°/s',
          'Payload yaw rate\n30°/s', 'Canopy roll rate\n30°/s',
          'Canopy pitch rate\n30°/s', 'Canopy yaw rate\n30°/s',
          'Rotor speed\n1000 rpm', 'Height\n1 m', '∫ height error\n1 m·s',
          '∫ turn-rate error\n10°', '∫ speed error\n0.5 m']
fig, ax = plt.subplots(figsize=(18, 6.8))
fig.subplots_adjust(left=.11, right=.88, bottom=.38, top=.84)
im = ax.imshow(c, cmap='RdBu_r', norm=SymLogNorm(linthresh=.05, vmin=-13,vmax=13), aspect='auto')
ax.set_xticks(range(19), labels, rotation=55, ha='right', fontsize=9)
ax.set_yticks(range(3), ['Throttle', 'Common brake', 'Differential brake'])
for j in range(3):
    for i in range(19):
        v=c[j,i]
        ax.text(i,j,f'{v:+.2f}',ha='center',va='center',fontsize=8,
                color='white' if abs(v)>.8 else 'black')
ax.axvline(15.5,color='black',lw=2)
ax.set_title('Published LQI: individual feedback contributions\nCommand percentage points per positive error shown below each column', pad=17)
fig.colorbar(im,ax=ax,shrink=.8,label='Signed command change (percentage points; symmetric log color scale)')
fig.text(.01,.015,'Exact deployed K and output scales. State errors are relative to mapped references; integral states use implemented error signs.\nThese are isolated pre-limit contributions, not observed flight activity. Trim/feedforward and actuator limits are excluded.',fontsize=10)
fig.savefig(here/'gain-contributions.png',dpi=170)
fig.savefig(here/'gain-contributions.svg')
