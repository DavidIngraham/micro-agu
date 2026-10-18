from pathlib import Path
import numpy as np,json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
O=Path(__file__).parent
# Illustrative inertia only: uniform radial blades, total 15 g, radius 114.3 mm.
# This excludes motor bell inertia and is a scale estimate, not a measurement.
I=.015*(.2286/2)**2/3
fig,axs=plt.subplots(3,2,figsize=(12,9),sharex='col');report=[]
for col,(day,cut,label) in enumerate([('2026-01-03',447,'Hood River'),('2026-02-15',3387,'Trout Lake')]):
 a=np.load(O/f'{day}-causal-input.npz');b=np.load(O/f'{day}-propulsion-9x45.npz');mask=(a['t']>=cut-5)&(a['t']<=cut+10);t=a['t'][mask];Y=a['Y'][mask]
 rpm=np.interp(t,b['t'],b['rpm']);power=np.interp(t,b['t'],b['shaft_power_apc']);thrust=np.interp(t,b['t'],b['thrust_apc']);omega=rpm*2*np.pi/60
 Q=np.divide(power,omega,out=np.full_like(power,np.nan),where=omega>100)
 gyro_y=-I*omega*Y[:,5];gyro_z=I*omega*Y[:,4]
 axs[0,col].plot(t-cut,.02*thrust,label='Old |reaction torque| = 0.02 T');axs[0,col].plot(t-cut,Q,label='APC proxy shaft power / angular speed');axs[0,col].set_title(label)
 axs[1,col].plot(t-cut,gyro_y,label='Gyroscopic pitch moment');axs[1,col].plot(t-cut,gyro_z,label='Gyroscopic yaw moment')
 # Diagnostic derivative of sampled RPM, not used as a causal simulation input.
 spin=-I*np.gradient(omega,t);axs[2,col].plot(t-cut,spin,label='Spin-up roll reaction (sampled RPM derivative)')
 for row in range(3):axs[row,col].axvline(0,color='k',alpha=.3);axs[row,col].grid(alpha=.25);axs[row,col].legend(fontsize=8);axs[row,col].set_ylabel('Moment (N m)')
 axs[2,col].set_xlabel('Seconds relative to forecast start')
 report.append(dict(day=day,cutoff=cut,median_old_torque_Nm=float(np.nanmedian(.02*thrust)),median_APC_proxy_torque_Nm=float(np.nanmedian(Q)),gyro_pitch_p95_Nm=float(np.nanpercentile(abs(gyro_y),95)),gyro_yaw_p95_Nm=float(np.nanpercentile(abs(gyro_z),95)),spinup_abs_p95_Nm=float(np.nanpercentile(abs(spin),95))))
fig.suptitle('Rotor moments: scale audit using flight RPM and body rates',fontsize=15)
fig.text(.5,.01,'Gyro/spin-up illustration: 15 g uniform radial blades, I = %.2g kg m²; motor rotor omitted. Positive spin about body +x.\nAPC 9×4.5 is a proxy for the installed prop. Sampled RPM differentiation amplifies noise; these are not calibrated moments.'%I,ha='center',fontsize=9)
fig.tight_layout(rect=[0,.065,1,.96]);fig.savefig(O/'rotor-moment-scale.png',dpi=160);plt.close(fig)
(O/'rotor-moment-scale.json').write_text(json.dumps(dict(illustrative_spin_inertia_kg_m2=I,assumptions='15 g uniform radial blades, radius 114.3 mm, motor rotor omitted. Not measured. Positive spin about +x. RPM derivative diagnostic only.',results=report),indent=2));print(json.dumps(report,indent=2))
