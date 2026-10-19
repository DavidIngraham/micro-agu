import numpy as np
from scipy.signal import welch
from analyze import fold
fs=50.
assert np.allclose(fold([0,25,50,101.6,201.6],fs),[0,25,0,1.6,1.6])
t=np.arange(0,20,1/fs)
for frequency in [1.6,101.6,201.6]:
    y=np.sin(2*np.pi*frequency*t)
    f,p=welch(y,fs=fs,nperseg=500,noverlap=250)
    assert abs(f[np.argmax(p)]-1.6)<1e-9
    assert abs(np.sum(p)*(f[1]-f[0])-.5)<1e-9
# Two-blade passage at 3048 RPM is 101.6 Hz and aliases to 1.6 Hz.
assert abs(float(fold(2*3048/60,50))-1.6)<1e-9
print("Synthetic order/alias, spectral peak, and PSD power checks passed")
