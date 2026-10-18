"""Illustrative scheduled nonlinear predictor / Kalman covariance correction.

Known-model diagnostic, not a flight EKF. No canopy or rotor measurements.
Noise assumptions are declared, not identified. Heading alignment is ideal.
"""
import numpy as np
from scipy.spatial.transform import Rotation as R
from experiment import DT, XS, weights

SCALE=np.r_[XS,.5,.5,.25]
NOISE=np.r_[np.full(2,np.deg2rad(1)),np.full(3,.1),np.full(3,np.deg2rad(1)),.2]

def measurement(chart,x):
    y,_=chart.decode(x)
    return np.r_[x[:2],y[7:10],x[8:11],x[15]]

class Observer:
    def __init__(self,sim,chart,grid,initial,seed):
        self.sim=sim;self.chart=chart;self.z=np.r_[initial,np.zeros(3)]
        # Deliberately initialize hidden canopy attitude/rates with small errors.
        self.z[2:5]+=np.deg2rad([2,-2,1]);self.z[11:14]+=np.deg2rad([1,-1,1])
        self.rng=np.random.default_rng(seed+99);self.Fs=[];self.Hs=[];self.ticks=0
        for x,A,E in zip(grid['x'],grid['A'],grid['E']):
            F=np.eye(19);F[:16,:16]=A;F[:16,16:]=E
            F=F*SCALE[None,:]/SCALE[:,None]
            H=np.zeros((9,19));eps=1e-5
            H[:,:16]=np.column_stack([(measurement(chart,x+e)-measurement(chart,x-e))/(2*eps) for e in np.eye(16)*eps])
            H=H*SCALE[None,:]/NOISE[:,None]
            self.Fs.append(F);self.Hs.append(H)
        self.Fs=np.array(self.Fs);self.Hs=np.array(self.Hs);self.H=self.Hs[13]
        self.P=np.eye(19)*.1
        self.Q=np.diag(np.r_[np.full(16,1e-5),np.full(3,.01)])

    @property
    def wind(self):return self.z[16:]

    def correct(self,truth_x):
        # Navigation velocity is expressed in heading-aligned axes for this chart.
        y=measurement(self.chart,truth_x)+self.rng.normal(size=9)*NOISE
        selected=np.arange(9) if self.ticks%10==0 else np.array([0,1,5,6,7])
        H=self.H[selected];residual=((y-measurement(self.chart,self.z[:16]))/NOISE)[selected]
        L=np.linalg.solve(H@self.P@H.T+np.eye(len(selected)),H@self.P).T
        self.z+=SCALE*(L@residual)
        J=np.eye(19)-L@H;self.P=J@self.P@J.T+L@L.T
        self.ticks+=1
        return self.z[:16].copy()

    def predict(self,command,voltage,climb,turn):
        y,rpm=self.chart.decode(self.z[:16])
        yn,rn=self.sim.step(y,rpm,command,voltage=voltage,wind=self.wind,dt=DT)
        # The chart removes absolute heading; rotate the wind estimate with it.
        dyaw=R.from_quat(yn[3:7]).as_euler('xyz')[2]-R.from_quat(y[3:7]).as_euler('xyz')[2]
        self.z[16:]=R.from_euler('z',-dyaw).apply(self.wind)
        self.z[:16]=self.chart.encode(yn,rn)
        w=weights(voltage,climb,turn);F=np.tensordot(w,self.Fs,axes=1)
        self.H=np.tensordot(w,self.Hs,axes=1)
        self.P=F@self.P@F.T+self.Q
