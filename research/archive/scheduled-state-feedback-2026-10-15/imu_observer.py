"""Payload accelerometer extension; truth is used only to generate a noisy measurement.

This remains a known-model diagnostic with ideal heading alignment and no IMU bias.
"""
import numpy as np
from scipy.spatial.transform import Rotation as R
from observer import Observer,measurement,SCALE,NOISE
from experiment import DT,weights,brake_command,setup,MODEL

ACC_NOISE=.2  # m/s^2 per-axis sample standard deviation, illustrative not identified

class ImuObserver(Observer):
    config=dict(state_q=1e-7,wind_q=.001)
    latest=None
    def __init__(self,sim,chart,grid,initial,seed):
        self.sensor_sim=sim
        # Fixed mission predictor: softer-roll tests intentionally have model mismatch.
        predictor=setup(MODEL)('trout_lake','mission');predictor.substeps=16
        super().__init__(predictor,chart,grid,initial,seed)
        self.Q=np.diag(np.r_[np.full(16,self.config['state_q']),np.full(3,self.config['wind_q'])])
        self.acc_noise_rng=np.random.default_rng(seed+199)
        self.ah=[];self.errors=[];ImuObserver.latest=self
        for x,u,voltage in zip(grid['x'],grid['u'],[20.4]*9+[22.2]*9):
            command=np.r_[u[0],brake_command(u[1])]
            z=np.r_[x,np.zeros(3)];eps=1e-5
            H=np.column_stack([(self.acceleration(z+e,command,voltage)-self.acceleration(z-e,command,voltage))/(2*eps) for e in np.eye(19)*eps])
            self.ah.append(H*SCALE[None,:]/ACC_NOISE)
        self.ah=np.array(self.ah);self.acc_H=self.ah[13]

    def acceleration(self,z,command,voltage,sim=None):
        y,rpm=self.chart.decode(z[:16]);delta=.00125
        yn,_=(self.sim if sim is None else sim).step(y,rpm,command,voltage=voltage,wind=z[16:],dt=delta)
        return R.from_quat(y[3:7]).inv().apply((yn[7:10]-y[7:10])/delta-np.array([0,0,9.81]))

    def context(self,state,rpm,voltage,wind,command):
        # Sensor generation only: no truth wind/state is stored for the predictor.
        yaw=R.from_quat(state[3:7]).as_euler('xyz')[2]
        z=np.r_[self.chart.encode(state,rpm),R.from_euler('z',-yaw).apply(wind)]
        self.acc_measured=self.acceleration(z,command,voltage,sim=self.sensor_sim)+self.acc_noise_rng.normal(size=3)*ACC_NOISE
        self.command=command;self.voltage=voltage

    def correct(self,truth_x):
        super().correct(truth_x)
        residual=(self.acc_measured-self.acceleration(self.z,self.command,self.voltage))/ACC_NOISE
        H=self.acc_H
        L=np.linalg.solve(H@self.P@H.T+np.eye(3),H@self.P).T
        self.z+=SCALE*(L@residual)
        J=np.eye(19)-L@H;self.P=J@self.P@J.T+L@L.T
        self.errors.append(self.z[:16]-truth_x)
        return self.z[:16].copy()

    def predict(self,command,voltage,climb,turn):
        super().predict(command,voltage,climb,turn)
        self.acc_H=np.tensordot(weights(voltage,climb,turn),self.ah,axes=1)
