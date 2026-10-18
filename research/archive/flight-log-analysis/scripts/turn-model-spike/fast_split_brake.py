from pathlib import Path
import ctypes,numpy as np
from parafoil_two_body import DEFAULT
KEYS=['rho','S','b','c','CL0','CLa','CD0','k_drag','CYbeta','CLbrake','CDbrake','Clbeta','Clp','Cldelta','Cm0','Cma','Cmq','payload_CDA','thrust_max','thrust_z','prop_torque','joint_k_roll','joint_k_pitch','joint_k_yaw','joint_c_roll','joint_c_pitch','joint_c_yaw','Cnbeta','Cnr','Cndelta']
KEYS=KEYS+['brake_lever_fraction']
DEFAULT=DEFAULT|dict(brake_lever_fraction=.25)
lib=ctypes.CDLL(str(Path(__file__).with_name('libfast_split_brake.so')));ptr=np.ctypeslib.ndpointer(dtype=np.float64,flags='C_CONTIGUOUS');lib.derivative.argtypes=[ptr,ptr,ptr,ctypes.c_double,ptr,ptr];lib.replay.argtypes=[ptr,ptr,ctypes.c_int,ctypes.c_double,ctypes.c_int,ptr,ctypes.c_double,ptr,ptr];lib.replay.restype=ctypes.c_int

def params(p=None):
 d=DEFAULT.copy();d.update(p or {});return np.array([d[k] for k in KEYS])
def derivative(y,u,p,m,w):
 out=np.zeros(26);lib.derivative(np.ascontiguousarray(y),np.ascontiguousarray(u,dtype=float),params(p),m,np.ascontiguousarray(w,dtype=float),out);return out
def replay(y,U,p,m,w,dt=.04,substeps=2):
 U=np.ascontiguousarray(U,dtype=float);out=np.zeros((len(U),26));n=lib.replay(np.ascontiguousarray(y),U,len(U),dt,substeps,params(p),m,np.ascontiguousarray(w,dtype=float),out)
 if n!=len(U):raise FloatingPointError('Replay diverged')
 return out

