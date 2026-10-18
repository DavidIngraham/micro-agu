from pathlib import Path
import ctypes,numpy as np
from parafoil_two_body import DEFAULT
KEYS=['rho','S','b','c','CL0','CLa','CD0','k_drag','CYbeta','CLbrake','CDbrake','Clbeta','Clp','Cldelta','Cm0','Cma','Cmq','payload_CDA','thrust_max','thrust_z','prop_torque','joint_k_roll','joint_k_pitch','joint_k_yaw','joint_c_roll','joint_c_pitch','joint_c_yaw','Cnbeta','Cnr','Cndelta']
KEYS+=['added_mx','added_my','added_mz','added_Jx','added_Jy','added_Jz','added_z1','added_z2']
DEFAULT=DEFAULT.copy();DEFAULT.update(dict(zip(KEYS[-8:],[.01,.1,.5,.01,.005,.005,0.,.5])))
KEYS+=['canopy_half_arc']
DEFAULT['canopy_half_arc']=.6
KEYS += ['hinge_x','hinge_y','hinge_z','Ixx','Iyy','Izz','Ixy','Ixz','Iyz','thrust_x','thrust_y','thrust_arm_z']
DEFAULT.update(dict(zip(KEYS[39:],[0,0,-.13,.006,.008,.003,0,0,0,0,0,-.058])))
KEYS+=['canopy_mass'];DEFAULT['canopy_mass']=.19
lib=ctypes.CDLL(str(Path(__file__).with_name('libfast_napkin_causal.so')));ptr=np.ctypeslib.ndpointer(dtype=np.float64,flags='C_CONTIGUOUS');lib.derivative.argtypes=[ptr,ptr,ptr,ctypes.c_double,ptr,ptr];lib.replay.argtypes=[ptr,ptr,ctypes.c_int,ctypes.c_double,ctypes.c_int,ptr,ctypes.c_double,ptr,ptr];lib.replay.restype=ctypes.c_int

def params(p=None):
 d=DEFAULT.copy();d.update(p or {});scale=d.get('added_scale',1.)
 for k in KEYS[30:36]:d[k]*=scale
 return np.array([d[k] for k in KEYS])
def derivative(y,u,p,m,w):
 out=np.zeros(26);lib.derivative(np.ascontiguousarray(y),np.ascontiguousarray(u,dtype=float),params(p),m,np.ascontiguousarray(w,dtype=float),out);return out
def replay(y,U,p,m,w,dt=.04,substeps=4):
 U=np.ascontiguousarray(U,dtype=float);out=np.zeros((len(U),26));n=lib.replay(np.ascontiguousarray(y),U,len(U),dt,substeps,params(p),m,np.ascontiguousarray(w,dtype=float),out)
 if n!=len(U):raise FloatingPointError('Replay diverged')
 return out


lib.causal_replay.argtypes=[ptr,ptr,ctypes.c_int,ctypes.c_double,ctypes.c_int,ptr,ctypes.c_double,ptr,ctypes.c_double,ctypes.c_double,ptr,ptr]
lib.causal_replay.restype=ctypes.c_int
lib.mapped_thrust.argtypes=[ctypes.c_double,ctypes.c_double];lib.mapped_thrust.restype=ctypes.c_double
def causal_replay(y,U,p,m,w,voltage,rpm,dt=.04,substeps=4):
 U=np.ascontiguousarray(U,dtype=float);out=np.zeros((len(U),26));r=np.zeros(len(U));n=lib.causal_replay(np.ascontiguousarray(y),U,len(U),dt,substeps,params(p),m,np.ascontiguousarray(w,dtype=float),voltage,rpm,out,r)
 if n!=len(U):raise FloatingPointError('Causal replay diverged')
 return out,r
