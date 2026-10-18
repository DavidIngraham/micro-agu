import unittest
import numpy as np
from scipy.spatial.transform import Rotation as R
from microagu_sim.compat.json_backend import Backend
from microagu_sim.compat import dynamics

class SurfaceTests(unittest.TestCase):
    def advance(self,b,dt=.005,u=None):
        b.state,b.rpm=dynamics.surface_step(b.state,b.sim.controls(*(b.command if u is None else u)),
            b.sim.parameters,b.sim.geometry['mass'],[0,0,0],b.voltage,b.rpm,dt,b.surface)
        self.assertTrue(np.isfinite(b.state).all())
        rp=R.from_quat(b.state[3:7]);rc=R.from_quat(b.state[16:20])
        gap=b.state[:3]+rp.apply(b.sim.geometry['hinge'])-b.state[13:16]-rc.apply([0,0,.85])
        self.assertLess(np.linalg.norm(gap),1e-5)
        self.assertLessEqual(b.state[2],-.1+1e-9)
        self.assertLessEqual(b.state[15],-.03+1e-9)

    def test_hold_launch_and_contact(self):
        b=Backend(profile='as_built',ground=True)
        for _ in range(20):self.advance(b)
        self.assertEqual(b.surface[0],0)
        self.assertEqual(b.rpm,0)
        b.surface[0]=1
        for _ in range(1000):self.advance(b)
        self.assertGreaterEqual(b.surface[0],2)
        self.assertGreater(b.surface[2],.95)
        for _ in range(4000):self.advance(b,u=[0,0,0])
        self.assertEqual(b.surface[0],3)

    def test_drop_convergence(self):
        ends=[]
        for dt in [.005,.0025]:
            b=Backend(profile='as_built',altitude=1,ground=True)
            b.state[[2,15]]-=1.; b.state[[9,22]]=2.
            b.surface[:]=[2,0,1]
            for _ in range(round(2/dt)):self.advance(b,dt,u=[0,0,0])
            ends.append(b.state.copy())
        np.testing.assert_allclose(ends[0],ends[1],atol=1e-6)

    def test_impact_dissipates_energy_and_preserves_joint_velocity(self):
        def energy(b):
            p=dynamics.params(b.sim.parameters);y=b.state
            rp=R.from_quat(y[3:7]);rc=R.from_quat(y[16:20])
            vp=rp.inv().apply(y[7:10]);wp=rp.inv().apply(y[10:13])
            vc=rc.inv().apply(y[20:23]);wc=rc.inv().apply(y[23:26])
            inertia=np.array([[p[42],p[45],p[46]],[p[45],p[43],p[47]],[p[46],p[47],p[44]]])
            v1=vc+np.cross(wc,[0,0,p[36]]);v2=vc+np.cross(wc,[0,0,p[37]])
            return .5*((b.sim.geometry['mass']-p[51])*vp@vp+wp@inertia@wp+p[51]*vc@vc+
                np.dot(wc*wc,[.075+p[33],.008+p[34],.077+p[35]])+p[30]*v1[0]**2+p[31]*v2[1]**2+p[32]*v2[2]**2)
        for vx in [0.,1.,4.]:
            b=Backend(profile='as_built',ground=True);b.surface[:]=[2,0,1]
            b.state[7:10]=b.state[20:23]=[vx,0,2.]
            before=energy(b);self.advance(b,.00125,u=[0,0,0])
            self.assertLess(energy(b),before)
            y=b.state;rp=R.from_quat(y[3:7]);rc=R.from_quat(y[16:20])
            gap=y[7:10]+np.cross(y[10:13],rp.apply(b.sim.geometry['hinge']))-y[20:23]-np.cross(y[23:26],rc.apply([0,0,.85]))
            self.assertLess(np.linalg.norm(gap),1e-5)

if __name__=='__main__':unittest.main()
