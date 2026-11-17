import pathlib,sys,unittest,math
import numpy as np
from dataclasses import replace
R=pathlib.Path(__file__).resolve().parents[1];sys.path[:0]=[str(R/'sim/pressure'),str(R/'cad/assembly')]
import cassette_inertia as c
class Inertia(unittest.TestCase):
 def setUp(self):self.p=c.default_partition();self.q=[0.,0.,.4,0.,0.,.2,.2];self.v=[.3,-.2,.7,0.,0.,0.,0.]
 def test_locked_aggregate_mass_com_yaw(self):
  r=c.evaluate(self.p,self.q,self.v);m=np.array(r['massMatrix'])
  np.testing.assert_allclose(m[:3,:3],np.diag([45,45,6]),atol=1e-14)
  np.testing.assert_allclose(r['bias'][:3],[0,0,0],atol=1e-15)
  a=c.acceleration(self.p,self.q,self.v,[45,90,18,0,0,0,0],locked=(3,4,5,6))
  np.testing.assert_allclose(a['acceleration'][:3],[1,2,3],atol=1e-14);self.assertEqual(a['constraintPowerW'],0)
 def test_asymmetric_internal_force_reacts_on_chassis(self):
  q=list(self.q);q[2]=0;v=[0.]*7
  a=c.acceleration(self.p,q,v,[0,0,0,1,0,0,0])['acceleration']
  self.assertNotEqual(a[1],0);self.assertNotEqual(a[2],0)
  r=c.evaluate(self.p,q,v);linear=np.zeros(2)
  for b in r['bodies']:linear+=b['massKg']*(np.array(b['jacobian'])@a)
  np.testing.assert_allclose(linear,[0,0],atol=1e-14)
 def test_energy_power_with_transport_terms(self):
  q=[.2,-.1,.7,.0004,.0008,.3,.5];v=[.4,-.2,.9,.003,-.002,.1,-.2];force=[2,-3,.4,4,-2,.001,-.002]
  result=c.acceleration(self.p,q,v,force);a=np.array(result['acceleration']);h=1e-7
  plus=c.evaluate(self.p,np.array(q)+h*np.array(v),np.array(v)+h*a)['kineticJ'];minus=c.evaluate(self.p,np.array(q)-h*np.array(v),np.array(v)-h*a)['kineticJ']
  self.assertAlmostEqual((plus-minus)/(2*h),float(np.array(force)@v),places=7)
 def test_yaw_objectivity(self):
  q=np.array(self.q);v=np.array(self.v);a=c.evaluate(self.p,q,v);angle=.8;rot=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]]);q[:2]=rot@q[:2];q[2]+=angle;v[:2]=rot@v[:2];b=c.evaluate(self.p,q,v)
  self.assertAlmostEqual(a['kineticJ'],b['kineticJ'],places=14)
 def test_invalid_partition_and_locking(self):
  with self.assertRaises(ValueError):replace(self.p,total_mass=2).terms()
  with self.assertRaises(ValueError):replace(self.p,total_yaw_inertia=.01).terms()
  with self.assertRaises(ValueError):c.acceleration(self.p,self.q,self.v,[0]*7,locked=(0,))
  with self.assertRaises(ValueError):c.evaluate(self.p,[True]*7,self.v)
if __name__=='__main__':unittest.main()
