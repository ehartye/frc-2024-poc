import pathlib,sys,unittest
from dataclasses import replace,asdict
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/pressure'))
import two_cassette as c
class TwoCassette(unittest.TestCase):
 def setUp(self):
  p=c.one.Parameters(resolution=128);self.p=c.Parameters(p,p);self.s=c.State(0.,0.,.00004,0.,.2,0.,.00004,0.,.2,0.);self.cfg=c.one.config()
 def test_symmetric_force_and_single_note_mass(self):
  s=replace(self.s,ydot=.01);r=c.ports(s,self.p,self.cfg)
  self.assertEqual(r['noteForceY'],0.)
  expected=.5*self.p.positive.note_mass*s.ydot**2
  self.assertAlmostEqual(r['kineticJ'],expected,places=15)
 def test_pressure_potential_gradients(self):
  s=replace(self.s,y=.0001,udn=.001,tdp=.01);r=c.ports(s,self.p,self.cfg);h=1e-8
  for name,force in [('y',r['noteForceY']),('up',r['sides'][0]['cassetteContactForce']),('un',r['sides'][1]['cassetteContactForce'])]:
   plus=c.ports(replace(s,**{name:getattr(s,name)+h}),self.p,self.cfg)['pressureJ'];minus=c.ports(replace(s,**{name:getattr(s,name)-h}),self.p,self.cfg)['pressureJ']
   self.assertAlmostEqual(-(plus-minus)/(2*h),force,places=7)
  self.assertLess(abs(r['powerResidualW']),1e-12)
 def test_reflection_exchanges_sides(self):
  s=replace(self.s,y=.0001,ydot=.001,up=.00003,un=.00005,udp=.002,tdn=.1)
  t=c.State(-s.y,-s.ydot,s.un,s.udn,s.tn,s.tdn,s.up,s.udp,s.tp,s.tdp)
  a=c.ports(s,self.p,self.cfg,(.001,-.002));b=c.ports(t,self.p,self.cfg,(-.002,.001))
  self.assertEqual(a['energyJ'],b['energyJ']);self.assertEqual(a['noteForceY'],-b['noteForceY'])
 def test_asymmetric_preload_does_not_prescribe_note_force(self):
  p=replace(self.p,negative=replace(self.p.negative,spring_free_length=.007));r=c.ports(self.s,p,self.cfg)
  self.assertEqual(r['sides'][1]['springForce'],0);self.assertLess(r['sides'][1]['cassetteContactForce'],0)
  self.assertNotEqual(r['accelerations'][1],r['accelerations'][3]);self.assertEqual(r['noteForceY'],0)
 def test_atomic_moving_stop(self):
  s=replace(self.s,un=.001499,udn=1.);before=asdict(s)
  with self.assertRaises(ValueError):c.step(s,self.p,self.cfg,.0001)
  self.assertEqual(asdict(s),before)
 def test_torque_off_does_not_reset_state(self):
  s=replace(self.s,tdp=.1,tdn=.2,ydot=.001);out=c.step(s,self.p,self.cfg,1e-6)
  self.assertNotEqual(out.tdp,0);self.assertNotEqual(out.tdn,0);self.assertNotEqual(out.ydot,0)
 def test_shared_note_and_shape_validation(self):
  with self.assertRaises(ValueError):c.ports(self.s,replace(self.p,negative=replace(self.p.negative,note_mass=1)),self.cfg)
  with self.assertRaises(ValueError):c.ports(replace(self.s,y=True),self.p,self.cfg)
  with self.assertRaises(ValueError):c.ports(self.s,self.p,self.cfg,(0.,))
if __name__=='__main__':unittest.main()
