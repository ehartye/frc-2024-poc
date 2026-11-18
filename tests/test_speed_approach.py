import pathlib,sys,unittest,math,json
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/integrated'))
from speed_approach import observe,SpeedApproach
class SpeedTests(unittest.TestCase):
 def setUp(self):
  self.c=json.loads((R/'sim/drivetrain/config.json').read_text());self.d=dict(x=1.,y=2.,heading=.7,vx=.3,vy=-.2,yaw_rate=1.2,wheel=[0.,0.]);self.n=dict(p=[1.4,2.2],v=[-.1,.4])
 def test_frame_derivative_finite_difference(self):
  a=observe(0,self.d,self.n);h=1e-7;d=dict(self.d);n=dict(self.n)
  d.update(x=d['x']+h*d['vx'],y=d['y']+h*d['vy'],heading=d['heading']+h*d['yaw_rate']);n['p']=[p+h*v for p,v in zip(n['p'],n['v'])];b=observe(h,d,n)
  self.assertAlmostEqual((b['relativeX']-a['relativeX'])/h,a['relativeVx'],places=6);self.assertAlmostEqual((b['relativeY']-a['relativeY'])/h,a['relativeVy'],places=6)
 def test_common_rigid_rotation_zero_relative_velocity(self):
  d=dict(self.d,vx=0.,vy=0.);n=dict(self.n,v=[-1.2*.2,1.2*.4]);o=observe(0,d,n)
  self.assertAlmostEqual(o['relativeVx'],0);self.assertAlmostEqual(o['relativeVy'],0)
 def test_fault_latches_and_no_state_reset(self):
  c=SpeedApproach(self.c);o=observe(0,dict(self.d,yaw_rate=0.),self.n);before=json.dumps(self.n)
  self.assertEqual(c.update(.1,o)['modes'],['brake','brake']);self.assertEqual(json.dumps(self.n),before)
  self.assertEqual(c.update(.1,dict(o,timestamp=.1,relativeY=0))['state'],'OBSERVATION_FAULT')
 def test_target_taper_and_zero_pinch_control(self):
  c=SpeedApproach(self.c,pinch_enabled=False);o=observe(0,dict(self.d,yaw_rate=0.),self.n);o.update(relativeY=0,relativeX=.22,relativeVx=0)
  r=c.update(0,o);self.assertLess(r['desiredSpeed'],.01);self.assertEqual(r['pinchDemands'],[0.,0.])
 def test_invalid_types_and_yaw_not_silently_controlled(self):
  o=observe(0,self.d,self.n)
  self.assertEqual(SpeedApproach(self.c).update(0,o)['state'],'OBSERVATION_FAULT')
  o['timestamp']=True;self.assertEqual(SpeedApproach(self.c).update(0,o)['state'],'OBSERVATION_FAULT')
if __name__=='__main__':unittest.main()
