import pathlib,sys,unittest,json
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/pressure'))
import bilateral_geometry as g
import bilateral_transport as b
import fast_geometry as original
import finite_transport as t
class Bilateral(unittest.TestCase):
 def setUp(self):self.cfg=json.loads((R/'cad/assembly/intake_components.json').read_text());self.kw=dict(note_center=[.01056744002,0.,0.],resolution=(128,128))
 def test_zero_travel_exact_geometry_and_transport(self):
  p=g.prepare(self.cfg,**self.kw);ref=original.prepare(self.cfg,**self.kw);self.assertEqual(p,ref)
  k=dict(note_velocity=[.2,.003,0],note_omega=[0,0,.1],cradle_velocity=[.1,-.002,0],cradle_omega=[0,0,.2],rotor_omega=[15,16],pitch_radius_m=.012)
  actual=b.rates(p,**k);expected=t.rates(ref,**k)
  self.assertEqual(actual.pop('cassetteGeneralizedForceN')[0],expected.pop('cassetteGeneralizedForceN'));self.assertEqual(actual,expected)
 def test_two_travel_potential_gradients_and_work(self):
  kw=dict(self.kw,u_positive_m=.00002,u_negative_m=.00003);p=g.prepare(self.cfg,**kw)
  r=b.rates(p,u_velocities=[.003,-.002],note_velocity=[.2,.003,0],cradle_velocity=[.1,0,0],rotor_omega=[15,16],pitch_radius_m=.012,mu=0)
  h=1e-8
  for i,key in enumerate(['u_positive_m','u_negative_m']):
   plus=dict(kw);minus=dict(kw);plus[key]+=h;minus[key]-=h
   force=-(g.prepare(self.cfg,**plus)['potentialJ']-g.prepare(self.cfg,**minus)['potentialJ'])/(2*h)
   self.assertAlmostEqual(force,r['cassetteGeneralizedForceN'][i],places=7)
  self.assertLess(abs(r['workResidualW']),1e-12)
 def test_friction_work_with_both_travels(self):
  p=g.prepare(self.cfg,**self.kw);r=b.rates(p,u_velocities=[.003,-.002],note_velocity=[.2,.003,0],cradle_velocity=[.1,0,0],rotor_omega=[15,16],pitch_radius_m=.012)
  self.assertGreater(r['slipHeatW'],0);self.assertLess(abs(r['workResidualW']),1e-12)
 def test_invalid_negative_travel_and_extreme(self):
  with self.assertRaises(ValueError):g.prepare(self.cfg,**self.kw,u_negative_m=-.0001)
  with self.assertRaises(ValueError):g.prepare(self.cfg,note_center=[1e308,0,0],u_negative_m=.0001)
if __name__=='__main__':unittest.main()
