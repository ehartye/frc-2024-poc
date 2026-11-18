import pathlib,unittest,copy
import numpy as np
import mujoco
from native_drive import NativeDrive
ROOT=pathlib.Path(__file__).resolve().parents[2]
class NativeTests(unittest.TestCase):
 def test_source_geometry_and_mass_inertia(self):
  p=NativeDrive(ROOT);self.assertAlmostEqual(sum(p.m.body_mass),45.)
  for s,row in enumerate(p.wheels_source):
   for i,q in enumerate(row):
    name=f'wheel_{s}_{i}';b=p.m.body(name);g=p.m.geom(name)
    np.testing.assert_allclose(b.pos,np.array(q['worldCenterMm'])/1000,atol=1e-14)
    self.assertAlmostEqual(g.size[1],q['sizeMm'][1]/2000)
  for s in range(2):
   self.assertAlmostEqual(sum(p.m.body_inertia[p.m.body(f'wheel_{s}_{i}').id][1]+p.m.dof_armature[p.v[f'wheel_{s}_{i}']] for i in range(3)),p.source.axle_inertia)
 def test_center_drop_support(self):
  p=NativeDrive(ROOT)
  for _ in range(400):p.step()
  r=p.observe();self.assertAlmostEqual(sum(r['wheelVerticalLoadsN'].values()),45*9.80665,delta=.01)
  for side in range(2):
   self.assertGreater(r['wheelVerticalLoadsN'][f'wheel_{side}_1'],200)
   for i in [0,2]:self.assertEqual(r['wheelVerticalLoadsN'][f'wheel_{side}_{i}'],0)
 def test_coast_brake_electrical_and_atomic_invalid(self):
  p=NativeDrive(ROOT);p.d.qvel[p.v['wheel_0_1']]=5;p.d.qvel[p.v['wheel_1_1']]=5
  e,tc,w=p.motor_query([0,0],('coast','coast'));b,tb,_=p.motor_query([0,0],('brake','brake'))
  self.assertEqual(e['busPowerW'],0);self.assertEqual(b['busPowerW'],0);self.assertLess(tb[0],tc[0]);self.assertGreater(b['windingHeatW'],0)
  before=p.d.qvel.copy()
  with self.assertRaises(ValueError):p.step([2,0],('drive','drive'))
  np.testing.assert_array_equal(before,p.d.qvel)
if __name__=='__main__':unittest.main()
