import unittest,sys,pathlib
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'cad/assembly'))
from cassette_spring_law import spring_state
class SpringLaw(unittest.TestCase):
 def test_source_nominal(self):
  self.assertAlmostEqual(spring_state(0)['pairInwardForceN'],14.6234)
  self.assertAlmostEqual(spring_state(.0015)['pairInwardForceN'],5.9534)
 def test_energy_gradient(self):
  u=.0007;eps=1e-8
  derivative=(spring_state(u+eps)['storedEnergyJ']-spring_state(u-eps)['storedEnergyJ'])/(2*eps)
  self.assertAlmostEqual(-derivative,spring_state(u)['pairInwardForceN'],places=7)
 def test_rate_and_stops_not_contact(self):
  self.assertAlmostEqual(spring_state(0,rate_fraction=.1)['pairInwardForceN'],14.6234*1.1)
  self.assertTrue(spring_state(.0015)['atInwardStop'])
  self.assertIsNone(spring_state(.0015)['stopReactionN']);self.assertIsNone(spring_state(.0015)['noteNormalForceN'])
 def test_invalid(self):
  for u in [-.001,.002,float('nan')]:
   with self.assertRaises(ValueError):spring_state(u)
