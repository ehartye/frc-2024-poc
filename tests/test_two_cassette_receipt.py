import pathlib,sys,unittest,json,hashlib
from dataclasses import replace
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/pressure'))
import two_cassette as c
class Receipt(unittest.TestCase):
 def setUp(self):self.r=json.loads((R/'sim/pressure/two-cassette-evidence.json').read_text())
 def test_source_pins(self):
  for p,h in self.r['inputs'].items():self.assertEqual(hashlib.sha256((R/p).read_bytes()).hexdigest(),h,p)
 def test_fine_grid_offset_domain(self):
  case=self.r['cases']['offset_control'];s=c.State(**case['initial']);p=c.Parameters(c.one.Parameters(**case['parameters']['positive']),c.one.Parameters(**case['parameters']['negative']))
  with self.assertRaisesRegex(ValueError,'UNSUPPORTED_DEEP_OR_CAVITY_CONTACT'):c.ports(s,p,c.one.config(),case['torques'])
  self.assertEqual(case['outcome'],'UNKNOWN_INITIAL_DOMAIN');self.assertEqual(case['rows'],[])
 def test_outcomes_and_refinement(self):
  cases=self.r['cases'];self.assertEqual(len(cases),8)
  for key,case in cases.items():
   if key!='offset_control':self.assertEqual(case['outcome'],'HORIZON');self.assertAlmostEqual(case['final']['time'],.002)
  for coarse,fine in [('symmetric_holding','symmetric_half'),('zero_preload','zero_preload_half'),('asymmetric_preload','asymmetric_preload_half')]:self.assertLess(cases[fine]['maxResidualJ'],cases[coarse]['maxResidualJ'])
  self.assertEqual(cases['symmetric_holding']['final']['y'],0)
  self.assertLess(cases['asymmetric_preload']['final']['ydot'],0)
  self.assertNotEqual(cases['torque_removed']['final']['tdp'],0)
if __name__=='__main__':unittest.main()
