import pathlib,sys,json,hashlib,unittest
R=pathlib.Path(__file__).resolve().parents[1]
class Evidence(unittest.TestCase):
 def test_nominal_pins_and_nonzero_state(self):
  r=json.loads((R/'sim/integrated/evidence/speed-approach-nominal.json').read_text())
  for p,h in r['inputs'].items():self.assertEqual(hashlib.sha256((R/p).read_bytes()).hexdigest(),h,p)
  self.assertEqual(r['outcome'],'GROUND_DWELL_DIAGNOSTIC_ONLY');self.assertFalse(r['acceptance'])
  f=r['final'];self.assertGreater(f['chassis']['vx'],.01);self.assertGreater(min(f['note']['w']),1.)
  self.assertGreater(f['relativeRobotXY'][0],.22);self.assertLess(r['maxAbsEnergyResidualJ'],.002)
  self.assertEqual(f['controller']['state'],'REGULATE')
if __name__=='__main__':unittest.main()
