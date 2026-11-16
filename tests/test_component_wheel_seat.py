import pathlib,json,unittest
R=pathlib.Path(__file__).resolve().parents[1]
class WheelSeatTests(unittest.TestCase):
 def test_seat_not_bounds(self):
  r=json.loads((R/'cad/components/wheel-seat-evidence.json').read_text());self.assertAlmostEqual(r['midwidthSeatRadiusMm'],45.72);self.assertAlmostEqual(r['finishedRadiusMm'],48.26);self.assertAlmostEqual(r['treadCenterOffsetFromAssemblyMm'],1.5875);self.assertFalse(r['nominalGripScreenPass']);self.assertLess(r['nominalGripStackMm'],r['rivetGripRangeMm'][0])
  for p in r['pointWitnesses']:
   if p['radiusMm'] in [45.8,46.8]:self.assertFalse(p['inside'])
 def test_component_config_matches_witness(self):
  r=json.loads((R/'cad/components/wheel-seat-evidence.json').read_text());c=json.loads((R/'cad/components/a2-transmission.json').read_text())['serviceInterfaces']['tread'];self.assertAlmostEqual(c['nativeSeatRadiusMm'],r['midwidthSeatRadiusMm']);self.assertAlmostEqual(c['assemblyTreadCenterOffsetMm'],r['treadCenterOffsetFromAssemblyMm'])
if __name__=='__main__':unittest.main()
