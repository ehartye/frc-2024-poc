import json,pathlib,hashlib,unittest
ROOT=pathlib.Path(__file__).resolve().parents[1]
class CoupledRetentionGeometry(unittest.TestCase):
 def test_pins(self):
  r=json.loads((ROOT/'cad/assembly/evidence/coupled-retention-geometry.json').read_text())
  for n,v in r['inputs'].items():self.assertEqual(hashlib.sha256((ROOT/n).read_bytes()).hexdigest(),v,n)
 def test_actual_state_and_rejected_pitch(self):
  r=json.loads((ROOT/'cad/assembly/evidence/coupled-retention-geometry.json').read_text());self.assertAlmostEqual(r['longitudinalOffsetMm'],50.67604936032026)
  self.assertEqual(len(r['samples']),15)
  self.assertTrue(all(len(s['points'])==30 and s['partCount']==711 for s in r['samples']))
  collisions=[s['pitchDeg'] for s in r['samples'] if any(not h['intendedContactHypothesis'] for h in s['hits'])]
  self.assertEqual(collisions,[10,20,30]);self.assertFalse(r['forceBound']['necessaryStaticConditionMet']);self.assertFalse(r['acceptance'])
  self.assertGreater(r['footprintXmm'][0],r['straightFaceXmm'][0]);self.assertLess(r['footprintXmm'][1],r['straightFaceXmm'][1])
if __name__=='__main__':unittest.main()
