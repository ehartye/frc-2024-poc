import unittest,pathlib,json,hashlib
ROOT=pathlib.Path(__file__).resolve().parents[1]
class Fasteners(unittest.TestCase):
 def test_source_closure(self):
  for name in ['servo-opening-fasteners.json','servo-opening-fasteners-screen.json']:
   r=json.loads((ROOT/'cad/assembly/evidence'/name).read_text())
   for p,sha in r['inputs'].items():self.assertEqual(hashlib.sha256((ROOT/p).read_bytes()).hexdigest(),sha,p)
   self.assertIn('cad/assembly/servo-opening-fastener-source.json',r['inputs'])
 def test_finite_target_and_wrong_alignment(self):
  r=json.loads((ROOT/'cad/assembly/evidence/servo-opening-fasteners.json').read_text())
  self.assertEqual(len(r['finiteThreadTargets']),2)
  for t in r['finiteThreadTargets']:
   self.assertGreater(t['rawMm3'],0);self.assertEqual(t['outsideSourceTargetMm3'],0);self.assertGreater(t['shiftedOutsideTargetMm3'],0)
 def test_center_head_negative_control(self):
  r=json.loads((ROOT/'cad/assembly/evidence/servo-opening-fasteners.json').read_text())
  self.assertGreater(r['negativeControls']['oldAdapterCenterHeadCollisionMm3'],70)
  self.assertEqual(r['negativeControls']['newAdapterCenterHeadCollisionMm3'],0)
  self.assertFalse(r['accepted']);self.assertGreater(r['shaftHornCommonMm3'],7)
  self.assertEqual(r['authoredMutualHits'],[])
 def test_full_robot_retained_scope(self):
  r=json.loads((ROOT/'cad/assembly/evidence/servo-opening-fasteners-screen.json').read_text())
  self.assertEqual(len(r['rows']),27);self.assertFalse(r['adopted']);self.assertTrue(all(not row['hits'] for row in r['rows']))
if __name__=='__main__':unittest.main()
