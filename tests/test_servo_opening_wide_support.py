import unittest,pathlib,json,hashlib
ROOT=pathlib.Path(__file__).resolve().parents[1]
class Support(unittest.TestCase):
 def test_geometry_scope(self):
  r=json.loads((ROOT/'cad/assembly/evidence/servo-opening-wide-support.json').read_text());self.assertEqual(len(r['rows']),27);self.assertFalse(r['adopted']);self.assertTrue(all(not x['hits'] for x in r['rows']))
 def test_real_contact_faces(self):
  r=json.loads((ROOT/'cad/assembly/evidence/wide-support-interfaces.json').read_text());self.assertFalse(r['accepted']);self.assertEqual(r['authoredMutualHits'],[])
  for row in r['contacts']:self.assertEqual(row['commonMm3'],0);self.assertEqual(row['minimumDistanceMm'],0);self.assertGreater(row['coplanarFaceIntersectionAreaMm2'],40)
  for row in r['nativeSupportChecks']:self.assertEqual(row['commonMm3'],0)
 def test_all_pins(self):
  for name in ['servo-opening-wide-support.json','wide-support-interfaces.json','wide-support-export.json']:
   r=json.loads((ROOT/'cad/assembly/evidence'/name).read_text())
   for p,sha in r['inputs'].items():self.assertEqual(hashlib.sha256((ROOT/p).read_bytes()).hexdigest(),sha,p)
if __name__=='__main__':unittest.main()
