import unittest,pathlib,json,hashlib
ROOT=pathlib.Path(__file__).resolve().parents[1]
class Retention(unittest.TestCase):
 def test_native_contradiction_preserved(self):
  r=json.loads((ROOT/'cad/assembly/evidence/servo-retention-band.json').read_text());rows={x['lengthMm']:x for x in r['rows']}
  self.assertFalse(r['accepted']);self.assertEqual(r['surfaceCount'],48)
  self.assertGreater(rows[8]['rawNativeCommonMm3'],6)
  for n in [10,12,14]:
   self.assertEqual(rows[n]['rawNativeCommonMm3'],0);self.assertGreater(rows[n]['strictSharedINCount'],0);self.assertEqual(rows[n]['nativeVerdict'],'UNKNOWN')
 def test_band_is_not_thread_acceptance(self):
  r=json.loads((ROOT/'cad/assembly/evidence/servo-retention-band.json').read_text());rows={x['lengthMm']:x for x in r['rows']}
  self.assertEqual(rows[2]['projectedBandEngagementMm'],0);self.assertGreater(rows[12]['extendsPastBandMm'],1.9);self.assertGreater(rows[8]['commonOutsideObservedBandMm3'],.19)
 def test_pins(self):
  for n in ['servo-retention-band.json','opening-support-compliance.json']:
   r=json.loads((ROOT/'cad/assembly/evidence'/n).read_text())
   for p,sha in r['inputs'].items():self.assertEqual(hashlib.sha256((ROOT/p).read_bytes()).hexdigest(),sha,p)
if __name__=='__main__':unittest.main()
