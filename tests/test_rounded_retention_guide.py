import unittest,pathlib,json,hashlib,math
ROOT=pathlib.Path(__file__).resolve().parents[1]
class GuideCandidate(unittest.TestCase):
 def test_bound_exact_state(self):
  r=json.loads((ROOT/'cad/assembly/evidence/rounded-retention-guide.json').read_text())
  for n,v in r['inputs'].items():self.assertEqual(hashlib.sha256((ROOT/n).read_bytes()).hexdigest(),v,n)
  self.assertEqual(r['indexedReceiptSha256'],'60b0e42aaf86bc934be1c0844cd7c9ed7fafa35efa910adef01dd4404675eac9')
  self.assertAlmostEqual(r['longitudinalOffsetMm'],10.5674400200612);self.assertFalse(r['adopted']);self.assertFalse(r['acceptance']);self.assertEqual(r['reopenedRoots'],1)
 def test_geometry_witness_and_unknown(self):
  r=json.loads((ROOT/'cad/assembly/evidence/rounded-retention-guide.json').read_text());self.assertEqual(len(r['samples']),16)
  self.assertEqual(r['guide']['guideThicknessMm'],1)
  self.assertEqual(r['samples'][0]['guideOverlapMm3'],0)
  self.assertGreater(max(s['guideOverlapMm3'] for s in r['samples']),3000)
  unknown=[s for s in r['samples'] if not s['booleanWitnessConsistent']];self.assertEqual([s['pitchDeg'] for s in unknown],[20]);self.assertTrue(unknown[0]['interiorMaterialWitnesses'])
  for s in r['samples']:
   for w in s['guideSurfaceWitnesses']:self.assertAlmostEqual(sum(x*x for x in w['guideOutwardNormal']),1)
if __name__=='__main__':unittest.main()
