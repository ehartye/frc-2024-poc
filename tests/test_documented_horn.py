import unittest,pathlib,json,hashlib
ROOT=pathlib.Path(__file__).resolve().parents[1]
class DocumentedHorn(unittest.TestCase):
 def test_actual_pair_and_head(self):
  r=json.loads((ROOT/'cad/assembly/evidence/documented-horn-native.json').read_text());pairs={(x['a'],x['b']):x['commonMm3'] for x in r['nativePairRows']}
  self.assertEqual(pairs['servo','horn'],0);self.assertGreater(r['hornShiftIntoShaftNegativeMm3'],7)
  self.assertGreater(r['nominalHeadRecessMm'],.24);self.assertLess(r['nominalHeadRecessMm'],.26)
  b=r['hornSourceBoundsMm'];e=r['hornRepresentationBoundsMm'];self.assertTrue(all(e[i]<=b[i] and e[i+3]>=b[i+3] for i in range(3)));self.assertGreater(r['nominalAxialSeatFaceAreaMm2'],0);self.assertGreater(pairs['servo','center screw'],0)
  self.assertGreater(r['rejectedPocketedEnvelopeOutsideMm3'],1.5);self.assertTrue(all(x['outsideRepresentationMm3']<1e-6 for x in r['nativeRepresentationContainment']));self.assertFalse(r['accepted']);self.assertTrue(all(x['commonMm3']==0 for x in r['authoredInterfaceRows']))
 def test_source_identity_not_v2_servo(self):
  r=json.loads((ROOT/'cad/assembly/documented-horn-sources.json').read_text());self.assertEqual(r['servo'],'REV-41-1097 V1');self.assertEqual(r['horn'],'REV-41-1828 V2');self.assertEqual(r['centerScrew'],'REV-41-1359 M3x8');self.assertFalse(r['adopted'])
 def test_receipt_pins(self):
  for n in ['documented-horn-native.json','documented-horn-screen.json']:
   r=json.loads((ROOT/'cad/assembly/evidence'/n).read_text())
   for p,sha in r['inputs'].items():self.assertEqual(hashlib.sha256((ROOT/p).read_bytes()).hexdigest(),sha,p)
 def test_external_screen(self):
  r=json.loads((ROOT/'cad/assembly/evidence/documented-horn-screen.json').read_text());self.assertEqual(len(r['rows']),27);self.assertTrue(all(not x['hits'] for x in r['rows']));self.assertFalse(r['adopted'])
if __name__=='__main__':unittest.main()
