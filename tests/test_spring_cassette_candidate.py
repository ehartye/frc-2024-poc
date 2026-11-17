import json,pathlib,hashlib,unittest
ROOT=pathlib.Path(__file__).resolve().parents[1]
def read(f):return json.loads((ROOT/'cad/assembly/evidence'/f).read_text())
class CassetteCandidate(unittest.TestCase):
 def test_final_source_pins(self):
  for file in ['spring-cassette-candidate.json','spring-cassette-native-feed.json']:
   for name,digest in read(file)['inputs'].items():self.assertEqual(hashlib.sha256((ROOT/name).read_bytes()).hexdigest(),digest,name)
 def test_only_inherited_proxy_contacts(self):
  r=read('spring-cassette-candidate.json');self.assertEqual(len(r['samples']),9)
  for s in r['samples']:
   self.assertEqual(s['candidatePartCount'],28);self.assertEqual(len(s['hits']),4)
   for h in s['hits']:self.assertEqual(h['aRole'],'drive');self.assertEqual(h['bRole'],'drive')
  self.assertFalse(r['adopted'])
 def test_native_feed_counterexamples(self):
  r=read('spring-cassette-native-feed.json');self.assertEqual(len(r['rows']),150)
  for u,count in [(0,0),(.75,2),(1.5,5)]:self.assertEqual(sum(bool(s['hits']) for s in r['rows'] if s['inwardTravelMm']==u),count)
  self.assertFalse(r['acceptance'])
 def test_export_and_unknown_force(self):
  r=read('spring-cassette-export.json');self.assertTrue(r['readOkay']);self.assertEqual(r['roots'],28);self.assertEqual(r['transferred'],28)
  c=json.loads((ROOT/'cad/assembly/spring-cassette-travel-contract.json').read_text());self.assertIsNone(c['springs']['ratesNPerM']);self.assertIsNone(c['movingCassetteMassKg']);self.assertFalse(c['openingActuator']['selected']);self.assertFalse(c['adopted'])
if __name__=='__main__':unittest.main()
