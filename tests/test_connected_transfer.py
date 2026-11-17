import unittest,pathlib,json,hashlib
R=pathlib.Path(__file__).resolve().parents[1];P=R/'sim/component-launcher'
class ConnectedTransfer(unittest.TestCase):
 def setUp(self):self.t=json.loads((P/'connected-transfer-evidence.json').read_text());self.n=json.loads((P/'connected-native-wheel-evidence.json').read_text())
 def test_current_pins(self):
  for report in [self.t,self.n]:
   for p,h in report['inputs'].items():self.assertEqual(hashlib.sha256((R/p).read_bytes()).hexdigest(),h,p)
 def test_full_assembly_domain(self):
  self.assertEqual(self.t['assemblyPartCount'],711);self.assertEqual(self.t['invalidOrNonSolidInputs'],[])
  self.assertEqual([r['advanceMm'] for r in self.t['rows']],list(range(0,481,10)))
 def test_only_shell_proxy_hits(self):
  for row in self.t['rows']:
   for h in row['hits']:
    if h['contactGroup']=='unintended':self.assertIn('original wheel shell representation',h['name'])
 def test_native_witnesses_complete_and_bound(self):
  self.assertEqual(self.n['transferSha256'],hashlib.sha256((P/'connected-transfer-evidence.json').read_bytes()).hexdigest())
  self.assertEqual([r['advanceMm'] for r in self.n['rows']],list(range(140,301,10)))
  self.assertTrue(all(len(r['nativeBareWheelOverlapMm3'])==4 for r in self.n['rows']))
 def test_positive_native_contacts_retained(self):
  positives=[r['advanceMm'] for r in self.n['rows'] if max(r['nativeBareWheelOverlapMm3'].values())>1e-5]
  self.assertEqual(positives,[150,160,170,270,280,290])
  self.assertFalse(self.t['acceptance'])
if __name__=='__main__':unittest.main()
