import pathlib,json,hashlib,unittest
R=pathlib.Path(__file__).resolve().parents[1];H=R/'sim/component-launcher'
class TransferTests(unittest.TestCase):
 def setUp(self):self.r=json.loads((H/'transfer-evidence.json').read_text())
 def test_pins(self):
  for f,h in self.r['inputs'].items():self.assertEqual(hashlib.sha256((R/f).read_bytes()).hexdigest(),h,f)
 def test_fixed_sample_scope(self):
  self.assertEqual([r['advanceMm'] for r in self.r['rows']],list(range(0,481,10)));self.assertEqual(self.r['invalidOrNonSolidInputs'],[]);self.assertFalse(self.r['acceptance'])
 def test_preserved_weldment_counterexample(self):
  rows=[r for r in self.r['rows'] if any('weldment' in h['name'] for h in r['hits'])];self.assertEqual([r['advanceMm'] for r in rows],list(range(110,181,10)));self.assertAlmostEqual(max(sum(h['overlapMm3'] for h in r['hits'] if 'weldment' in h['name']) for r in rows),222.7425962835678)
 def test_no_hidden_exemption(self):
  for row in self.r['rows']:
   for h in row['hits']:
    self.assertGreater(h['overlapMm3'],0)
    if 'weldment' in h['name'] or 'bearing representation' in h['name']:self.assertEqual(h['contactGroup'],'unintended')
 def test_native_wheel_witnesses(self):
  r=json.loads((H/'native-wheel-witnesses.json').read_text());self.assertEqual([x['advanceMm'] for x in r['rows']],[140,220,300])
  for row in r['rows']:self.assertEqual(len(row['nativeBareWheelOverlapMm3']),4);self.assertTrue(all(v==0 for v in row['nativeBareWheelOverlapMm3'].values()))
  for f,h in r['inputs'].items():self.assertEqual(hashlib.sha256((R/f).read_bytes()).hexdigest(),h,f)
 def test_separate_contact_groups(self):
  for r in self.r['rows']:
   for group in ['pinch','launcher']:self.assertAlmostEqual(r[group+'OverlapMm3'],sum(h['overlapMm3'] for h in r['hits'] if h['contactGroup']==group))
if __name__=='__main__':unittest.main()
