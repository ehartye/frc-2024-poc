import json,pathlib,sys,unittest
ROOT=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'cad/assembly'))
import build_connected_candidate as a
class CartridgeInterfacesTest(unittest.TestCase):
 def test_current_mounts_bound_and_raw_contacts_retained(self):
  r=json.loads((ROOT/'cad/assembly/evidence/cartridge-gap-corrected-mounts.json').read_text())
  self.assertEqual({k:v for k,v in r['inputs'].items() if k!='cad/assembly/audit_cartridge_interfaces.py'},a.input_pins())
  self.assertEqual(r['gapMm'],49.2)
  for s in r['states']:
   self.assertEqual(s['parts'],711);self.assertEqual(len(s['hits']),16)
   self.assertFalse(any('connection web' in h['a'] or 'connection web' in h['b'] for h in s['hits']))
   self.assertFalse(any('driver pulley' in h['b'] and 'bridge proposal' in h['a'] for h in s['hits']))
   self.assertEqual(len(s['proposedWebSeats']),8)
   self.assertTrue(all(x['minimumGapMm']<1e-6 for x in s['proposedWebSeats']))
 def test_preserved_inherited_mounting_counterexample(self):
  p=ROOT/'cad/assembly/evidence';old=json.loads((p/'cartridge-gap-baseline.json').read_text());gap=json.loads((p/'cartridge-gap-wider-gap.json').read_text())
  self.assertAlmostEqual(old['gapMm'],46.8);self.assertAlmostEqual(gap['gapMm'],49.2)
  for x,y in zip(old['states'],gap['states']):
   self.assertEqual(len(x['hits']),30);self.assertEqual({(h['a'],h['b']) for h in x['hits']},{(h['a'],h['b']) for h in y['hits']})
if __name__=='__main__':unittest.main()
