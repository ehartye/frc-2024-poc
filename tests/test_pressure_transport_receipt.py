import pathlib,sys,json,hashlib,unittest
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/pressure'));import run_transport as run
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
class Receipt(unittest.TestCase):
 def test_exact_pins_and_work(self):
  r=json.loads((R/'sim/pressure/transport-evidence.json').read_text());b=json.loads((R/'sim/pressure/evidence.json').read_text());pins=dict(b['inputs']);pins['sim/pressure/evidence.json']=sha(R/'sim/pressure/evidence.json');pins.update({f:sha(R/f) for f in run.FILES});self.assertEqual(r['inputs'],pins);self.assertEqual(len(r['cases']),9)
  for c in r['cases']:self.assertLess(abs(c['result']['workResidualW']),1e-12);self.assertFalse(c['result']['acceptance'])
  self.assertEqual([c['activeSamples'] for c in r['cases'][:3]],[0,0,0]);self.assertGreater(r['cases'][3]['features']['outer-wrap'],0)
if __name__=='__main__':unittest.main()
