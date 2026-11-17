import unittest,pathlib,json,hashlib,sys
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/pressure'));import benchmark_geometry as bench
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
class BenchmarkReceipt(unittest.TestCase):
 def test_exact_inventory_and_scope(self):
  r=json.loads((R/'sim/pressure/grid-benchmark.json').read_text());b=json.loads((R/'sim/pressure/transport-evidence.json').read_text());pins=dict(b['inputs']);pins['sim/pressure/transport-evidence.json']=sha(R/'sim/pressure/transport-evidence.json');pins.update({f:sha(R/f) for f in bench.FILES});self.assertEqual(r['inputs'],pins);self.assertTrue(r['contactByteEquality']);self.assertTrue(r['forceWorkByteEquality']);self.assertEqual(r['arguments']['resolution'],[1024,1024]);self.assertEqual(r['activeSamples'],880);self.assertFalse(r['acceptance']);self.assertNotIn('sim/pressure/pressure_acquisition.py',r['inputs'])
if __name__=='__main__':unittest.main()
