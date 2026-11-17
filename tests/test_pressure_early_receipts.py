import unittest,pathlib,sys,json
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/pressure'))
import summarize_early_policy as s
class EarlyReceipts(unittest.TestCase):
 def test_source_bound_same_time_reconstruction(self):
  e=s.summarize();self.assertEqual(e,json.loads((R/'sim/pressure/early-policy-evidence/summary.json').read_text()));self.assertTrue(e['controlsPhysicalTraceEquality']);self.assertLess(abs(e['halfMinusCoarse']['noteX']),2e-7);self.assertLess(abs(e['halfMinusCoarse']['chassisX']),3e-7);self.assertGreater(e['rows'][0]['relativeX'],.26);self.assertGreater(e['rows'][0]['chassisVx'],.5);self.assertLess(abs(e['rows'][1]['newPhaseResidualJ']),abs(e['rows'][0]['newPhaseResidualJ'])/4)
if __name__=='__main__':unittest.main()
