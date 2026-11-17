import json,pathlib,sys,unittest
ROOT=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'cad/assembly'))
import bearing_seat_candidate as q
class BearingSeatTest(unittest.TestCase):
 def test_bound_seats_and_wrong_position_control(self):
  r=json.loads((ROOT/'cad/assembly/evidence/bearing-seat-candidate.json').read_text());self.assertEqual(r['inputs'],q.pins());self.assertEqual(len(r['cases']),3)
  for c in r['cases']:
   self.assertLess(c['plateNativeCommonMm3'],1e-6);self.assertLess(c['minimumSeatGapMm'],1e-6);self.assertGreater(c['wrongAxialOneMmOverlapMm3'],100)
  self.assertFalse(r['adopted']);self.assertEqual(r['originalReopenedRoots'],3)
 def test_round_bore_and_clearance_limits(self):
  r=json.loads((ROOT/'cad/assembly/evidence/bearing-seat-candidate.json').read_text());self.assertEqual(r['nominalSteppedProxy']['roundBoreDiameterMm'],13.75)
  self.assertTrue(all(t['shaftNativeOverlapMm3']<1e-6 for t in r['shaftClockingTrials']));self.assertGreater(r['oversizedShaftControlMm3'],10)
  self.assertAlmostEqual(r['lowerBearingGroundGapAtCurrentFloorPoseMm'],.65,places=5);self.assertAlmostEqual(r['raisedLowerBearingGroundGapMm'],2.65,places=5);self.assertAlmostEqual(r['existingStubExtensionBelowRaisedBearingMm'],.25,places=5)
if __name__=='__main__':unittest.main()
