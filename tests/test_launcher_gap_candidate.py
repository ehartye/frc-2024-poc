import json,pathlib,sys,unittest
ROOT=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'cad/components'))
import check_launcher_gap_candidate as q
class GapCandidateTest(unittest.TestCase):
 def test_bound_nominal_alternative_and_rejected_baseline(self):
  r=json.loads((ROOT/'cad/components/launcher-gap-candidate.json').read_text());self.assertEqual(r['inputs'],q.pins());self.assertFalse(r['adopted'])
  old,new=r['cases'];self.assertGreater(max(x['bareWheelOverlapMm3'] for s in old['samples'] for x in s['contacts']),6)
  self.assertGreater(new['bareWheelAlignedPlaneClearanceMm'],.46)
  self.assertLess(new['bareWheelAlignedPlaneClearanceMm'],.48)
  self.assertTrue(all(x['bareWheelOverlapMm3']<1e-5 for s in new['samples'] for x in s['contacts']))
  self.assertGreater(max(x['treadOverlapMm3'] for s in new['samples'] for x in s['contacts']),60)
  self.assertAlmostEqual(new['nominalIndentationPerSideMm'],.8)
 def test_all_six_baseline_misses_in_domain(self):
  r=json.loads((ROOT/'cad/components/launcher-gap-candidate.json').read_text())
  for c in r['cases']:
   self.assertTrue({150,160,170,270,280,290}.issubset({s['advanceMm'] for s in c['samples']}))
   self.assertTrue(all(len(s['contacts'])==4 for s in c['samples']))
if __name__=='__main__':unittest.main()
