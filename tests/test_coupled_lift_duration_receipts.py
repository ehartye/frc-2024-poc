import json,pathlib,sys,unittest
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/integrated'))
import coupled_lift_duration as full,coupled_lift_event_refine as half
D=R/'sim/integrated/evidence/lift-duration-domain'
class DurationReceiptTest(unittest.TestCase):
 def test_exact_source_inventory(self):
  for name,m in [('100ms.json',full),('events-half-step.json',half)]:
   r=json.loads((D/name).read_text());pins=m.setup()[-1];pins.update({f:m.sha(R/f) for f in m.EXTRA});self.assertEqual(r['inputs'],pins);self.assertEqual([c['name'] for c in r['cases']],[p[0] for p in m.PLAN]);self.assertFalse(r['acceptance'])
 def test_partial_witness_loss_not_release(self):
  for name in ['100ms.json','events-half-step.json']:
   r=json.loads((D/name).read_text())
   failures=[c for c in r['cases'] if c['rejectedEvaluation']]
   self.assertEqual({c['name'] for c in failures},{'both-plus-003','coast','brake'})
   for c in failures:
    q=c['rejectedEvaluation'];self.assertEqual(c['reason'],'UNKNOWN_FINITE_PATCH_PERSISTENCE');self.assertEqual(sum(p['valid'] for p in q['patchWitnesses']['points']),28);self.assertEqual(q['guideQuery']['status'],'SEPARATED_FROM_ENTIRE_GUIDE');self.assertFalse(q['accepted']);self.assertEqual(c['finalState'],c['trace'][-1]['state']);self.assertGreater(q['state']['timeS'],c['finalState']['timeS'])
 def test_targeted_step_refinement(self):
  a={c['name']:c for c in json.loads((D/'100ms.json').read_text())['cases']};b=json.loads((D/'events-half-step.json').read_text())['cases']
  for c in b:
   prior=a[c['name']];self.assertLessEqual(abs(prior['summary']['acceptedTimeS']-c['summary']['acceptedTimeS']),.000501);self.assertLess(c['summary']['maxIncrementalEnergyErrorJ'],prior['summary']['maxIncrementalEnergyErrorJ'])
if __name__=='__main__':unittest.main()
