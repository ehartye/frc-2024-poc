import unittest,pathlib,json,hashlib,sys,math
R=pathlib.Path(__file__).resolve().parents[1];H=R/'sim/retention';sys.path.insert(0,str(H));import wrench
class RetentionEvidence(unittest.TestCase):
 def setUp(self):self.r=json.loads((H/'evidence.json').read_text());self.g=json.loads((R/'cad/assembly/evidence/retention-patch.json').read_text())
 def test_current_pins(self):
  for f,h in self.r['inputs'].items():self.assertEqual(hashlib.sha256((R/f).read_bytes()).hexdigest(),h,f)
 def test_complete_cases_and_actual_offset(self):
  self.assertEqual(len(self.r['cases']),28);self.assertIn(9.88835038313412,{c['noteOffsetMm'] for c in self.r['cases']});self.assertTrue(self.r['trajectoryRequirementsOnly']);self.assertFalse(self.r['acceptance'])
 def test_slower_conditional_and_fast_failure(self):
  for c in self.r['cases']:
   if c['case']=='slower':self.assertEqual(c['statusCounts'],{'CONSTRUCTIVE_ALLOCATION_WITHIN_HYPOTHESIS':441})
   if c['case']=='fast':self.assertGreater(c['statusCounts'].get('EXCEEDS_NECESSARY_CAPACITY_BOUND',0),0)
   if c['case']=='one-side-loss':self.assertEqual(set(c['statusCounts']),{'UNRESOLVED_NONPLANAR_WRENCH'})
 def test_one_normal_budget_per_side(self):
  p=[dict(side=x['side'],positionM=[v/1000 for v in x['noteRelativePointMm']]) for x in self.g['cases'][0]['points']]
  a=wrench.allocate(p,[0.,2.3],0.,.6,{-1:10.,1:10.});self.assertAlmostEqual(sum(a['pointCapacitiesN']),12.)
 def test_independent_allocations_at_worst_samples(self):
  for c in self.r['cases']:
   geo=next(g for g in self.g['cases'] if g['state']=='floor' and g['noteLongitudinalOffsetMm']==c['noteOffsetMm']);p=[dict(side=x['side'],positionM=[v/1000 for v in x['noteRelativePointMm']]) for x in geo['points']]
   if c['case']=='point-contact-control':p=[dict(side=x['side'],positionM=[0.,x['positionM'][1],0.]) for x in p]
   row=max(c['rows'],key=lambda r:r['maxUtilization']);a=wrench.allocate(p,row['forceLocalXZ'],row['torqueNm'],c['friction'],{int(k):v for k,v in c['normalPerSideN'].items()})
   self.assertEqual(a['status'],row['status']);self.assertAlmostEqual(a['maxUtilization'],row['maxUtilization'])
 def test_power_off_not_static_hold_claim(self):
  for c in self.r['cases']:
   self.assertFalse(c['rows'][-1]['powerOffStaticHoldSupported']);self.assertFalse(c['acceptance'])
if __name__=='__main__':unittest.main()
