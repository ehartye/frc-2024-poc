import copy,json,pathlib,sys,unittest
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/integrated'))
import coupled_lift_sweep as original,coupled_lift_separated_sweep as separated
D=R/'sim/integrated/evidence/lift-command-envelope'
class ReceiptTest(unittest.TestCase):
 def test_source_inventory_and_commands(self):
  for name in ['10ms.json','10ms-half-step.json','25ms.json','separated-10ms.json','separated-25ms.json','separated-25ms-half-step.json']:
   module=separated if name.startswith('separated') else original
   r=json.loads((D/name).read_text());pins=module.setup()[-1];pins.update({f:module.sha(R/f) for f in module.EXTRA})
   self.assertEqual(r['inputs'],pins,name);self.assertEqual([c['name'] for c in r['cases']],[p[0] for p in module.PLAN]);self.assertFalse(r['acceptance'])
   for case in r['cases']:
    self.assertFalse(case['acceptance']);self.assertAlmostEqual(case['summary']['acceptedTimeS'],len(case['trace'])*r['stepS'],places=12)
    for row in case['trace']:
     self.assertEqual(row['energyJ']['inheritedNormalSpring'],r['config']['inheritedNormalSpringJ']);self.assertIn('UNKNOWN',row['guideMaterialStatus'])
 def test_original_accepted_prefix_unchanged(self):
  for old,new in [('10ms.json','separated-10ms.json'),('25ms.json','separated-25ms.json')]:
   a=json.loads((D/old).read_text());b=json.loads((D/new).read_text())
   for x,y in zip(a['cases'],b['cases']):
    self.assertEqual(x['initialState'],y['initialState']);self.assertEqual(x['commands'],y['commands'])
    for before,after in zip(x['trace'],y['trace']):
     after=copy.deepcopy(after);after.pop('endGuideQuery');self.assertEqual(before,after)
 def test_fallback_has_certificate_not_contact_acceptance(self):
  r=json.loads((D/'separated-25ms.json').read_text());count=0
  for case in r['cases']:
   for row in case['trace']:
    q=row['endGuideQuery']
    if q.get('separationFallbackUsed'):
     count+=1;self.assertTrue(q['separationCertificate']['separated']);self.assertEqual(q['penetrationM'],0);self.assertEqual(q['originalStatus'],'UNKNOWN');self.assertIn('UNKNOWN',q['nativeMaterialStatus'])
  self.assertGreater(count,0)
if __name__=='__main__':unittest.main()
