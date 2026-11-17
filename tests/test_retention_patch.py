import pathlib,json,sys,unittest
ROOT=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'cad/assembly'))
import extract_retention_patch as q
class RetentionPatchTest(unittest.TestCase):
 def test_bound_domain_and_points(self):
  r=json.loads((ROOT/'cad/assembly/evidence/retention-patch.json').read_text());self.assertEqual(r['inputs'],q.pins())
  self.assertEqual({(c['state'],c['noteLongitudinalOffsetMm']) for c in r['cases']},{(s,x) for s in ['floor','rotated','handoff'] for x in [0,10,20]})
  for c in r['cases']:
   self.assertEqual(len(c['points']),30)
   for p in c['points']:
    self.assertEqual(p['inwardNormalCradle'],[0,-p['side'],0]);self.assertEqual(p['cradlePointMm'][0]-p['noteRelativePointMm'][0],c['noteLongitudinalOffsetMm'])
  self.assertFalse(r['acceptance'])
 def test_transform_preserves_actual_offset(self):
  self.assertEqual(q.world([10,2,3],0,[178,0,25.4]),[188,2,28.4])
  p=q.world([10,0,0],__import__('math').pi/3,[178,0,474.8333949243112]);self.assertAlmostEqual(p[0],183);self.assertAlmostEqual(p[2],483.4936489621556)
if __name__=='__main__':unittest.main()
