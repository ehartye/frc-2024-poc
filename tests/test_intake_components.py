import unittest,json,pathlib,sys,math
ROOT=pathlib.Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'cad'),str(ROOT/'cad/assembly')]
import intake_components as q
import a2_primitives as h
class IntakeTest(unittest.TestCase):
 def setUp(self):self.c=json.loads((ROOT/'cad/assembly/intake_components.json').read_text())
 def test_chain_length_closes_nominal_center(self):
  c=self.c;l=c['lift'];self.assertAlmostEqual((l['chainLinksHypothesis']-l['sprocketTeeth'])*6.35/2,l['upperPulleyZ']-l['lowerPulleyZ'])
 def test_lift_accounts_for_rotation_rise(self):
  c=self.c;l=c['lift'];travel=c['handoffCenterZ']-c['floorCenterZ'];self.assertGreater(travel,440)
  for z in [c['floorCenterZ'],c['handoffCenterZ']]:
   anchor=z+l['carriageAnchorAboveNote'];self.assertGreater(anchor,l['lowerPulleyZ']+l['sprocketOuterRadius']);self.assertLess(anchor,l['upperPulleyZ']-l['sprocketOuterRadius'])
 def test_source_sizes_not_invented_belt_inventory(self):self.assertEqual(self.c['pinch']['beltTeethHypothesis'],104);self.assertIn('pre2024',self.c['limits'][0])
 def test_explicit_motion_and_configured_pitch(self):
  import audit_intake_motion as audit
  c=dict(self.c,pitchDeg=30);self.assertEqual(max(x[1] for x in audit.poses(c)),30)
  rows=q.build(metadata=True);self.assertEqual(len({x['id'] for x in rows}),len(rows));self.assertEqual({x['motion'] for x in rows},{'fixed','carriage','cradle'})
 def test_shapes_valid_all_key_poses(self):
  for state in ['floor','rotated','handoff']:
   for n,s,k in q.build(state):self.assertTrue(h.BRepCheck_Analyzer(s).IsValid(),n)
if __name__=='__main__':unittest.main()
