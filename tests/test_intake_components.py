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
 def test_contact_classification_never_waives_unknown_joint(self):
  from audit_internal_motion import contact_kind
  a={'name':'side pinch belt -1','motion':'cradle'};b={'name':'12T track pulley representation -1','motion':'cradle'}
  self.assertEqual(contact_kind(a,b),'nominal-drive-envelope-engagement-unvalidated')
  b['motion']='carriage';self.assertEqual(contact_kind(a,b),'unresolved-cross-motion-intersection')
  b={'name':'unbored stock','motion':'cradle'};self.assertEqual(contact_kind(a,b),'unresolved-solid-intersection')
 def test_dense_motion_tracks_contract(self):
  from audit_dense_motion import dense_poses
  c=dict(self.c,pitchDeg=31);rows=list(dense_poses(c));self.assertEqual(max(r[1] for r in rows),31)
  rotate=[r[1] for r in rows if r[0]=='rotate'];self.assertLessEqual(max(b-a for a,b in zip(rotate,rotate[1:])),1)
  lift=[r[2] for r in rows if r[0]=='lift'];self.assertLessEqual(max(b-a for a,b in zip(lift,lift[1:])),5)
 def test_rotation_chain_closes(self):
  c=self.c['rotation'];self.assertAlmostEqual((c['chainLinks']-c['sprocketTeeth'])*6.35/2,c['centerDistance'])
 def test_side_weldments_are_connected_solids(self):
  from OCP.TopExp import TopExp_Explorer
  from OCP.TopAbs import TopAbs_SOLID
  for n,s,k in q.build():
   if 'side weldment' not in n:continue
   e=TopExp_Explorer(s,TopAbs_SOLID);count=0
   while e.More():count+=1;e.Next()
   self.assertEqual(count,1,n)
 def test_pivot_sprocket_bore_and_unbored_negative(self):
  from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
  from OCP.BRepGProp import BRepGProp
  from OCP.GProp import GProp_GProps
  parts={n:s for n,s,k in q.build()}
  shaft=parts['intake candidate cradle side pivot -1']
  sprocket=next(s for n,s in parts.items() if n.startswith('intake candidate rotation16T sprocket 25.4'))
  def volume(a,b):
   p=GProp_GProps();BRepGProp.VolumeProperties_s(BRepAlgoAPI_Common(a,b).Shape(),p);return p.Mass()
  self.assertLess(abs(volume(shaft,sprocket)),1e-5)
  c=self.c;solid=h.cyl((c['stageCenterX'],c['rotation']['chainPlaneY']-4.75,c['floorCenterZ']),(0,1,0),17.95,9.5)
  self.assertGreater(volume(shaft,solid),100)
 def test_shapes_valid_all_key_poses(self):
  for state in ['floor','rotated','handoff']:
   for n,s,k in q.build(state):self.assertTrue(h.BRepCheck_Analyzer(s).IsValid(),n)
if __name__=='__main__':unittest.main()
