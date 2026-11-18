import pathlib,tempfile,unittest
from fusion_visual_provenance import LOCAL,EXTERNAL,inventory,verify_inventory
class ProvenanceTests(unittest.TestCase):
 def fixture(self,p):
  here=p/'audit';root=p/'robot'
  for base,names in [(here,LOCAL),(root,EXTERNAL)]:
   for name in names:
    f=base/name;f.parent.mkdir(parents=True,exist_ok=True);f.write_text(name)
  return here,root
 def test_explicit_dependencies(self):
  self.assertIn('native_drive_interactive_v3.py',LOCAL)
  self.assertEqual(set(EXTERNAL),{'sim/drivetrain/config.json','sim/drivetrain/plant.py','cad/drivetrain/candidate.json'})
 def test_missing_and_stale_rejected(self):
  with tempfile.TemporaryDirectory() as td:
   h,r=self.fixture(pathlib.Path(td));pins=inventory(h,r);self.assertTrue(verify_inventory(pins,h,r))
   for key in list(pins):
    incomplete=dict(pins);incomplete.pop(key)
    with self.assertRaises(ValueError):verify_inventory(incomplete,h,r)
   for key in ['robot/sim/drivetrain/config.json','robot/sim/drivetrain/plant.py','robot/cad/drivetrain/candidate.json','audit/native_drive_interactive_v3.py']:
    stale=dict(pins);stale[key]='0'*64
    with self.assertRaises(ValueError):verify_inventory(stale,h,r)
 def test_midrun_source_change(self):
  with tempfile.TemporaryDirectory() as td:
   h,r=self.fixture(pathlib.Path(td));pins=inventory(h,r);(r/'sim/drivetrain/config.json').write_text('mutated')
   with self.assertRaises(ValueError):verify_inventory(pins,h,r)
if __name__=='__main__':unittest.main()
