import unittest,pathlib,sys
ROOT=pathlib.Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'cad'),str(ROOT/'cad/assembly')]
import build_connected_candidate as q
class IdentifierTest(unittest.TestCase):
 def test_attachment_modules_reject_duplicate_ids(self):
  duplicate=[('duplicate',None,'support'),('duplicate',None,'support')]
  for call in [lambda:q.upper.lower.apply(duplicate),lambda:q.upper.apply(duplicate,25.4),lambda:q.pads.apply(duplicate,25.4)]:
   with self.assertRaisesRegex(ValueError,'duplicate'):call()
 def test_shooter_pulley_identifiers_distinguish_drive_members(self):
  parts=q.assembly.build('floor');names=[n for n,s,k in parts]
  self.assertEqual(len(names),len(set(names)))
  self.assertEqual(sum('driver pulley envelope' in n for n in names),4)
  self.assertEqual(sum('driven pulley envelope' in n for n in names),4)
if __name__=='__main__':unittest.main()
