import unittest,pathlib,sys
R=pathlib.Path(__file__).resolve().parents[1];sys.path[:0]=[str(R/'cad'),str(R/'sim/acquisition')]
import export_obstacles as e
import a2_primitives as h
class CoverageTest(unittest.TestCase):
 def test_complete_cover_and_missing_material_control(self):
  s=h.box(0,0,0,2,2,2);e.verify_cover(s,[[-.1,-.1,-.1,2.1,2.1,2.1]],h)
  with self.assertRaises(ValueError):e.verify_cover(s,[[0,0,0,1,2,2]],h)
if __name__=='__main__':unittest.main()
