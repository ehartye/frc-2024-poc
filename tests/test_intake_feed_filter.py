import unittest,sys,pathlib,math
ROOT=pathlib.Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'cad'),str(ROOT/'cad/assembly')]
from audit_intake_feed import possible_torus_box
class TorusFilterTest(unittest.TestCase):
 def test_preserves_surface_points_and_boxes(self):
  for advance in [0,110,220,480]:
   for i in range(32):
    theta=2*math.pi*i/32
    for j in range(16):
     phi=2*math.pi*j/16;r=152.4+25.4*math.cos(phi)
     p=[advance+r*math.cos(theta),r*math.sin(theta),25.4*math.sin(phi)]
     box=[x-.0001 for x in p]+[x+.0001 for x in p]
     self.assertTrue(possible_torus_box(box,advance,152.4,25.4))
 def test_rejects_hole_and_separated_boxes(self):
  for box in [[-1,-1,-1,1,1,1],[300,-1,-1,301,1,1],[150,-1,30,155,1,31]]:
   self.assertFalse(possible_torus_box(box,0,152.4,25.4))
 def test_keeps_wide_uncertain_boxes(self):
  self.assertTrue(possible_torus_box([-200,-200,-30,200,200,30],0,152.4,25.4))
if __name__=='__main__':unittest.main()
