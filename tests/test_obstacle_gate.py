import unittest,pathlib,sys,math
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/acquisition'))
from obstacle_gate import torus_box_distance,torus_vertical_cylinder_distance
class BoxGateTest(unittest.TestCase):
 def test_finite_cylinder_matches_dense_ring_samples(self):
  for cx,cy,z in [(0,0,0),(20,4,0),(10,3,7),(3,-2,-8)]:
   b=[13,-3,-2,19,3,2];v=torus_vertical_cylinder_distance([cx,cy,z],10,b);samples=[]
   for i in range(14400):
    a=i*2*math.pi/14400;x=cx+10*math.cos(a);y=cy+10*math.sin(a)
    samples.append(math.hypot(max(math.hypot(x-16,y)-3,0),max(-2-z,0,z-2)))
   self.assertLessEqual(v,min(samples)+1e-9);self.assertLess(min(samples)-v,1e-5)
 def test_hole_is_not_filled(self):self.assertAlmostEqual(torus_box_distance([0,0,0],10,[-1,-1,-1,1,1,1]),10-math.sqrt(2))
 def test_ring_intersection(self):self.assertEqual(torus_box_distance([0,0,0],10,[9,-1,-1,11,1,1]),0)
 def test_vertical_separation(self):self.assertEqual(torus_box_distance([0,0,0],10,[9,-1,3,11,1,4]),3)
 def test_external_rectangle(self):self.assertAlmostEqual(torus_box_distance([0,0,0],10,[20,0,0,21,1,1]),10)
 def test_sampling_lower_bound(self):
  b=[3,4,2,6,8,5];actual=torus_box_distance([0,0,0],10,b)
  values=[]
  for i in range(720):
   x,y=10*math.cos(i*math.pi/360),10*math.sin(i*math.pi/360)
   values.append(math.sqrt(max(3-x,0,x-6)**2+max(4-y,0,y-8)**2+2**2))
  self.assertLessEqual(actual,min(values)+1e-9);self.assertLess(min(values)-actual,.001)
if __name__=='__main__':unittest.main()
