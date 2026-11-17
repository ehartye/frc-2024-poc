import unittest,pathlib,sys,json
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/pressure'))
import reconstruct_policy_boundary as b
class BoundaryTest(unittest.TestCase):
 def test_nominal_rejected_query(self):
  p=R/'sim/pressure/policy-evidence/nominal.json';r=b.reconstruct(p);self.assertTrue(r['atomicStateUnchanged']);self.assertEqual(r['reason'],'OUTSIDE_BOUNDED_PLANAR_EXPERIMENT');self.assertLess(r['rejectedGeometryQuery']['relativeRobotXY'][0],.178);self.assertEqual(r['rejectedGeometryQuery']['status'],r['reason'])
if __name__=='__main__':unittest.main()
