import unittest,json,pathlib,sys,copy,importlib.util
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'cad'))
import a2_kinematics as k,build_a2_layout as cad
spec=importlib.util.spec_from_file_location('screen',R/'scripts/a2-intake-screen.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
class A2SweepRegression(unittest.TestCase):
 def setUp(self):self.c=json.loads((R/'cad/cycle-contract-A2.json').read_text())
 def test_all_moving_parts_all_sampled_poses(self):
  r=m.screen(self.c);self.assertFalse(r['movingComponentEnvelopeViolations']);self.assertFalse(r['unreachableTelescopePoses']);self.assertLessEqual(r['maxMovingComponentHeightM'],1.2192);self.assertAlmostEqual(r['maxMovingComponentHeightM'],1.2142)
 def test_previous_ninety_degree_height_failure_fixed(self):
  i=self.c['geometry']['intake'];p=k.pose(i,90);self.assertAlmostEqual(p['length'],.6642);b=k.moving_bounds(i,p);self.assertAlmostEqual(next(x['max'][2] for x in b if x['id']=='retention'),1.2142)
 def test_pivot_mutation_changes_screen(self):
  old=m.screen(self.c,1);self.c['geometry']['intake']['pivot'][0]=.4;new=m.screen(self.c,1);self.assertNotEqual(old,new);self.assertTrue(new['unreachableTelescopePoses'])
 def test_roller_mutation_is_not_silently_accepted(self):
  self.c['geometry']['intake']['rollerRadius']=.09;r=m.screen(self.c,1);self.assertTrue(r['movingComponentEnvelopeViolations'])
 def test_lateral_and_nonfinite_mutations(self):
  self.c['geometry']['intake']['armYOrigins'][0]=-.8;self.assertTrue(m.screen(self.c,1)['movingComponentEnvelopeViolations']);self.c['geometry']['intake']['pivot'][0]=float('nan');self.assertRaises(ValueError,k.pose,self.c['geometry']['intake'],0)
 def test_native_all_parts_at_critical_and_endpoint_poses(self):
  from OCP.Bnd import Bnd_Box
  from OCP.BRepBndLib import BRepBndLib
  for deg in [-40,0,90,110]:
   c=copy.deepcopy(self.c);c['geometry']['intake']['stowAngleDeg']=deg;c['geometry']['intake']['floorAngleDeg']=min(deg,c['geometry']['intake']['floorAngleDeg']);parts=cad.build(c,'stowed')
   for name,shape,kind in parts:
    b=Bnd_Box();BRepBndLib.Add_s(shape,b);lo,hi=b.CornerMin(),b.CornerMax();xmin,ymin,zmin=lo.X(),lo.Y(),lo.Z();xmax,ymax,zmax=hi.X(),hi.Y(),hi.Z();self.assertLessEqual(zmax,1219.2+1e-4,(deg,name,zmax));self.assertLessEqual(xmax,659.8+1e-4,(deg,name,xmax));self.assertGreaterEqual(zmin,-1e-4,(deg,name,zmin))
if __name__=='__main__':unittest.main()
