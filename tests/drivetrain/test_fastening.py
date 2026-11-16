import importlib.util,pathlib,json,unittest,copy
ROOT=pathlib.Path(__file__).resolve().parents[2];s=importlib.util.spec_from_file_location('fast',ROOT/'cad/drivetrain/build_fastening.py');m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
class Tests(unittest.TestCase):
 def setUp(self):self.c=json.loads((ROOT/'cad/drivetrain/fastening.json').read_text())
 def test_parts(self):self.assertEqual(len(m.build(self.c)),46)
 def test_changed_fixed_dimension(self):
  self.c['tubeWallMm']=1;self.assertRaises(ValueError,m.build,self.c)
 def test_bad_dimension(self):
  self.c['plateThicknessMm']=float('nan');self.assertRaises(ValueError,m.build,self.c)
 def test_sourced_holes(self):
  for pair in self.c['railBoltXmm']:
   for x in pair:
    for y in [-260.35,260.35]:self.assertTrue(any(abs(h['xMm']-x)<1e-6 and abs(h['yMm']-y)<1e-6 for h in self.c['holes']))
 def test_stack(self):
  self.assertAlmostEqual(19.05-4-3.175-self.c['nutThicknessMm'],5.921875);self.assertAlmostEqual(69.85-50.8-4-self.c['nutThicknessMm'],9.096875)
 def test_motor_clearance(self):
  motor=json.loads((ROOT/'cad/drivetrain/candidate.json').read_text())['parts'];motor=[p for p in motor if p['role']=='motor'];worst=0
  for name,shape,role in m.build(self.c):
   for p in motor:
    b=p['boundsMm'];other=m.box(*b[:3],*p['sizeMm']);worst=max(worst,m.volume(m.BRepAlgoAPI_Common(shape,other).Shape()))
  self.assertLess(worst,1e-5)
if __name__=='__main__':unittest.main()
