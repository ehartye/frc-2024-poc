import importlib.util,json,pathlib,unittest,copy,math
ROOT=pathlib.Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('drive',ROOT/'cad/drivetrain/build_drivetrain.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
class ContractTests(unittest.TestCase):
 def setUp(self):self.c=json.loads((ROOT/'cad/drivetrain/candidate.json').read_text())
 def test_count(self):self.assertEqual(sum(p['role']=='wheel' for p in self.c['parts']),6);self.assertEqual(sum(p['role']=='motor' for p in self.c['parts']),4)
 def test_ratio(self):self.assertAlmostEqual(self.c['gearRatio'],75/7)
 def test_ground_drop(self):
  b=sorted(p['boundsMm'][2] for p in self.c['parts'] if p['role']=='wheel');self.assertAlmostEqual(b[0],0,places=5);self.assertAlmostEqual(b[2],1.778,places=5)
 def test_track(self):
  p=sorted([p for p in self.c['parts'] if p['role']=='wheel'],key=lambda x:abs(x['worldCenterMm'][0]))[:2];self.assertAlmostEqual(abs(p[1]['worldCenterMm'][1]-p[0]['worldCenterMm'][1]),self.c['centerTrackMm'],places=5)
 def test_nonfinite(self):
  for v in [True,float('nan'),float('inf'),'1']:
   c=copy.deepcopy(self.c);c['parts'][0]['boundsMm'][0]=v
   with self.assertRaises(ValueError):m.validate(c)
 def test_duplicate(self):
  self.c['parts'].append(self.c['parts'][0]);self.assertRaises(ValueError,m.validate,self.c)
 def test_mass_partial(self):self.assertAlmostEqual(sum(p['massKg'] or 0 for p in self.c['parts']),5.080234544);self.assertTrue(any(p['massKg'] is None for p in self.c['parts']))
 def test_native(self):self.assertEqual(len(m.build(self.c)),90);self.assertEqual(len(m.build(self.c,True)),98)
if __name__=='__main__':unittest.main()
