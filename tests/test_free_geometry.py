import unittest,pathlib,sys,json,math
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/acquisition'))
from free_geometry import make_geometry
class GeometryAdapterTest(unittest.TestCase):
 def setUp(self):self.c=json.loads((R/'cad/assembly/intake_components.json').read_text())
 def test_gate_required(self):
  with self.assertRaises(ValueError):make_geometry(self.c,None)
 def test_gate_not_bypassed(self):
  g=make_geometry(self.c,lambda *a:dict(status='UNKNOWN_PULLEY_CANDIDATE'))
  self.assertEqual(g([.178,.005])['status'],'UNKNOWN_PULLEY_CANDIDATE')
 def test_finite_faces_and_units_synthetic_gate_only(self):
  g=make_geometry(self.c,lambda *a:dict(status='SUPPORTED')) # test geometry only, not physical clearance
  self.assertEqual(g([.658,0.])['contacts'],[])
  r=g([.25,0.]);self.assertEqual(len(r['contacts']),2)
  for c in r['contacts']:
   self.assertAlmostEqual(c['penetrationM'],(.1778-.1769223933363976),places=12)
   self.assertEqual(c['tangent'],[-1.,0.]);self.assertAlmostEqual(abs(c['leverM'][1]),.1769223933363976,places=12)
 def test_end_wrap_is_not_infinite_plane(self):
  g=make_geometry(self.c,lambda *a:dict(status='SUPPORTED'));self.assertEqual(g([.5,.005])['contacts'],[])
  r=g([.36,.005]);self.assertTrue(r['contacts']);self.assertGreater(r['contacts'][0]['normal'][0],0.)
if __name__=='__main__':unittest.main()
