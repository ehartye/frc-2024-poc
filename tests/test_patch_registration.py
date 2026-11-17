import pathlib,sys,unittest,copy,math,json,hashlib
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/integrated'))
from patch_registration import evaluate,rotation,matvec,add
class Registration(unittest.TestCase):
 def setUp(self):
  self.points=[dict(motor=m,cradlePointM=[x,side*.1769223933363976,z],normalCradle=[0.,-side,0.]) for m,side in enumerate([-1,1]) for x in [-.012,-.006,0.,.006,.012] for z in [-.003,0.,.003]]
 def query(self,origin=[0.,0.,0.],cb=None,center=[0.,0.,0.],nb=None,points=None):return evaluate(self.points if points is None else points,origin,rotation(0.,0.) if cb is None else cb,center,rotation(0.,0.) if nb is None else nb,.1524,.0254)
 def test_rigid_covariance_not_attachment(self):
  a=self.query();b=self.query([1.,2.,3.],rotation(.4,.7),[1.,2.,3.],rotation(.4,.7));self.assertTrue(a['allNominalWitnessesInside']);self.assertTrue(b['allNominalWitnessesInside'])
  for x,y in zip(a['points'],b['points']):self.assertAlmostEqual(x['torusImplicitM2'],y['torusImplicitM2'],places=13)
 def test_actual_slip_or_pitch_can_lose_patch(self):
  self.assertFalse(self.query(center=[.05,0.,0.])['allNominalWitnessesInside']);self.assertFalse(self.query(cb=rotation(0.,math.pi/2))['allNominalWitnessesInside'])
 def test_reflections_bad_vectors_and_normals_rejected(self):
  with self.assertRaises(ValueError):self.query(cb=[[1.,0.,0.],[0.,-1.,0.],[0.,0.,1.]])
  p=copy.deepcopy(self.points);p[0]['normalCradle']=[0.,0.,0.]
  with self.assertRaises(ValueError):self.query(points=p)
  p=copy.deepcopy(self.points);p[0]['cradlePointM']=[True,0.,0.]
  with self.assertRaises(ValueError):self.query(points=p)
 def test_exact_index_registration_and_source_identity(self):
  r=json.loads((R/'sim/integrated/evidence/indexed-patch-registration.json').read_text())
  for p,h in r['inputs'].items():self.assertEqual(hashlib.sha256((R/p).read_bytes()).hexdigest(),h,p)
  f=json.loads((R/'sim/integrated/evidence/fast-regulated-index.json').read_text())['final'];self.assertEqual(r['preservedState'],dict(chassis=f['chassis'],note=f['note']))
  self.assertEqual(len(r['cradlePoints']),30);self.assertTrue(r['evaluation']['allNominalWitnessesInside']);self.assertFalse(r['acceptance'])
 def test_native_torus_material_at_exact_registered_points(self):
  from OCP.BRepPrimAPI import BRepPrimAPI_MakeTorus
  from OCP.BRepClass3d import BRepClass3d_SolidClassifier
  from OCP.gp import gp_Ax2,gp_Pnt,gp_Dir
  from OCP.TopAbs import TopAbs_IN
  r=json.loads((R/'sim/integrated/evidence/indexed-patch-registration.json').read_text());c=r['noteCenterWorldM'];m=r['noteBasis'];n=[m[i][2] for i in range(3)];shape=BRepPrimAPI_MakeTorus(gp_Ax2(gp_Pnt(*c),gp_Dir(*n)),.1524,.0254).Shape()
  for p in r['evaluation']['points']:self.assertEqual(BRepClass3d_SolidClassifier(shape,gp_Pnt(*p['pointWorldM']),1e-10).State(),TopAbs_IN)
if __name__=='__main__':unittest.main()
