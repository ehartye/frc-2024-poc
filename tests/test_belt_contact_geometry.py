import unittest,pathlib,json,sys,math
ROOT=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'cad/assembly'))
import belt_contact_geometry as q
class BeltGeometryTest(unittest.TestCase):
 @classmethod
 def setUpClass(cls):cls.c=json.loads((ROOT/'cad/assembly/intake_components.json').read_text());cls.g=q.geometry(cls.c)
 def test_finite_straight_face_and_direction(self):
  for side in [-1,1]:
   p=q.inner_face_at(0,0,side,self.g);self.assertAlmostEqual(p['pointCradleMm'][1],side*self.c['pinch']['innerFaceYAbs']);self.assertEqual(p['acquisitionUnitTangent'],[-1.0,0.0,0]);self.assertEqual(p['outwardNormal'],[0.0,-side,0])
 def test_no_infinite_plane_extension(self):
  self.assertIsNone(q.inner_face_at(self.g['frontX']+self.g['outerRadius']+1,0,1,self.g));self.assertIsNone(q.inner_face_at(0,7,1,self.g))
  p=q.inner_face_at(self.g['frontX']+self.g['outerRadius']/2,0,1,self.g);self.assertEqual(p['feature'],'end-wrap');self.assertAlmostEqual(p['outwardNormal'][0],.5);self.assertAlmostEqual(sum(v*v for v in p['outwardNormal']),1)
 def test_material_cavity_and_height(self):
  face=self.c['pinch']['innerFaceYAbs'];self.assertAlmostEqual(q.query([0,face+1,0],1,self.g)['signedDistanceToMaterialMm'],-1)
  self.assertGreater(q.query([0,self.g['axisYAbs'],0],1,self.g)['signedDistanceToMaterialMm'],0)
  self.assertAlmostEqual(q.query([0,face+1,8],1,self.g)['signedDistanceToMaterialMm'],1.65)
 def test_invalids_rejected(self):
  with self.assertRaises(ValueError):q.query([float('nan'),0,0],1,self.g)
 def test_coplanar_analytic_matches_dense_ring(self):
  R=152.4;r=25.4
  for center in [[0,0,0],[160,5,0],[180,-5,0],[220,0,0]]:
   for side in [-1,1]:
    result=q.coplanar_note_contact(center,R,r,side,self.g);self.assertTrue(result['supported'])
    sampled=min(q.query([center[0]+R*math.cos(j*math.pi/720),center[1]+R*math.sin(j*math.pi/720),0],side,self.g)['signedDistanceToMaterialMm'] for j in range(1440))
    exact=r-result['penetrationMm'];self.assertGreaterEqual(sampled+1e-9,exact);self.assertLess(sampled-exact,.01)
 def test_coplanar_domain_fails_closed(self):
  self.assertFalse(q.coplanar_note_contact([0,0,1],152.4,25.4,1,self.g)['supported'])
  self.assertFalse(q.coplanar_note_contact([0,100,0],152.4,25.4,1,self.g)['supported'])
if __name__=='__main__':unittest.main()
