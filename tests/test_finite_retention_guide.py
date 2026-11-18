import unittest,pathlib,sys,json,math
ROOT=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'cad/assembly'))
import finite_retention_guide as g
class FiniteGuide(unittest.TestCase):
 def setUp(self):
  self.c=g.load_contract(ROOT);self.rows=json.loads((ROOT/'cad/assembly/evidence/rounded-retention-guide.json').read_text())['samples']
 def pose(self,deg):
  s=next(s for s in self.rows if s['pitchDeg']==deg);return [s['noteCenterRobotMm'][j]/1000 for j in [0,2]],math.radians(deg)
 def test_lower_round_and_flat_normals(self):
  for deg in [4,6,8,10,12,14,16,18]:
   x,p=self.pose(deg);q=g.query_round_xz(x,p,self.c);self.assertTrue(q['geometryDomainValid']);self.assertGreater(q['penetrationM'],0);self.assertAlmostEqual(sum(v*v for v in q['normalRobotXZ']),1)
  x,p=self.pose(4);self.assertLess(g.query_round_xz(x,p,self.c)['normalRobotXZ'][1],0)
  x,p=self.pose(10);q=g.query_round_xz(x,p,self.c);self.assertEqual(q['normalRobotXZ'],[-1,0]);self.assertGreater(q['positivePitchTorquePerNormalN'],0)
 def test_potential_derivative(self):
  for deg in [4,6,10,16]:
   x,p=self.pose(deg);q=g.query_round_xz(x,p,self.c);eps=1e-7
   for j in range(3):
    xp=x.copy();xm=x.copy();pp=p;pm=p
    if j<2:xp[j]+=eps;xm[j]-=eps
    else:pp+=eps;pm-=eps
    d=(g.query_round_xz(xp,pp,self.c)['penetrationM']-g.query_round_xz(xm,pm,self.c)['penetrationM'])/(2*eps)
    self.assertAlmostEqual(d,q['penetrationJacobianXZPitch'][j],places=6)
   self.assertAlmostEqual(q['positivePitchTorquePerNormalN'],-q['penetrationJacobianXZPitch'][2])
 def test_domains_and_unknown(self):
  for deg in [0,2,22,30]:
   x,p=self.pose(deg);self.assertEqual(g.query_round_xz(x,p,self.c)['status'],'SEPARATED_FROM_ENTIRE_GUIDE')
  x,p=self.pose(20);q=g.query_round_xz(x,p,self.c);self.assertEqual(q['status'],'UNKNOWN');self.assertFalse(q['geometryDomainValid'])
  x,p=self.pose(10);self.assertEqual(g.query_round_xz(x,p,self.c,y_m=.12)['status'],'UNKNOWN');self.assertEqual(g.query_round_xz(x,-.1,self.c)['status'],'UNKNOWN')
 def test_no_force_or_acceptance(self):
  x,p=self.pose(10);q=g.query_round_xz(x,p,self.c);self.assertIsNone(q['forceLaw']);self.assertFalse(q['adopted']);self.assertIn('UNKNOWN',q['nativeMaterialStatus']);self.assertFalse(q['pointIsPressureCentroid'])
if __name__=='__main__':unittest.main()
