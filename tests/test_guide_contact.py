import pathlib,sys,json,unittest,math
R=pathlib.Path(__file__).resolve().parents[1];sys.path[:0]=[str(R/'sim/integrated'),str(R/'cad/assembly')]
from guide_contact import wrench,cross,GuideDomainError
import finite_retention_guide as guide
class GuideForce(unittest.TestCase):
 def setUp(self):
  self.c=guide.load_contract(R);self.samples=json.loads((R/'cad/assembly/evidence/rounded-retention-guide.json').read_text())['samples']
 def query(self,angle=10):
  s=next(s for s in self.samples if s['pitchDeg']==angle);center=[v/1000 for v in s['noteCenterRobotMm']];q=guide.query_round_xz([center[0],center[2]],math.radians(angle),self.c);return center,q
 def call(self,q,center,v=[-.02,0.,.03],cv=[.01,0.,0.],**kw):return wrench(q,stiffness=2000.,damping=3.,mu=.2,slip_scale=.01,chassis_position=[0.,0.,0.],chassis_velocity=cv,heading=0.,yaw_rate=0.,piece_center=center,piece_point_velocity=v,**kw)
 def test_reciprocal_force_moment_and_energy_rate(self):
  center,q=self.query();r=self.call(q,center)
  for a,b in zip(r['forceWorldN'],r['chassisReactionWorldN']):self.assertEqual(a+b,0.)
  orbital=cross(center,r['forceWorldN'])
  for i in range(3):self.assertAlmostEqual(orbital[i]+r['pieceMomentWorldNm'][i]+r['chassisMomentWorldNm'][i],0.)
  potential_rate=2000*q['penetrationM']*r['penetrationRateMps']
  self.assertAlmostEqual(r['pieceWorkW']-r['guideWorkW']+potential_rate+r['normalHeatW']+r['frictionHeatW'],0.,places=12)
  self.assertGreaterEqual(r['normalHeatW'],0.);self.assertGreaterEqual(r['frictionHeatW'],0.);self.assertFalse(r['acceptance'])
 def test_common_velocity_boost_preserves_relative_dissipation(self):
  center,q=self.query(4);a=self.call(q,center);b=self.call(q,center,v=[.28,.4,-.17],cv=[.31,.4,-.2])
  for key in ['forceWorldN','normalHeatW','frictionHeatW']:
   if isinstance(a[key],list):
    for x,y in zip(a[key],b[key]):self.assertAlmostEqual(x,y)
   else:self.assertAlmostEqual(a[key],b[key])
  self.assertNotEqual(a['guideWorkW'],b['guideWorkW'])
 def test_normal_power_matches_geometric_jacobian(self):
  center,q=self.query(4);vel=[.013,0.,.023];omega=.17;lever=q['leverRobotXZm'];pointv=[vel[0]-omega*lever[1],0.,vel[2]+omega*lever[0]];r=self.call(q,center,v=pointv,cv=[0.,0.,0.]);expected=sum(a*b for a,b in zip(q['penetrationJacobianXZPitch'],[vel[0],vel[2],omega]));self.assertAlmostEqual(r['penetrationRateMps'],expected)
 def test_unknown_rejected_and_separation_no_force(self):
  center,q=self.query(20)
  with self.assertRaises(GuideDomainError):self.call(q,center)
  center,q=self.query(0);self.assertEqual(self.call(q,center)['forceWorldN'],[0.,0.,0.]);self.assertEqual(self.call(q,center)['nativeMaterialStatus'],q['nativeMaterialStatus'])
if __name__=='__main__':unittest.main()
