import unittest,pathlib,sys,json,math,copy
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/pressure'))
import finite_transport as t
import finite_area as a
class TransportTest(unittest.TestCase):
 @classmethod
 def setUpClass(cls):cls.c=json.loads((R/'cad/assembly/intake_components.json').read_text());cls.radius=cls.c['pinch']['teeth']*cls.c['pinch']['pitch']/1000/(2*math.pi)
 def test_straight_reference_unchanged(self):
  args=dict(note_center=[.010567440020061214,0,0],resolution=(128,128));p=t.prepare(self.c,**args);q=a.evaluate(self.c,**args);r=t.rates(p,pitch_radius_m=self.radius,mu=0.)
  self.assertEqual(p['potentialJ'],q['potentialJ']);self.assertEqual(r['noteForceWorldN'],q['noteForceWorldN']);self.assertEqual(r['noteMomentWorldNm'],q['noteMomentWorldNm'])
 def test_wrap_gradient_and_tangent(self):
  g=a.belt.geometry(self.c);dx=-g['outerRadius']*.5;rad=g['outerRadius']-.2;p=[(g['rearX']+dx)/1000,(g['axisYAbs']-math.sqrt(rad*rad-dx*dx))/1000,0.];q=t.gradient(p,1,g)
  self.assertEqual(q['feature'],'outer-wrap')
  for j in range(3):
   h=1e-8;u=p.copy();v=p.copy();u[j]+=h;v[j]-=h;fd=(a.belt.query(a.scale(u,1000),1,g)['signedDistanceToMaterialMm']-a.belt.query(a.scale(v,1000),1,g)['signedDistanceToMaterialMm'])/1000/(2*h);self.assertAlmostEqual(fd,q['normal'][j],places=7)
  tangent=a.belt.query(a.scale(p,1000),1,g)['outerSidewall']['acquisitionUnitTangent'];self.assertAlmostEqual(a.dot(tangent,q['normal']),0.,places=12)
 def test_wrap_material_quadrature_and_power(self):
  p=t.prepare(self.c,note_center=[.158,0,0],resolution=(512,512));self.assertGreater(p['features'].get('outer-wrap',0),0)
  r=t.rates(p,pitch_radius_m=self.radius,rotor_omega=[30.,30.]);self.assertLess(r['noteForceWorldN'][0],0.);self.assertLess(r['rotorPowerW'],0.);self.assertGreater(r['slipHeatW'],0.);self.assertAlmostEqual(r['workResidualW'],0.,places=11)
 def test_pure_moving_body_cradle_u_and_rotors(self):
  p=t.prepare(self.c,note_center=[.0105,0,.0003],resolution=(128,128),u_positive_m=.00002);before=copy.deepcopy(p)
  for w in [[20.,20.],[-10.,4.],[0.,0.]]:
   r=t.rates(p,pitch_radius_m=self.radius,rotor_omega=w,note_velocity=[.1,.02,-.03],note_omega=[.02,.01,-.04],cradle_velocity=[-.02,.01,.03],cradle_omega=[-.02,.02,.01],u_velocity=.003)
   self.assertAlmostEqual(r['workResidualW'],0.,places=11);self.assertGreaterEqual(r['slipHeatW'],0.)
  self.assertEqual(p,before)
 def test_zero_mu_and_zero_slip(self):
  p=t.prepare(self.c,note_center=[.0105,0,0],resolution=(128,128))
  for kw in [dict(mu=0.,rotor_omega=[20.,20.]),dict(mu=.6,rotor_omega=[0.,0.])]:
   r=t.rates(p,pitch_radius_m=self.radius,**kw);self.assertEqual(r['rotorContactTorqueNm'],[0.,0.]);self.assertEqual(r['slipHeatW'],0.)
 def test_deep_and_tie_stay_unknown(self):
  g=a.belt.geometry(self.c);y=(g['axisYAbs']-g['innerRadius']-.1)/1000
  with self.assertRaises(a.DomainError):t.gradient([.03,y,0],1,g)
  y=(g['axisYAbs']-g['outerRadius']+.2)/1000;z=(g['halfHeight']-.2)/1000
  with self.assertRaises(a.DomainError):t.gradient([.03,y,z],1,g)
 def test_derived_overflow_is_rejected(self):
  p=t.prepare(self.c,note_center=[.0105,0,0],resolution=(128,128))
  with self.assertRaises(a.DomainError):t.rates(p,pitch_radius_m=self.radius,note_velocity=[1.7e308,1.7e308,1.7e308])
 def test_invalid_electromechanical_inputs(self):
  p=t.prepare(self.c,note_center=[-1.,0,0],resolution=(16,16))
  for v in [True,float('inf'),float('nan'),-1.,0.]:
   with self.assertRaises(ValueError):t.rates(p,pitch_radius_m=v)
if __name__=='__main__':unittest.main()
