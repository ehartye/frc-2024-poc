import unittest,pathlib,sys,copy
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/acquisition'))
import free_planar as f
C=dict(stepS=.0001,horizonS=.05,kvRadpsPerV=50.39638215133627,motorResistanceOhm=.11428571428571428,sourceV=12.,sourceResistanceOhm=.02,currentLimitA=40.,radiusM=.012127606663602424,slipRegularizationMps=.01,pieceMassKg=.235,yawInertiaKgM2=.235*(.1524**2+.75*.0254**2),rotorInertiaKgM2=.00003,normalStiffnessNpm=2000.,compressionAlphaSpm=2.,beltMu=.6,groundMu=.4,groundRingRadiusM=.1524)
def empty(q):return dict(status='SUPPORTED',contacts=[])
def walls(q):
 contacts=[]
 for side in [-1,1]:
  delta=max(0.,side*q[1]+.1778-.1769223933363976)
  contacts.append(dict(motor=0 if side<0 else 1,penetrationM=delta,normal=[0.,-side],tangent=[-1.,0.],leverM=[0.,side*.1778]))
 return dict(status='SUPPORTED',contacts=contacts)
class PlanarTest(unittest.TestCase):
 def test_zero_force_free_motion_and_spin(self):
  c=dict(C,groundMu=0.);r=f.simulate(c,empty,dict(p=[0.,.002],v=[.1,.03],spin=2.))
  self.assertAlmostEqual(r['final']['p'][0],.005,places=10);self.assertAlmostEqual(r['final']['p'][1],.0035,places=10);self.assertAlmostEqual(r['final']['spin'],2.,places=10);self.assertLess(r['maxAbsEnergyResidualJ'],1e-12)
 def test_ground_friction_dissipates_translation_and_spin(self):
  r=f.simulate(C,empty,dict(p=[0.,0.],v=[.2,.1],spin=2.))
  self.assertLess(r['final']['energyJ']['kinetic'],r['final']['energyJ']['initial']);self.assertGreater(r['final']['energyJ']['groundHeat'],0);self.assertLess(r['maxAbsEnergyResidualJ'],1e-5)
 def test_symmetry_and_lateral_response(self):
  plus=f.simulate(C,walls,dict(p=[0.,.004]),duties=(.12,.12));minus=f.simulate(C,walls,dict(p=[0.,-.004]),duties=(.12,.12))
  self.assertAlmostEqual(plus['final']['p'][1],-minus['final']['p'][1],places=10);self.assertLess(abs(plus['final']['p'][1]),.004);self.assertAlmostEqual(plus['final']['spin'],-minus['final']['spin'],places=10)
 def test_half_step_energy_and_trajectory(self):
  a=f.simulate(C,walls,dict(p=[0.,.004]),duties=(.12,.12));b=f.simulate(dict(C,stepS=C['stepS']/2),walls,dict(p=[0.,.004]),duties=(.12,.12))
  self.assertLess(abs(a['final']['p'][1]-b['final']['p'][1]),1e-5);self.assertLess(b['maxAbsEnergyResidualJ'],a['maxAbsEnergyResidualJ']);self.assertLess(a['maxAbsEnergyResidualJ'],.002)
 def test_unknown_geometry_stops_without_advancing_invalid_state(self):
  def finite(q):return empty(q) if q[0]<.001 else dict(status='UNSUPPORTED_HARD_CONTACT_DOMAIN',contacts=[])
  r=f.simulate(dict(C,groundMu=0.),finite,dict(p=[0.,0.],v=[.1,0.]))
  self.assertTrue(r['outcome'].startswith('UNKNOWN_DOMAIN'));self.assertLess(r['final']['p'][0],.001)
 def test_zero_motor_demand_does_not_reset_momentum(self):
  r=f.simulate(dict(C,groundMu=0.),empty,dict(p=[0.,0.],w=[10.,10.]),duties=(0.,0.));self.assertEqual(r['final']['w'],[10.,10.]);self.assertEqual(r['final']['energyJ']['electricSource'],0.)
 def test_zero_contact_stiffness_and_elastic_energy(self):
  zero=f.simulate(dict(C,normalStiffnessNpm=0.,groundMu=0.),walls,dict(p=[0.,.004]))
  self.assertEqual(zero['final']['p'],[0.,.004]);self.assertEqual(zero['final']['energyJ']['kinetic'],0.)
  a=f.simulate(dict(C,compressionAlphaSpm=0.,groundMu=0.,beltMu=0.),walls,dict(p=[0.,.004]))
  b=f.simulate(dict(C,stepS=C['stepS']/2,compressionAlphaSpm=0.,groundMu=0.,beltMu=0.),walls,dict(p=[0.,.004]))
  self.assertLess(b['maxAbsEnergyResidualJ'],a['maxAbsEnergyResidualJ']);self.assertLess(a['maxAbsEnergyResidualJ'],1e-6)
 def test_bad_inputs(self):
  for v in [True,float('nan'),float('inf'),-1]:
   with self.assertRaises(ValueError):f.simulate(dict(C,normalStiffnessNpm=v),empty,dict(p=[0.,0.]))
if __name__=='__main__':unittest.main()
