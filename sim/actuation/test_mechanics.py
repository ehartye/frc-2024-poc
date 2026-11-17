import unittest,pathlib,json,math,copy,hashlib,random
from mechanics import ActuatorMechanics,ActuatorDomainError
R=pathlib.Path(__file__).resolve().parents[2];C=json.loads((R/'sim/actuation/config.json').read_text())
def model():return ActuatorMechanics(C,dict(timeS=0.,q=[0.,.3,.2],qd=[0.,0.,0.]))
def solve(a,b):
 a=[list(row)+[value] for row,value in zip(a,b)]
 for i in range(len(b)):
  pivot=a[i][i];a[i]=[x/pivot for x in a[i]]
  for j in range(len(b)):
   if i!=j:
    factor=a[j][i];a[j]=[x-factor*y for x,y in zip(a[j],a[i])]
 return [row[-1] for row in a]
class Mechanics(unittest.TestCase):
 def test_geometry_pins_and_candidate_ratios(self):
  p=R/'cad/assembly/actuator-geometry-contract.json';g=json.loads(p.read_text());self.assertEqual(hashlib.sha256(p.read_bytes()).hexdigest(),C['geometryContractSha256'])
  for name,pin in g['inputs'].items():self.assertEqual(hashlib.sha256((R/name).read_bytes()).hexdigest(),pin)
  self.assertEqual(C['liftGearRatio'],g['lift']['gearRatioHypothesis']);self.assertEqual(C['liftRadiusM'],g['lift']['nominalPitchRadiusM']);self.assertEqual(C['pitchGearRatio'],g['pitch']['gearRatioHypothesis']*g['pitch']['chainRatio'])
 def test_full_mass_symmetry_positive_and_coupling(self):
  p=model()
  for theta in [-2,-1,0,1,2]:
   m=p.terms([0,.3,theta],[0,0,0])['massMatrix'];self.assertEqual(m[0][2],m[2][0]);self.assertEqual(m[1][2],m[2][1]);self.assertGreater(m[0][0],0);self.assertGreater(m[0][0]*m[1][1],0)
   determinant=m[0][0]*(m[1][1]*m[2][2]-m[1][2]**2)-m[1][1]*m[0][2]**2;self.assertGreater(determinant,0)
  self.assertEqual(p.terms([0,0,0],[0,0,0])['massMatrix'][0][2],-C['rotatingMassKg']*C['comLocalXZ_M'][1])
 def test_full_moving_base_work_identity(self):
  p=model();rng=random.Random(192)
  for _ in range(30):
   q=[rng.random(),.3,rng.uniform(-1,1)];v=[rng.uniform(-1,1) for _ in range(3)];ext=[3.,-2.,.5];g=p.generalized(12.,(.1,-.1),('drive','drive'),q,v,ext);acc=solve(g['massMatrix'],g['rhs']);eps=1e-6
   plus=p.energy([x+eps*y for x,y in zip(q,v)],[x+eps*y for x,y in zip(v,acc)]);minus=p.energy([x-eps*y for x,y in zip(q,v)],[x-eps*y for x,y in zip(v,acc)])
   power=sum((a+b)*w for a,b,w in zip(g['electrical']['generalizedMotorForce'],ext,v));self.assertAlmostEqual((plus-minus)/(2*eps),power,places=6)
 def test_fixed_base_not_silent_reset(self):
  p=model();p.state['qd'][0]=.2;before=copy.deepcopy(p.state)
  with self.assertRaises(ActuatorDomainError):p.step_fixed(.001,12.,(.1,.1),('drive','drive'))
  self.assertEqual(p.state,before)
 def test_gravity_and_motor_force_signs(self):
  p=model();a=p.preview_fixed(.0001,12.,(0,0),('coast','coast'));self.assertLess(a['midpoint']['acceleration'][1],0)
  e=p.electrical(12.,(.1,.1),('drive','drive'),p.state['q'],p.state['qd']);self.assertGreater(e['generalizedMotorForce'][1],0);self.assertGreater(e['generalizedMotorForce'][2],0)
 def test_reflection_once_and_work(self):
  p=model();t=p.terms([0,0,0],[0,1,0]);expected=C['carriageMassKg']+C['rotatingMassKg']+C['liftRotorInertiaKgM2']*(C['liftGearRatio']/C['liftRadiusM'])**2;self.assertAlmostEqual(t['massMatrix'][1][1],expected)
  self.assertAlmostEqual(t['rotorOmegaRadps'][0],C['liftGearRatio']/C['liftRadiusM'])
 def test_port_purity_and_step_power(self):
  p=model();before=copy.deepcopy(p.state);port=p.power_port_fixed(.001,(.1,.1),('drive','drive'),(3.,1.,.1));e=port(11.);self.assertEqual(before,p.state);r=p.step_fixed(.001,11.,(.1,.1),('drive','drive'),(3.,1.,.1));self.assertEqual(e['busPowerW'],r['electrical']['busPowerW']);self.assertEqual(r['energyJ']['fixedConstraintWork'],0)
 def test_coast_brake_regeneration_and_voltage_domain(self):
  p=model();p.state['qd']=[0.,.05,.3];e=p.electrical(12.,(.01,.001),('drive','drive'),p.state['q'],p.state['qd']);self.assertLess(e['busPowerW'],0)
  r=p.step_fixed(.0001,12.,(0,0),('coast','brake'));self.assertNotEqual(r['state']['qd'],[0,0,0]);self.assertEqual(r['electrical']['busPowerW'],0)
  p.state['qd'][1]=3.;before=copy.deepcopy(p.state)
  with self.assertRaises(ActuatorDomainError):p.step_fixed(.0001,1.,(.1,.1),('drive','drive'))
  self.assertEqual(before,p.state)
 def test_energy_refinement(self):
  values=[]
  for dt in [.001,.0005,.00025]:
   p=model()
   for _ in range(round(.05/dt)):r=p.step_fixed(dt,12.,(.04,.01),('drive','drive'),(0.,-1.,.1))
   values.append(r)
  self.assertLess(abs(values[1]['energyJ']['residual']),abs(values[0]['energyJ']['residual']));self.assertLess(abs(values[2]['energyJ']['residual']),abs(values[1]['energyJ']['residual']))
  self.assertLess(abs(values[0]['energyJ']['residual']),.001)
 def test_evidence_inventory_and_motor_snapshot(self):
  import run as runner
  r=json.loads((R/'sim/actuation/evidence.json').read_text());g=json.loads((R/'cad/assembly/actuator-geometry-contract.json').read_text())
  self.assertEqual(set(r['inputs']),set(g['inputs'])|set(runner.FILES))
  for name,pin in r['inputs'].items():self.assertEqual(hashlib.sha256((R/name).read_bytes()).hexdigest(),pin)
  law=C['motorLawProvenance'];self.assertEqual(hashlib.sha256((R/law['snapshot']).read_bytes()).hexdigest(),law['sha256'])
  for case in r['cases']:self.assertLess(case['peakResidualJ'],.001);self.assertTrue(case['final']['fixedBase']);self.assertFalse(case['final']['acceptance'])
 def test_bad_dt_no_mutation(self):
  p=model();before=copy.deepcopy(p.state)
  for dt in [True,float('nan'),0.,.01]:
   with self.assertRaises(ValueError):p.step_fixed(dt,12.,(.1,.1),('drive','drive'))
  self.assertEqual(before,p.state)
if __name__=='__main__':unittest.main()
