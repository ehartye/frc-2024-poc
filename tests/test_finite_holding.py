import pathlib,sys,json,copy,math,unittest,hashlib
R=pathlib.Path(__file__).resolve().parents[1];sys.path[:0]=[str(R/'sim/holding'),str(R/'sim/integrated')]
from finite_motor_patch import FiniteMotorPatch,HoldingDomainError,rotate
from shared_supply import solve_supply
C=json.loads((R/'sim/holding/config.json').read_text())
def model(law='memoryless-tanh',w=(0.,0.),shear=None):
 c=dict(C,law=law,patchMotor=[0,1]);s=dict(timeS=0.,frameAngleRad=0.,rotorOmegaRadps=list(w))
 if law=='elastic-plastic':s['shearWorldM']=shear or [[0.,0.],[0.,0.]]
 return FiniteMotorPatch(c,s)
def args(dt=.001,bus=12.,d=(0.,0.),m=('coast','coast'),angle=0.,body=None,cradle=None,valid=None):
 return (dt,bus,d,m,angle,body or [[0.,0.],[0.,0.]],cradle or [[0.,0.],[0.,0.]],[[0.,0.],[0.,0.]],valid or [True,True])
class Holding(unittest.TestCase):
 def test_source_map_and_negative_weight_support(self):
  m=json.loads((R/'sim/holding/checkpoint-map.json').read_text());self.assertEqual(hashlib.sha256((R/m['source']).read_bytes()).hexdigest(),m['sourceSha256'])
  self.assertEqual(m['retentionRotorOmegaRadps'],[-w for w in m['sourceRotorOmegaRadps']]);self.assertLess(m['maximumTangentialForceN'],m['weightN']);self.assertFalse(m['finitePatchRegistered']);self.assertFalse(m['elasticHistoryAvailable'])
 def test_longitudinal_memoryless_law_matches_acquisition(self):
  m=json.loads((R/'sim/holding/checkpoint-map.json').read_text());a=json.loads((R/'sim/acquisition/free-planar-config.json').read_text())['model']
  slip=a['radiusM']*m['sourceRotorOmegaRadps'][0]+m['sourceLongitudinalNoteVelocity']-m['sourceLongitudinalChassisVelocity']
  expected=a['beltMu']*m['normalSideN'][0]*math.tanh(slip/a['slipRegularizationMps'])
  from coupled_acquisition import acquisition_rates
  normal=m['normalSideN'][0]
  geometry=lambda q:dict(status='SUPPORTED',contacts=[dict(motor=0,penetrationM=normal/a['normalStiffnessNpm'],normal=[0.,1.],tangent=[-1.,0.],leverM=[0.,0.])])
  n=dict(p=[0.,0.],v=[m['sourceLongitudinalNoteVelocity'],0.],spin=0.,w=m['sourceRotorOmegaRadps']);d=dict(x=0.,y=0.,heading=0.,vx=m['sourceLongitudinalChassisVelocity'],vy=0.,yaw_rate=0.)
  actual=acquisition_rates(n,d,a,geometry,m['sourceBusV'],[-x for x in m['retentionDuties']])
  self.assertAlmostEqual(expected,actual['contacts'][0]['tangentialN'],places=15)
  retained_slip=C['beltRadiusM']*m['retentionRotorOmegaRadps'][0]+m['sourceLongitudinalChassisVelocity']-m['sourceLongitudinalNoteVelocity'];self.assertAlmostEqual(retained_slip,-slip,places=15)
  self.assertAlmostEqual(C['mu']*C['normalSideN'][0]*math.tanh(retained_slip/C['slipRegularizationMps']),-expected,places=15)
 def test_pure_query_and_shared_supply(self):
  p=model();before=copy.deepcopy(p.state);port=p.power_port(.001,(.1,.1),('drive','drive'),0.,[[0.,0.]]*2,[[0.,0.]]*2,[[0.,0.]]*2,[True,True]);bus=solve_supply({'hold':port},12.,.02);q=p.query(*args(bus=bus['busV'],d=(.1,.1),m=('drive','drive')))
  self.assertEqual(before,p.state);self.assertEqual(q['busPowerW'],bus['ports']['hold']['busPowerW']);r=p.step(*args(bus=bus['busV'],d=(.1,.1),m=('drive','drive')));self.assertLess(abs(r['energyJ']['residual']),1e-9)
 def test_coast_finite_rotor_backdrive_and_energy(self):
  p=model();r=p.step(*args(body=[[.2,0.]]*2));self.assertGreater(r['nextState']['rotorOmegaRadps'][0],0);self.assertLess(r['bodyForceXZ_N'][0],0);self.assertEqual(r['busPowerW'],0);self.assertLess(abs(r['energyJ']['residual']),1e-10)
 def test_friction_circle_and_creep_not_ideal_hold(self):
  p=model();r=p.query(*args(body=[[.1,.1]]*2))
  for force,cap in zip(r['patches'],C['normalSideN']):self.assertLessEqual(math.hypot(*force['force']),C['mu']*cap+1e-12)
  rest=p.query(*args());self.assertEqual(rest['bodyForceXZ_N'],[0.,0.]);self.assertNotIn('shearWorldM',rest['nextState'])
 def test_objective_elastic_history(self):
  shear=[[0.,.0002],[0.,.0002]];a=model('elastic-plastic',shear=shear);b=model('elastic-plastic',shear=shear)
  qa=a.query(*args());qb=b.query(*args(angle=.7));expected=rotate(qa['bodyForceXZ_N'],.7)
  for x,y in zip(expected,qb['bodyForceXZ_N']):self.assertAlmostEqual(x,y,places=12)
  self.assertAlmostEqual(a.stored(qa['nextState']),b.stored(qb['nextState']),places=12)
 def test_elastic_plastic_energy_and_numeric_loss_named(self):
  p=model('elastic-plastic')
  for _ in range(20):r=p.step(*args(body=[[.1,.1]]*2))
  self.assertGreater(r['energyJ']['returnMapNumericalLoss'],0);self.assertGreater(r['energyJ']['plasticHeat'],0);self.assertLess(abs(r['energyJ']['residual']),1e-9)
 def test_regeneration_and_infeasible_domain(self):
  p=model(w=(100.,100.));r=p.query(*args(d=(.1,.1),m=('drive','drive')));self.assertLess(r['busPowerW'],0)
  p=model(w=(1000.,1000.));before=copy.deepcopy(p.state)
  with self.assertRaises(HoldingDomainError):p.step(*args(bus=1.,d=(.1,.1),m=('drive','drive')))
  self.assertEqual(p.state,before)
 def test_finite_validity_and_initial_history_required(self):
  p=model();before=copy.deepcopy(p.state)
  with self.assertRaises(HoldingDomainError):p.step(*args(valid=[True,False]))
  self.assertEqual(p.state,before)
  with self.assertRaises((KeyError,ValueError)):FiniteMotorPatch(dict(C,law='elastic-plastic'),dict(timeS=0.,frameAngleRad=0.,rotorOmegaRadps=[0.,0.]))
 def test_dt_sensitivity(self):
  out=[]
  for dt in [.001,.0005,.00025]:
   p=model()
   for _ in range(round(.05/dt)):r=p.step(*args(dt=dt,d=(.02,.02),m=('drive','drive'),body=[[.05,.02]]*2))
   self.assertLess(abs(r['energyJ']['residual']),1e-8);out.append(r['nextState']['rotorOmegaRadps'][0])
  self.assertLess(abs(out[1]-out[2]),abs(out[0]-out[1]))
 def test_fresh_receipt_source_set_and_numerical_sensitivity(self):
  import importlib.util
  spec=importlib.util.spec_from_file_location('holding_runner',R/'sim/holding/run.py');runner=importlib.util.module_from_spec(spec);spec.loader.exec_module(runner)
  report=json.loads((R/'sim/holding/evidence.json').read_text());source=json.loads((R/'sim/integrated/evidence/fast-regulated-index.json').read_text())
  expected=set(source['inputs'])|set(runner.FILES)|{'sim/integrated/evidence/fast-regulated-index.json'}
  self.assertEqual(set(report['inputs']),expected)
  for name,pin in report['inputs'].items():self.assertEqual(hashlib.sha256((R/name).read_bytes()).hexdigest(),pin)
  self.assertEqual(report['rotorEnergyBeforeJ'],report['rotorEnergyAfterJ']);self.assertFalse(report['staticBound']['necessaryConditionMet']);self.assertFalse(report['finitePatchRegistered'])
  for case in report['cases']:self.assertLess(case['peakResidualJ'],1e-10);self.assertTrue(case['synthetic'])
  for offset in [0,3]:
   a,b,c=[x['final']['nextState']['rotorOmegaRadps'][0] for x in report['cases'][offset:offset+3]]
   self.assertLess(abs(c-b),abs(b-a))
if __name__=='__main__':unittest.main()
