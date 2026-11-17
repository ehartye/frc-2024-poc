import unittest,pathlib,sys,json,copy,math
R=pathlib.Path(__file__).resolve().parents[1];sys.path[:0]=[str(R/'sim/drivetrain'),str(R/'sim/integrated'),str(R/'sim/component-launcher')]
from plant import DrivePlant
from shared_supply import solve_supply,SupplyDomainError
from power_state import PowerState
C=json.loads((R/'sim/drivetrain/config.json').read_text());N=[C['massKg']*9.80665/2]*2

def run(demand=(.4,.4),dt=.0005,duration=.2,config=None):
 p=DrivePlant(config or C)
 for _ in range(round(duration/dt)):last=p.step(dt,demand,12.,N,modes=('drive','drive'))
 return p,last

class DrivePlantTest(unittest.TestCase):
 def test_electrical_current_limit_and_query_purity(self):
  p=DrivePlant(C);s=copy.deepcopy(p.state);e=p.electrical(12.,(.8,.8),modes=('drive','drive'))
  self.assertEqual(p.state,s);self.assertEqual([r['currentA'] for r in e['perMotor']],[40.]*4);self.assertAlmostEqual(e['busPowerW'],4*40**2*.091);self.assertAlmostEqual(e['perMotor'][0]['actualTerminalV'],40*.091)
 def test_regeneration_signed_and_supply_rejects_net_charge(self):
  p=DrivePlant(C);p.state['wheel']=[20.,20.];e=p.electrical(12.,(.1,.1),modes=('drive','drive'));self.assertLess(e['busPowerW'],0)
  with self.assertRaises(SupplyDomainError):solve_supply({'drive':lambda v:p.electrical(v,(.1,.1),modes=('drive','drive'))},12,.02)
 def test_coast_opens_current_but_preserves_motion(self):
  p,last=run();before=copy.deepcopy(p.state);row=p.step(.0005,(0.,0.),6.,N,modes=('coast','coast'))
  self.assertEqual(row['electrical']['busPowerW'],0.);self.assertGreater(row['state']['vx'],0);self.assertGreater(row['state']['wheel'][0],0);self.assertGreater(row['state']['t'],before['t']);self.assertGreater(row['state']['x'],before['x'])
 def test_forward_reverse_and_rotation_signs(self):
  p,a=run();self.assertGreater(a['state']['vx'],0);self.assertLess(abs(a['state']['y']),1e-8);self.assertLess(abs(a['state']['heading']),1e-8)
  _,b=run((-.4,-.4));self.assertLess(b['state']['vx'],0)
  _,r=run((-.4,.4));self.assertGreater(r['state']['yaw_rate'],0);self.assertLess(abs(r['state']['vx']),1e-8)
 def test_no_normal_load_means_no_magic_chassis_motion(self):
  p=DrivePlant(C)
  for _ in range(100):row=p.step(.0005,(.4,.4),12.,[0.,0.],modes=('drive','drive'))
  self.assertEqual(row['state']['vx'],0.);self.assertGreater(row['state']['wheel'][0],0.)
 def test_lateral_and_yaw_scrub_signs_and_friction_circle(self):
  p=DrivePlant(C);p.state['vy']=1.;r=p._rates(p.state,12,(0,0),('coast','coast'),N,(0,0),0)
  self.assertLess(r['contactForceBodyN'][1],0.)
  for a in r['contacts']:self.assertLessEqual(math.hypot(*a['forceBodyN']),C['tireMu']*a['normalN']+1e-9)
  p.state['vy']=0;p.state['yaw_rate']=1;r=p._rates(p.state,12,(0,0),('coast','coast'),N,(0,0),0);self.assertLess(r['contactYawTorqueNm'],0.)
 def test_reflected_rotor_energy_counted_once(self):
  p=DrivePlant(C);p.state['wheel']=[10.,10.];expected=100*(C['wheelSideInertiaKgM2']+2*C['motorRotorInertiaKgM2']*C['gearRatio']**2);self.assertAlmostEqual(p.kinetic(),expected)
 def test_external_work_reconciles(self):
  c=dict(C,motorViscousDragNmPerRadps=0.,gearViscousDragNmPerRadps=0.,tireMu=0.);p=DrivePlant(c)
  for _ in range(100):r=p.step(.001,(0,0),0,[0,0],modes=('coast','coast'),external_force_world=(10.,3.),external_yaw_torque=2.)
  self.assertGreater(r['energyJ']['externalWork'],0);self.assertLess(abs(r['energyJ']['residual']),1e-10)
 def test_energy_and_half_step(self):
  _,a=run(dt=.0005);_,b=run(dt=.00025)
  self.assertLess(abs(b['energyJ']['residual']),abs(a['energyJ']['residual']));self.assertLess(abs(a['energyJ']['residual']),.5);self.assertLess(abs(a['state']['x']-b['state']['x']),.001)
 def test_brake_dissipates_without_instant_state_reset(self):
  p,_=run();before=copy.deepcopy(p.state);r=p.step(.0005,(0,0),12,N,modes=('brake','brake'));self.assertEqual(r['electrical']['busPowerW'],0);self.assertGreater(r['energyJ']['windingHeat'],0);self.assertGreater(r['state']['vx'],0);self.assertLess(r['state']['wheel'][0],before['wheel'][0])
 def test_power_port_nonmutating_and_shared_load_same_voltage(self):
  p=DrivePlant(C);state=copy.deepcopy(p.state);port=p.power_port(.0005,(.4,.4),N,modes=('drive','drive'))
  a=solve_supply({'drive':port},12,.02);b=solve_supply({'drive':port,'mechanism':lambda v:{'busPowerW':v*v/2}},12,.02)
  self.assertEqual(p.state,state);self.assertLess(b['busV'],a['busV']);r=p.step(.0005,(.4,.4),b['busV'],N,modes=('drive','drive'))
  self.assertAlmostEqual(r['electrical']['busPowerW'],b['ports']['drive']['busPowerW']);self.assertLess(abs(b['powerResidualW']),1e-7)
 def test_brownout_disable_preserves_chassis_and_rotor_state(self):
  p,_=run();policy=PowerState();out=policy.update(p.state['t'],6.5,True);self.assertFalse(out['outputsEnabled']);r=p.step(.0005,(0,0),6.5,N,modes=('coast','coast'));self.assertGreater(r['state']['vx'],0);self.assertGreater(r['state']['wheel'][0],0)
 def test_source_endpoint_mismatch_not_hidden(self):
  k=C['motorK'];r=C['motorResistanceOhm'];b=C['motorViscousDragNmPerRadps'];predicted_rpm=k*12/(k*k+r*b)*60/(2*math.pi)
  self.assertGreater(predicted_rpm/C['source']['reportedFreeRpm'],1.1)
 def test_infeasible_voltage_reported_and_not_stepped(self):
  p=DrivePlant(C);p.state['wheel']=[100.,100.];s=copy.deepcopy(p.state);e=p.electrical(1.,(.2,.2),modes=('drive','drive'));self.assertTrue(e['infeasibleCurrentLimitVoltage'])
  with self.assertRaises(ValueError):p.step(.0005,(.2,.2),1.,N,modes=('drive','drive'))
  self.assertEqual(p.state,s)
class SourceContractTest(unittest.TestCase):
 def test_geometry_contract_and_changed_source_rejection(self):
  import importlib.util
  spec=importlib.util.spec_from_file_location('drive_runner',R/'sim/drivetrain/run.py');runner=importlib.util.module_from_spec(spec);spec.loader.exec_module(runner)
  runner.validate(C)
  for key,value in [('geometrySourceSha256','0'*64),('trackM',.6),('gearRatio',10.),('wheelRadiusM',.08)]:
   changed=copy.deepcopy(C);changed[key]=value
   with self.assertRaises(ValueError):runner.validate(changed)
 def test_early_supply_failure_preserves_empty_trace(self):
  import importlib.util,unittest.mock
  spec=importlib.util.spec_from_file_location('drive_runner',R/'sim/drivetrain/run.py');runner=importlib.util.module_from_spec(spec);spec.loader.exec_module(runner)
  with unittest.mock.patch.object(runner,'solve_supply',side_effect=SupplyDomainError('synthetic unsupported source')):
   result=runner.run_case(C,'shared-load',duration=.001)
  self.assertIsNone(result['final']);self.assertEqual(result['trace'],[]);self.assertTrue(result['status'].startswith('UNKNOWN_SUPPLY'))
 def test_invalid_load_and_step_do_not_mutate_state(self):
  p=DrivePlant(C);before=copy.deepcopy(p.state)
  for dt,loads in [(True,N),(.001,[-1,1]),(float('nan'),N),(.01,N)]:
   with self.assertRaises(ValueError):p.step(dt,(.4,.4),12.,loads,modes=('drive','drive'))
  self.assertEqual(p.state,before)
if __name__=='__main__':unittest.main()
