import pathlib,sys,copy,math,json,unittest,hashlib
from unittest.mock import patch
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/integrated'))
import coupled_lift_run as runner
from coupled_lift_phase import CoupledLift,PhaseDomainError
import finite_retention_guide as guide
class Lift(unittest.TestCase):
 @classmethod
 def setUpClass(cls):cls.data=runner.setup()
 def model(self):
  source,patch,a,h,g,c,_=self.data;return CoupledLift(source['checkpoint'],patch,a,h,g,c,source['inputs'])
 def test_checkpoint_state_energy_and_mass_partition(self):
  p=self.model();source=self.data[0];saved=source['checkpoint'];self.assertEqual(p.s['drive'],saved['drive']);self.assertEqual(p.s['note'],saved['note']);self.assertEqual(p.s['pinchW'],[-w for w in saved['note']['w']]);self.assertEqual(p.s['timeS'],saved['timeS']);self.assertEqual(p.drive.c['massKg'],38.)
  self.assertLess(abs(p.initialMechanical-source['final']['energyJ']['kinetic']-source['final']['energyJ']['spring']),1e-12)
  for k,v in saved['energy'].items():self.assertEqual(p.ledger[k],v)
  for k in ['vPivot','theta','thetaRate','noteVz','notePitch','notePitchRate']:self.assertEqual(p.s[k],0.)
 def test_pure_rates_and_eight_motor_power(self):
  p=self.model();before=copy.deepcopy((p.s,p.ledger));cmd=p.commands('gravity-feedforward');r=p.rates(p.s,12.,cmd,{})
  d=p.drive.electrical(12.,cmd['drive'],modes=('drive','drive'),state=p.s['drive']);a=p.act.electrical(12.,cmd['actuator'],cmd['actuatorModes'],[p.s['drive']['x'],p.s['zPivot'],p.s['theta']],[p.s['drive']['vx'],0.,0.]);h=[p.hold.electrical(12.,cmd['pinch'][i],'drive',p.s['pinchW'][i]) for i in [0,1]]
  self.assertEqual(len(d['perMotor'])+len(a['motors'])+len(h),8);self.assertAlmostEqual(r['busPowerW'],d['busPowerW']+a['busPowerW']+sum(x['busPowerW'] for x in h),places=12);self.assertEqual(before,(p.s,p.ledger));self.assertEqual(r['floorElasticJ'],0.);self.assertEqual(r['guideElasticJ'],0.)
 def test_nonplanar_and_witness_domain(self):
  p=self.model();p.s['note']['v'][1]=.01;before=copy.deepcopy((p.s,p.ledger))
  with self.assertRaisesRegex(PhaseDomainError,'NONPLANAR'):p.step(.001,p.commands('gravity-feedforward'))
  self.assertEqual(before,(p.s,p.ledger))
  p=self.model();p.s['note']['p'][0]+=.1;before=copy.deepcopy((p.s,p.ledger))
  with self.assertRaisesRegex(PhaseDomainError,'PATCH'):p.step(.001,p.commands('gravity-feedforward'))
  self.assertEqual(before,(p.s,p.ledger))
 def test_yawed_static_frame_and_entry_identity_rejected(self):
  p=self.model();p.s['drive']['heading']=.1;p.s['drive']['yaw_rate']=0.;before=copy.deepcopy((p.s,p.ledger))
  with self.assertRaisesRegex(PhaseDomainError,'FRAME'):p.step(.001,p.commands('gravity-feedforward'))
  self.assertEqual(before,(p.s,p.ledger))
  source,pa,a,h,g,c,_=self.data
  saved=copy.deepcopy(source['checkpoint']);saved['driveConfig']['massKg']=46.
  with self.assertRaises(ValueError):CoupledLift(saved,pa,a,h,g,c,source['inputs'])
  for target in ['heading','yaw']:
   changed=copy.deepcopy(source['checkpoint']);reg=copy.deepcopy(pa)
   if target=='heading':changed['drive']['heading']=.1
   else:changed['note']['yaw']=.1
   reg['preservedState']={'chassis':changed['drive'],'note':changed['note']}
   with self.assertRaisesRegex(PhaseDomainError,'FRAME'):CoupledLift(changed,reg,a,h,g,c,source['inputs'])
  identity=dict(source['inputs']);identity[next(iter(identity))]='0'*64
  with self.assertRaises(ValueError):CoupledLift(source['checkpoint'],pa,a,h,g,c,identity)
 def test_unknown_guide_atomic(self):
  p=self.model();before=copy.deepcopy((p.s,p.ledger))
  with patch.object(guide,'query_round_xz',return_value={'status':'UNKNOWN','geometryDomainValid':False}):
   with self.assertRaises(ValueError):p.step(.001,p.commands('gravity-feedforward'))
  self.assertEqual(before,(p.s,p.ledger))
 def test_archived_20deg_disagreement_retained(self):
  c=self.data[4];self.assertTrue(c['knownDisagreements'])
  for q in c['knownDisagreements']:
   xyz=q['centerRobotMm'];r=guide.query_round_xz([xyz[0]/1000,xyz[2]/1000],math.radians(q['pitchDeg']),c,y_m=xyz[1]/1000);self.assertEqual(r['status'],'UNKNOWN')
 def test_instantaneous_work_balance(self):
  p=self.model();cmd=p.commands('gravity-feedforward')
  # Positive tiny pitch/rates, tiny floor penetration; no declared acquisition reset.
  s=copy.deepcopy(p.s);s['noteZ']-=.00003;s['notePitch']=1e-6;s['noteVz']=-.001;s['notePitchRate']=.0001;s['theta']=1e-6;s['thetaRate']=.0001;s['vPivot']=-.0001
  r=p.rates(s,12.,cmd,{});eps=1e-7;plus=p.shifted(s,r,eps);minus=p.shifted(s,r,-eps);ep=p.mechanical(plus,p.rates(plus,12.,cmd,{}));em=p.mechanical(minus,p.rates(minus,12.,cmd,{}));heat=sum(v for k,v in r['energyRates'].items() if k not in ['sourceWork','sourceHeat'])
  self.assertAlmostEqual((ep-em)/(2*eps),r['busPowerW']-heat,places=5)
 def test_step_preserves_actual_nonplanar_state(self):
  p=self.model();before=copy.deepcopy(p.s);p.step(.00025,p.commands('gravity-feedforward'))
  self.assertEqual(p.s['drive']['vy'],before['drive']['vy']);self.assertEqual(p.s['drive']['yaw_rate'],before['drive']['yaw_rate']);self.assertEqual(p.s['note']['spin'],before['note']['spin']);self.assertAlmostEqual(p.s['note']['yaw'],before['note']['yaw']+.00025*before['note']['spin'],places=30)
 def test_fresh_receipt_refinement_and_failure(self):
  r=json.loads((R/'sim/integrated/coupled_lift_evidence.json').read_text());self.assertEqual(set(r['inputs']),set(self.data[-1]))
  for name,pin in r['inputs'].items():self.assertEqual(hashlib.sha256((R/name).read_bytes()).hexdigest(),pin)
  errors=[]
  for case in r['cases'][:3]:
   self.assertEqual(case['outcome'],'HORIZON');self.assertLess(case['finalState']['zPivot'],.0254);self.assertLess(case['finalState']['noteZ'],.0254);errors.append(max(abs(row['energyJ']['residual']-case['initialEnergyResidualJ']) for row in case['trace']))
  self.assertLess(errors[1],errors[0]);self.assertLess(errors[2],errors[1]);self.assertLess(errors[0],4e-6);self.assertEqual(r['cases'][-1]['outcome'],'UNKNOWN_DOMAIN')
if __name__=='__main__':unittest.main()
