import pathlib,sys,json,copy,random,unittest,math
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/integrated'))
from coupled_acquisition import CoupledAcquisition,DomainExit
from electrical_midpoint import make_port,ElectricalOnlyCoupled
D=json.loads((R/'sim/drivetrain/config.json').read_text());C=json.loads((R/'sim/acquisition/free-planar-config.json').read_text())['model']
def empty(q):return dict(status='SUPPORTED',contacts=[])
def contacts(q):return dict(status='SUPPORTED',contacts=[dict(motor=i,penetrationM=.001+i*.0003,normal=[0.,s],tangent=[-1.,0.],leverM=[.01,-s*.17]) for i,s in enumerate([-1.,1.])])
def initial():return dict(p=[.2,.001],v=[0.,0.],yaw=0.,spin=0.,w=[0.,0.])
class ElectricalPort(unittest.TestCase):
 def test_randomized_exact_query_and_purity(self):
  rng=random.Random(728)
  for j in range(120):
   p=CoupledAcquisition(D,C,contacts if j%2 else empty,initial());d=p.drive.state
   for key in ['x','y','heading','vx','vy','yaw_rate']:d[key]=rng.uniform(-1,1)
   d['wheel']=[rng.uniform(-30,30) for _ in range(2)];p.note['w']=[rng.uniform(0,400) for _ in range(2)];p.note['v']=[rng.uniform(-1,1) for _ in range(2)];p.note['spin']=rng.uniform(-10,10)
   modes=tuple(rng.choice(['drive','coast','brake']) for _ in range(2));dd=tuple(rng.uniform(-1,1) for _ in range(2));pd=tuple(rng.choice([0.,.04,.12,.8]) for _ in range(2));loads=tuple(rng.uniform(0,400) for _ in range(2));dt=rng.choice([.0001,.001,.0025]);saved=copy.deepcopy((d,p.note,p.energy))
   port=make_port(p,dt,dd,pd,loads,modes)
   for bus in [.1,1.,6.5,9.,12.,16.]:
    ref=p.preview(dt,dd,pd,bus,loads,modes);self.assertEqual(port(bus)['busPowerW'],ref['a']['busPowerW']+ref['b']['electrical']['busPowerW'])
   self.assertEqual(saved,(d,p.note,p.energy))
 def test_all_grid_voltages_with_contact(self):
  p=CoupledAcquisition(D,C,contacts,initial());port=make_port(p,.001,(.2,-.3),(.12,.8),(220,200),('drive','drive'))
  for i in range(513):
   v=.1+15.9*i/512;r=p.preview(.001,(.2,-.3),(.12,.8),v,(220,200),('drive','drive'));self.assertEqual(port(v)['busPowerW'],r['a']['busPowerW']+r['b']['electrical']['busPowerW'])
 def test_domains_and_no_state_mutation(self):
  for cls in [CoupledAcquisition,ElectricalOnlyCoupled]:
   p=cls(D,C,lambda q:dict(status='UNKNOWN_CONTACT',contacts=[]),initial());saved=copy.deepcopy((p.drive.state,p.note,p.energy))
   with self.assertRaises(DomainExit):p.step(.001,(.2,.2),(.12,.12),(220,220))
   self.assertEqual(saved,(p.drive.state,p.note,p.energy))
 def test_step_reference_exact_and_voltage_domain(self):
  for mode in [('drive','drive'),('coast','brake')]:
   a=CoupledAcquisition(D,C,contacts,initial());b=ElectricalOnlyCoupled(D,C,contacts,initial())
   for _ in range(5):self.assertEqual(a.step(.001,(.2,.2),(.12,.12),(220,220),modes=mode),b.step(.001,(.2,.2),(.12,.12),(220,220),modes=mode))
  for cls in [CoupledAcquisition,ElectricalOnlyCoupled]:
   p=cls(D,C,empty,initial());p.drive.state['wheel']=[100,100];saved=copy.deepcopy((p.drive.state,p.note,p.energy))
   with self.assertRaises((DomainExit,ValueError)):p.step(.001,(.2,.2),(.12,.12),(220,220),source_v=1.)
   self.assertEqual(saved,(p.drive.state,p.note,p.energy))
 def test_signed_pinch_state_failure_matches(self):
  for speed in [-20.,-200.,-1000.,0.]:
   p=CoupledAcquisition(D,C,empty,initial());p.note['w']=[speed,speed]
   def outcome(f):
    try:return f()
    except (DomainExit,ValueError) as e:return type(e).__name__,str(e)
   def reference():
    r=p.preview(.001,(.2,.2),(.12,.12),.1,(220,220),('drive','drive'));return {'busPowerW':r['a']['busPowerW']+r['b']['electrical']['busPowerW']}
   self.assertEqual(outcome(reference),outcome(lambda:make_port(p,.001,(.2,.2),(.12,.12),(220,220),('drive','drive'))(.1)))
 def test_fresh_evidence_inventory_and_trace(self):
  import verify_electrical_midpoint as runner,hashlib
  report=json.loads((R/'sim/integrated/electrical-midpoint-evidence.json').read_text())
  _,_,pins=runner.run_free_planar.load(R/'sim/acquisition/free-obstacles.json')
  expected=set(pins)|set(runner.FILES)
  self.assertEqual(set(report['inputs']),expected)
  for name,pin in report['inputs'].items():self.assertEqual(hashlib.sha256((R/name).read_bytes()).hexdigest(),pin)
  self.assertEqual(report['traceSha256'][0],report['traceSha256'][1]);self.assertTrue(report['exactEveryRowEqual'])
  self.assertEqual(report['steps'],752);self.assertEqual(report['status'],'UNKNOWN_DOMAIN: OUTSIDE_BOUNDED_PLANAR_EXPERIMENT');self.assertLess(report['maxAbsEnergyResidualJ'],.002)
if __name__=='__main__':unittest.main()
