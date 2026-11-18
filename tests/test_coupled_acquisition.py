import pathlib,sys,json,copy,unittest,math
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/integrated'))
from coupled_acquisition import CoupledAcquisition,acquisition_rates,DomainExit,cross
D=json.loads((R/'sim/drivetrain/config.json').read_text());C=json.loads((R/'sim/acquisition/free-planar-config.json').read_text())['model']
def empty(q):return dict(status='SUPPORTED',contacts=[])
def plane(q):return dict(status='SUPPORTED',contacts=[] if q[1]<=0 else [dict(motor=0,penetrationM=q[1],normal=[0.,-1.],tangent=[-1.,0.],leverM=[0.,0.])])
def initial():return dict(p=[.2,.001],v=[0.,0.],yaw=0.,spin=0.,w=[0.,0.])
class Coupled(unittest.TestCase):
 def test_equal_opposite_contact_and_world_torque(self):
  n=initial();d=dict(x=0.,y=0.,heading=.2,vx=0.,vy=0.,yaw_rate=0.)
  a=acquisition_rates(n,d,dict(C,groundMu=0.),lambda q:dict(status='SUPPORTED',contacts=[dict(motor=0,penetrationM=.001,normal=[0.,-1.],tangent=[-1.,0.],leverM=[.01,.17])]),12.,(0.,0.))
  for i in range(2):self.assertAlmostEqual(sum(r['forceWorldN'][i] for r in a['contacts'])+a['reactionForceWorldN'][i],0.)
  moment=sum(cross([n['p'][i]+r['leverWorldM'][i] for i in range(2)],r['forceWorldN']) for r in a['contacts'])+a['reactionYawNm']+cross([d['x'],d['y']],a['reactionForceWorldN']);self.assertAlmostEqual(moment,0.)
 def test_internal_force_changes_chassis_without_prescription(self):
  p=CoupledAcquisition(D,dict(C,groundMu=0.),plane,initial());r=p.step(.0001,(0.,0.),(0.,0.),(0.,0.),modes=('coast','coast'))
  self.assertGreater(r['chassis']['vy'],0);self.assertLess(r['note']['v'][1],0)
  for i,k in enumerate(['vx','vy']):self.assertAlmostEqual(D['massKg']*r['chassis'][k]+C['pieceMassKg']*r['note']['v'][i],0.,places=12)
 def test_query_pure_and_domain_stop_atomic(self):
  p=CoupledAcquisition(D,C,empty,initial());before=copy.deepcopy((p.drive.state,p.note,p.energy));p.preview(.0001,(.1,.1),(.12,.12),11.,(220.,220.),('drive','drive'));self.assertEqual(before,(p.drive.state,p.note,p.energy))
  p.geometry=lambda q:dict(status='UNKNOWN_CONTACT',contacts=[])
  with self.assertRaises(DomainExit):p.step(.0001,(.1,.1),(.12,.12),(220.,220.))
  self.assertEqual(before,(p.drive.state,p.note,p.energy))
 def test_combined_bus_load_and_positive_losses(self):
  a=CoupledAcquisition(D,C,empty,initial());b=CoupledAcquisition(D,C,empty,initial());ra=a.step(.0001,(.2,.2),(0.,0.),(220.,220.));rb=b.step(.0001,(.2,.2),(.12,.12),(220.,220.))
  self.assertLess(rb['busV'],ra['busV']);self.assertGreater(rb['energyJ']['pinchWinding'],0.);self.assertFalse(rb['acceptance'])
 def test_coast_keeps_state_momentum(self):
  p=CoupledAcquisition(D,dict(C,groundMu=0.),empty,initial());p.drive.state['vx']=.4;p.note['v']=[.3,.2];p.note['w']=[20.,20.]
  r=p.step(.0001,(0.,0.),(0.,0.),(0.,0.),modes=('coast','coast'))
  self.assertEqual(r['note']['v'],[.3,.2]);self.assertEqual(r['note']['w'],[20.,20.]);self.assertAlmostEqual(r['chassis']['vx'],.4)
if __name__=='__main__':unittest.main()
