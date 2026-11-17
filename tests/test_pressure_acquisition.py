import unittest,pathlib,sys,json,copy,random
R=pathlib.Path(__file__).resolve().parents[1];sys.path[:0]=[str(R/'sim/pressure'),str(R/'sim/acquisition'),str(R/'sim/integrated')]
from pressure_acquisition import PressureAcquisition
from fast_coupled_acquisition import FastCoupledAcquisition
from free_geometry import make_geometry
from free_planar import DomainExit
from coupled_acquisition import rotate
class AdapterTest(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.drive=json.loads((R/'sim/drivetrain/config.json').read_text());cls.note=json.loads((R/'sim/acquisition/free-planar-config.json').read_text())['model'];cls.cad=json.loads((R/'cad/assembly/intake_components.json').read_text());cls.initial=dict(p=[.658,0.],v=[0.,0.],yaw=0.,spin=0.,w=[0.,0.]);cls.loads=[cls.drive['massKg']*9.80665/2]*2
 def make(self,res=(128,128),initial=None):
  geometry=make_geometry(self.cad,lambda *args:dict(status='SUPPORTED')) # synthetic unit fixture, NOT actual obstacle clearance
  return PressureAcquisition(self.drive,self.note,geometry,self.initial if initial is None else initial,self.cad,resolution=res)
 def test_fresh_zero_energy_and_reject_checkpoint(self):
  p=self.make();self.assertEqual(p.initial_energy,0.);self.assertEqual(p.prepared(p.drive.state,p.note,{})['potentialJ'],0.)
  n=copy.deepcopy(self.initial);n['w']=[1.,1.]
  with self.assertRaises(ValueError):self.make(initial=n)
  n=copy.deepcopy(self.initial);n['p']=[.188567440020061214,0.]
  with self.assertRaises(ValueError):self.make(initial=n)
 def test_pure_midpoint_port_matches_full_preview(self):
  rng=random.Random(91)
  for localx in [.48,.158,.010567440020061214]:
   p=self.make();d=p.drive.state;d['heading']=.17;d['vx']=.05;d['vy']=.001;d['yaw_rate']=.02;d['wheel']=[.7,.8];p.note['p']=rotate([.178+localx,0.],d['heading']);p.note['yaw']=d['heading'];p.note['v']=[.01,-.003];p.note['spin']=.015;p.note['w']=[10.,12.];cache={};before=copy.deepcopy((p.drive.state,p.note,p.energy,p.t));port=p.make_port(.001,[.2,.18],[.12,.1],self.loads,['drive','drive'],cache)
   for v in [.1,1.,5.,9.,12.]:
    full=p.preview(.001,[.2,.18],[.12,.1],v,self.loads,['drive','drive'],cache);self.assertEqual(port(v)['busPowerW'],full['a']['busPowerW']+full['b']['electrical']['busPowerW'])
   self.assertEqual(before,(p.drive.state,p.note,p.energy,p.t))
 def test_new_fine_grid_start_matches_legacy_before_contact(self):
  p=self.make((1024,1024));old=FastCoupledAcquisition(self.drive,self.note,p.geometry,self.initial)
  for _ in range(3):
   x=p.step(.001,[.2,.2],[.12,.12],self.loads);y=old.step(.001,[.2,.2],[.12,.12],self.loads)
   for key in ['chassis','note','busV','busCurrentA','energyJ']:self.assertEqual(x[key],y[key])
  self.assertEqual(x['energyJ']['initial'],0.);self.assertEqual(x['energyJ']['spring'],0.)
 def test_atomic_unknown_gate(self):
  p=self.make();before=copy.deepcopy((p.drive.state,p.note,p.energy,p.t));p.geometry=lambda q:dict(status='UNKNOWN_TEST_OBSTACLE',contacts=[])
  with self.assertRaises(DomainExit):p.step(.001,[.2,.2],[.12,.12],self.loads)
  self.assertEqual(before,(p.drive.state,p.note,p.energy,p.t))
 def test_command_disable_does_not_reset_rotors(self):
  p=self.make()
  for _ in range(3):p.step(.001,[.2,.2],[.12,.12],self.loads)
  before=p.note['w'].copy();self.assertGreater(min(before),0.);p.step(.001,[0.,0.],[0.,0.],self.loads,modes=['coast','coast']);self.assertEqual(p.note['w'],before);self.assertGreater(max(p.drive.state['wheel']),0.)
if __name__=='__main__':unittest.main()
