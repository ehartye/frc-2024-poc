import sys,pathlib,unittest,math
from dataclasses import replace,asdict
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/pressure'))
import cassette_coupling as c
class Coupling(unittest.TestCase):
 def setUp(self):self.p=c.Parameters(resolution=128);self.cfg=c.config();self.s=c.State(0.,.001,.00004,.002,.2,.1)
 def test_actual_pressure_not_spring_budget(self):
  r=c.ports(self.s,self.p,self.cfg)
  self.assertLess(r['noteForceY'],0);self.assertLess(r['cassetteContactForce'],0)
  self.assertNotAlmostEqual(abs(r['noteForceY']),r['springForce'])
  self.assertAlmostEqual(r['noteForceY'],r['cassetteContactForce'],places=12)
 def test_reciprocal_instantaneous_work(self):
  r=c.ports(self.s,self.p,self.cfg,.001)
  self.assertLess(abs(r['powerResidualW']),1e-12);self.assertLess(abs(r['contactWorkResidualW']),1e-12)
  self.assertEqual(r['constraintPowerW'],0)
 def test_potential_gradient(self):
  h=1e-8;r=c.ports(self.s,self.p,self.cfg)
  for name,force in [('y',r['noteForceY']),('u',r['cassetteContactForce'])]:
   a=c.ports(replace(self.s,**{name:getattr(self.s,name)+h}),self.p,self.cfg)['pressureJ']
   b=c.ports(replace(self.s,**{name:getattr(self.s,name)-h}),self.p,self.cfg)['pressureJ']
   self.assertAlmostEqual(-(a-b)/(2*h),force,places=7)
 def test_zero_preload_not_zero_contact(self):
  r=c.ports(self.s,replace(self.p,spring_free_length=.007),self.cfg)
  self.assertEqual(r['springForce'],0);self.assertLess(r['noteForceY'],0)
 def test_separated_piece_zero_contact(self):
  r=c.ports(replace(self.s,y=-.003),self.p,self.cfg)
  self.assertEqual(r['activeCells'],0);self.assertEqual(r['pressureJ'],0);self.assertEqual(r['noteForceY'],0)
 def test_atomic_domain(self):
  s=replace(self.s,u=.001499,udot=1.);before=asdict(s)
  with self.assertRaises(ValueError):c.step(s,self.p,self.cfg,1e-4)
  self.assertEqual(asdict(s),before)
 def test_torque_off_preserves_motion(self):
  s=c.step(self.s,self.p,self.cfg,1e-6,0.)
  self.assertNotEqual(s.thetadot,0);self.assertNotEqual(s.ydot,0)
 def test_matches_two_side_cells_where_both_supported(self):
  r=c.positive_side(self.cfg,self.s,self.p)
  full=c.geometry.prepare(self.cfg,note_center=[self.p.note_x,self.s.y,0],u_positive_m=self.s.u,resolution=(128,128))
  self.assertEqual(r['contacts'],[x for x in full['contacts'] if x['side']==1])
 def test_one_side_cull_reference_phase_parity(self):
  a=c.transport.a
  for phase in [0.,.013,.07]:
   p=replace(self.p,resolution=64,grid_phase_rad=phase);s=replace(self.s,y=-.0002)
   r=c.positive_side(self.cfg,s,p);g=dict(a.belt.geometry(self.cfg));g['axisYAbs']-=s.u*1000
   cs=math.cos(phase);sn=math.sin(phase);nb=[[cs,-sn,0],[sn,cs,0],[0,0,1]];expected=[]
   for local,area in a.material_points(self.cfg['note']['majorRadius']/1000,self.cfg['note']['tubeRadius']/1000,64,64):
    point=a.add([p.note_x,s.y,0],a.mv(nb,local));q=c.transport.gradient(point,1,g)
    if q['penetrationM']>0:expected.append((point,area,q['penetrationM']))
   self.assertEqual([(x['pointWorldM'],x['areaM2'],x['penetrationM']) for x in r['contacts']],expected)
 def test_bad_inputs(self):
  for p in [replace(self.p,note_mass=0),replace(self.p,resolution=True),replace(self.p,cam_inertia=float('inf')),replace(self.p,note_x=1e308)]:
   with self.assertRaises(ValueError):c.ports(self.s,p,self.cfg)
  with self.assertRaises(ValueError):c.ports(replace(self.s,ydot=True),self.p,self.cfg)
  with self.assertRaises(ValueError):c.ports(replace(self.s,ydot=1e200),self.p,self.cfg)
if __name__=='__main__':unittest.main()
