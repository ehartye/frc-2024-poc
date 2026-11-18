import unittest,pathlib,sys,json,math,copy,random
R=pathlib.Path(__file__).resolve().parents[1];sys.path[:0]=[str(R/'sim/pressure'),str(R/'sim/integrated')]
import fast_geometry as fast,finite_transport as ref
from patch_registration import rotation
class FastGeometryTest(unittest.TestCase):
 @classmethod
 def setUpClass(cls):cls.c=json.loads((R/'cad/assembly/intake_components.json').read_text())
 def compare(self,args,c=None):
  c=self.c if c is None else c
  try:a=ref.prepare(c,**args)
  except ValueError as e:
   with self.assertRaises(type(e)) as caught:fast.prepare(c,**args)
   self.assertEqual(str(caught.exception),str(e));return
  b=fast.prepare(c,**args);self.assertEqual(a,b);self.assertEqual(json.dumps(a,sort_keys=True),json.dumps(b,sort_keys=True))
  if a['contacts']:
   self.assertEqual(ref.rates(a,pitch_radius_m=.01212760666360242,rotor_omega=[20.,3.]),ref.rates(b,pitch_radius_m=.01212760666360242,rotor_omega=[20.,3.]))
 def test_separated_straight_wrap_caps_boundary(self):
  for p in [[-1.,0.,0.],[.010567440020061214,0.,0.],[.158,0.,0.],[.1642,0.,0.],[.01,0.,.012],[.01,0.,-.012]]:self.compare(dict(note_center=p,resolution=(256,256)))
 def test_random_objective_frames(self):
  rng=random.Random(2024)
  for _ in range(12):
   heading=rng.uniform(-2,2);pitch=rng.uniform(-.03,.03);origin=[rng.uniform(-1,1),rng.uniform(-1,1),rng.uniform(-.1,.1)];cb=rotation(heading,pitch);relative=[rng.uniform(.0,.16),rng.uniform(-.00005,.00005),rng.uniform(-.002,.002)];center=ref.a.add(origin,ref.a.mv(cb,relative));nb=rotation(heading+rng.uniform(-.01,.01),pitch+rng.uniform(-.01,.01));self.compare(dict(note_center=center,note_basis=nb,cradle_origin=origin,cradle_basis=cb,u_positive_m=.00002,resolution=(64,64)))
 def test_deep_and_gradient_tie_parity(self):
  self.compare(dict(note_center=[.01,.0005,0.],resolution=(128,128)))
  args=dict(note_center=[.01,0.,0.],resolution=(128,128));p=ref.prepare(self.c,**args);cell=p['contacts'][0];c=copy.deepcopy(self.c);c['pinch']['beltWidth']=2*(abs(cell['pointWorldM'][2])+cell['penetrationM'])*1000;self.compare(args,c)
 def test_extreme_separated_scalar_domain_parity(self):
  for axis in range(3):
   for sign in [-1,1]:
    center=[0.,0.,0.];center[axis]=sign*1e308
    self.compare(dict(note_center=center,resolution=(8,8)))
  self.compare(dict(note_center=[1e308,0.,0.],cradle_origin=[1e308,0.,0.],resolution=(8,8)))
  self.compare(dict(note_center=[1e308,0.,0.],cradle_origin=[-1e308,0.,0.],resolution=(8,8)))
 def test_cache_is_immutable(self):
  g=fast.material_grid(.1524,.0254,64,64)
  with self.assertRaises(ValueError):g[0,0]=1.
 def test_near_axis_frame_not_underbounded(self):
  # Accepted near-unit transforms are bounded as affine maps, not normalized-away scales.
  b=rotation(.00001,.00001);b[0]=[x*(1+2e-11) for x in b[0]]
  self.compare(dict(note_center=[.164,0.,0.],note_basis=b,resolution=(128,128)))
if __name__=='__main__':unittest.main()
