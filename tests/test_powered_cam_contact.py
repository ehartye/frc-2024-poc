import unittest,math,random,pathlib,sys,copy,json,hashlib
ROOT=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'cad/assembly'))
from powered_cam_contact import evaluate,command_target,CamGeometry
from cassette_spring_law import spring_state

def dot(a,b):return sum(x*y for x,y in zip(a,b))
def add(a,b):return tuple(x+y for x,y in zip(a,b))
def cross(a,b):return (a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0])
def rotate(p,axis,t):
 c,s=math.cos(t),math.sin(t);ap=cross(axis,p);d=dot(axis,p)
 return tuple(c*p[i]+s*ap[i]+(1-c)*d*axis[i] for i in range(3))
class PoweredCam(unittest.TestCase):
 def ev(self,u=.0009,ud=.02,t=.7,td=1.3,k=20000,c=.5):return evaluate(u,ud,t,td,stiffness_n_m=k,damping_n_s_m=c)
 def test_gap_jacobian_and_potential_gradient(self):
  r=self.ev(c=0);h=1e-6;dh=1e-9
  gt=(self.ev(t=.7+h,c=0)['gap_m']-self.ev(t=.7-h,c=0)['gap_m'])/(2*h)
  self.assertAlmostEqual(gt,r['gap_jacobian'][1],places=11)
  du=(self.ev(u=.0009+dh,c=0)['potential_j']-self.ev(u=.0009-dh,c=0)['potential_j'])/(2*dh)
  dt=(self.ev(t=.7+h,c=0)['potential_j']-self.ev(t=.7-h,c=0)['potential_j'])/(2*h)
  self.assertAlmostEqual(-du,r['cassette_generalized_force_n'],places=6);self.assertAlmostEqual(-dt,r['cam_generalized_torque_nm'],places=8)
 def test_separation_and_onset_policy(self):
  self.assertEqual(self.ev(u=0,t=1)['normal_force_n'],0)
  r=self.ev(u=0,t=0,ud=1);self.assertEqual(r['mode'],'touching');self.assertEqual(r['normal_force_n'],0)
  self.assertGreater(self.ev(u=1e-8,t=0,ud=1)['normal_force_n'],0)
 def test_nonnegative_loss_and_no_tension(self):
  for ud in [-1,0,1]:
   r=self.ev(ud=ud);self.assertGreaterEqual(r['normal_force_n'],0);self.assertGreaterEqual(r['dissipation_w'],0);self.assertAlmostEqual(r['power_identity_residual_w'],0,places=10)
  self.assertEqual(self.ev(ud=-1)['dissipation_w'],0)
 def test_rigid_frame_reciprocity(self):
  rng=random.Random(42)
  for _ in range(60):
   u=rng.uniform(0,.0015);t=rng.uniform(0,math.pi);ud=rng.uniform(-.2,.2);td=rng.uniform(-3,3);r=self.ev(u,ud,t,td)
   axis=[rng.uniform(-1,1) for _ in range(3)];n=math.sqrt(dot(axis,axis));axis=tuple(x/n for x in axis);angle=rng.uniform(-3,3)
   origin=tuple(rng.uniform(-2,2) for _ in range(3));v=tuple(rng.uniform(-2,2) for _ in range(3));omega=tuple(rng.uniform(-2,2) for _ in range(3));power=0;moment=(0,0,0);force=(0,0,0)
   for role,rel in [('cam',(0,-td*.00075*math.sin(t),td*(.006+.00075*math.cos(t)))),('follower',(0,-ud,0))]:
    p=rotate(r[role+'_point_local_m'],axis,angle);f=rotate(r[role+'_force_local_n'],axis,angle);vp=add(add(v,cross(omega,p)),rotate(rel,axis,angle));power+=dot(f,vp);moment=add(moment,cross(add(origin,p),f));force=add(force,f)
   self.assertLess(max(abs(x) for x in moment+force),1e-10);self.assertAlmostEqual(power,r['cam_power_w']+r['cassette_power_w'],places=10)
 def test_dead_center_is_not_latch(self):
  r=self.ev(t=0);self.assertEqual(r['cam_generalized_torque_nm'],0);self.assertFalse(r['positive_latch'])
  self.assertLess(spring_state(.00075*(1-math.cos(.01)))['storedEnergyJ'],spring_state(0)['storedEnergyJ'])
 def test_atomic_invalid_inputs(self):
  state={'u_m':.0009,'u_rate_m_s':.02,'theta_rad':.7,'theta_rate_rad_s':1.3};before=copy.deepcopy(state)
  for bad in [True,float('nan'),float('inf'),10**1000,-1]:
   with self.assertRaises(ValueError):evaluate(**state,stiffness_n_m=bad,damping_n_s_m=.5)
  with self.assertRaises(ValueError):evaluate(.002,0,0,0,stiffness_n_m=1,damping_n_s_m=0)
  with self.assertRaises(ValueError):evaluate(.001,1e308,0,0,stiffness_n_m=1e308,damping_n_s_m=1e308)
  self.assertEqual(state,before)
 def test_source_bound_receipt(self):
  r=json.loads((ROOT/'cad/assembly/evidence/powered-cam-contact.json').read_text())
  self.assertFalse(r['accepted']);self.assertEqual(len(r['rows']),6)
  for p,sha in r['inputs'].items():self.assertEqual(hashlib.sha256((ROOT/p).read_bytes()).hexdigest(),sha,p)
  for row in r['rows']:
   self.assertEqual(row['state_u_udot_theta_thetadot'],row['unchangedMechanicalInput']);self.assertLess(abs(row['result']['power_identity_residual_w']),1e-12)
 def test_pwm_target_not_position(self):
  r=command_target(1500,verified_default_270_mode=True,center_cam_angle_rad=math.pi/2,direction=1);self.assertEqual(r['requested_cam_angle_rad'],math.pi/2);self.assertFalse(r['actual_angle_known'])
  self.assertFalse(command_target(500,verified_default_270_mode=True,center_cam_angle_rad=math.pi/2,direction=1)['within_studied_cam_domain'])
  with self.assertRaises(ValueError):command_target(1500,verified_default_270_mode=False,center_cam_angle_rad=math.pi/2,direction=1)
if __name__=='__main__':unittest.main()
