import pathlib,sys,unittest,math
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/retention'));import wrench as w
class WrenchTests(unittest.TestCase):
 def setUp(self):self.p=[dict(side=s,positionM=[x,s*.177,z]) for s in [-1,1] for x in [-.015,.015] for z in [-.003,.003]]
 def test_static_weight_constructive(self):
  r=w.allocate(self.p,[0.,2.3],0.,.6,{-1:10.,1:10.});self.assertEqual(r['status'],'CONSTRUCTIVE_ALLOCATION_WITHIN_HYPOTHESIS');self.assertLess(r['wrenchResidual'],1e-9)
 def test_small_patch_finite_torque(self):
  r=w.allocate(self.p,[0.,0.],.05,.6,{-1:10.,1:10.});self.assertEqual(r['status'],'CONSTRUCTIVE_ALLOCATION_WITHIN_HYPOTHESIS')
 def test_point_contact_cannot_supply_pitch(self):
  p=[dict(side=s,positionM=[0.,s*.177,0.]) for s in [-1,1]];r=w.allocate(p,[0.,2.3],.01,.6,{-1:10.,1:10.});self.assertEqual(r['status'],'EXCEEDS_NECESSARY_CAPACITY_BOUND')
 def test_low_preload_fails_force_bound(self):
  self.assertEqual(w.allocate(self.p,[0.,2.3],0.,.3,{-1:2.,1:2.})['status'],'EXCEEDS_NECESSARY_CAPACITY_BOUND')
 def test_one_side_loss_not_planar_acceptance(self):
  self.assertEqual(w.allocate(self.p,[0.,2.3],0.,.6,{-1:0.,1:10.})['status'],'UNRESOLVED_NONPLANAR_WRENCH')
 def test_quintic_end_rates(self):
  for t in [0.,.5]:
   q,dq,ddq=w.quintic(t,.5);self.assertAlmostEqual(dq,0);self.assertAlmostEqual(ddq,0)
 def test_input_rejects_nonfinite(self):
  for v in [True,float('nan'),float('inf')]:
   with self.assertRaises(ValueError):w.allocate(self.p,[0.,v],0.,.6,{-1:10.,1:10.})
if __name__=='__main__':unittest.main()
