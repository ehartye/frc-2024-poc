import pathlib,unittest,copy
import numpy as np
import mujoco
from native_drive_interactive import command,InteractiveDrive
ROOT=pathlib.Path(__file__).resolve().parents[2]
class Controls(unittest.TestCase):
 def test_keys_and_focus(self):
  self.assertEqual(command(['W'])[0],(.3,.3));self.assertEqual(command(['A'])[0],(-.2,.2));self.assertEqual(command(['C'])[1],('coast','coast'));self.assertEqual(command(['W'],False)[0],(0.,0.));self.assertEqual(command(['W'],False)[1],('brake','brake'));self.assertEqual(command(['W'],True,True)[2],'quit')
 def test_no_teleport_and_direct_step_parity(self):
  a=InteractiveDrive(ROOT);b=copy.deepcopy(a.physics)
  for _ in range(160):b.step([.3,.3],('drive','drive'))
  a.tick(['W']);np.testing.assert_array_equal(a.physics.d.qpos,b.d.qpos);np.testing.assert_array_equal(a.physics.d.qvel,b.d.qvel)
  before=a.physics.d.qvel.copy();self.assertTrue(np.any(before!=0));a.tick(quit_requested=True);self.assertTrue(np.any(a.physics.d.qvel!=0));self.assertTrue(a.closed)
 def test_display_cannot_reset_physics(self):
  a=InteractiveDrive(ROOT);a.tick(['W']);state=a.physics.d.qpos.copy();vm=copy.deepcopy(a.physics.m);vd=mujoco.MjData(vm);mujoco.mj_resetData(vm,vd);np.testing.assert_array_equal(a.physics.d.qpos,state)
 def test_inertia_and_rate(self):
  a=InteractiveDrive(ROOT);p=a.physics;self.assertEqual(p.m.opt.timestep,.0000625)
  for s in range(2):self.assertGreater(p.m.dof_armature[p.v[f'wheel_{s}_1']],p.m.dof_armature[p.v[f'wheel_{s}_0']])
  a.tick();self.assertAlmostEqual(p.d.time,.01,places=14)
if __name__=='__main__':unittest.main()
