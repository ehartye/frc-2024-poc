"""Check the installed module frames and power transfer on the combined robot."""
import math
from pathlib import Path
import sys
import unittest

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim/pressure"))
from maxswerve import KE, LIMIT_A, RESISTANCE, SwervePickup
from run_axial_shear_lift import ROOT, load
from second_wing_pickup import ReturnPlant
from wing_note_release import SwerveShot


def at_rest(plant_type):
    start = dict(chassis=dict(x=0., y=0., heading=0., vx=0., vy=0., yaw_rate=0., wheel=[0., 0.]),
                 note=dict(p=[.658, 0.], v=[0., 0.], yaw=0., spin=0., w=[0., 0.]))
    state = dict(q=[0.]*5 + [.658, 0., .0254, 1., 0., 0., 0.], v=[0.]*11,
                 busV=12., clock=0., rotor=[0.]*4, shear=[0.]*4)
    data = load(ROOT / "sim/mujoco/evidence/launcher-quadrature-verification-20261118.json")
    return plant_type(start, state, .0005, 256, 32, data["runs"]["grid-16384"]["parameters"])


class SwerveTest(unittest.TestCase):
    def setUp(self):
        self.p = at_rest(SwervePickup)

    def test_shooting_station_uses_field_error_on_four_modules(self):
        shot = at_rest(SwerveShot)
        shot.data.qpos[2] = math.pi/2
        mujoco.mj_forward(shot.model,shot.data)
        shot.shoot_position = np.array([.01,0.])
        shot.shoot_heading = math.pi/2
        expected = self.p.module_duties(0.,-.024,0.)
        np.testing.assert_allclose(shot.station_drive(),expected,atol=1e-12)

    def test_module_registration_and_conserved_chassis_inertia(self):
        p = self.p
        original = at_rest(ReturnPlant)
        np.testing.assert_allclose(p.data.xpos[p.wheels], p.stations)
        np.testing.assert_allclose(np.cos(p.mount), [1, 0, 0, -1], atol=1e-12)
        np.testing.assert_allclose(np.sin(p.mount), [0, -1, 1, 0], atol=1e-12)
        self.assertAlmostEqual(p.model.body_subtreemass[p.chassis], original.model.body_subtreemass[original.chassis])
        np.testing.assert_allclose(p.data.subtree_com[p.chassis], original.data.subtree_com[original.chassis], atol=1e-12)
        def matrix(robot):
            full = np.empty((robot.model.nv, robot.model.nv))
            mujoco.mj_fullM(robot.model, robot.data, full)
            return full[:5, :5]
        np.testing.assert_allclose(matrix(p), matrix(original), atol=1e-12)
        self.assertEqual(p.model.nv-original.model.nv, 8)

    def test_station_contact_and_transmission_power(self):
        p = self.p
        for az, roll in ((1.7, 0.), (0., -4.), (-1.7, 4.)):
            p.data.qvel[p.az] = az
            p.data.qvel[p.roll] = roll
            speeds = np.array([az+p.drive_ratio*roll, p.steer_ratio*az])
            for duty in (-1., 0., 1.):
                drive = p.drive_ports(11., np.full((4, 2), duty))
                p.data.qfrc_applied[:] = 0.
                p.apply_drive(11., None, drive)
                torque = drive["torque"]
                motor_power = (speeds*torque).sum()
                loss = sum(m["busPowerW"] for m in drive["perMotor"]) - motor_power
                copper = np.sum(drive["amps"]**2 * RESISTANCE)
                self.assertGreaterEqual(loss+1e-10, copper)
                self.assertTrue((np.abs(drive["amps"]) <= LIMIT_A).all())
                # Any difference after the torque map is tire slip dissipation.
                tire_power = p.data.qvel @ p.data.qfrc_applied - motor_power
                self.assertLessEqual(tire_power, 1e-9)
                if roll == 0:
                    self.assertAlmostEqual(tire_power, 0.)
                    np.testing.assert_allclose(drive["amps"][:, 0], np.clip(
                        (duty*11.-KE[0]*az)/RESISTANCE[0], -LIMIT_A[0], LIMIT_A[0]))
                else:
                    self.assertLess(tire_power, 0.)

    def test_speed_reversal_and_zero_speed_hold(self):
        p = self.p
        duties = p.module_duties(.5, 0., 0.)
        self.assertGreater(duties[0, 0], 0)
        self.assertLess(duties[-1, 0], 0)
        np.testing.assert_allclose(duties[[0, 3], 1], 0., atol=1e-12)
        targets = p.angle_target.copy()
        p.module_duties(0., 0., 0.)
        np.testing.assert_allclose(p.angle_target, targets)
        p.data.qpos[p.az_pos[0]] = math.radians(179)
        p.module_duties(-1., -math.tan(math.radians(1)), 0.)
        self.assertAlmostEqual(p.angle_target[0]-p.data.qpos[p.az_pos[0]], math.radians(2))


if __name__ == "__main__":
    unittest.main()
