"""Counterexamples for the simultaneous motion, wrench and swept-volume checks."""
import json
import math
import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "sim" / directory) for directory in
                ("pressure", "integrated", "actuation", "acquisition", "drivetrain")]
from turning_contact import belt_wrenches
from turning_sweep import NativeTurnPath
from verify_turning_lift import overlap


class LiftOverlap(unittest.TestCase):
    def setUp(self):
        self.t = np.linspace(0, 5.8, 59)
        self.travel = .449 * self.t / self.t[-1]
        self.speed = np.full_like(self.t, .32)
        self.yaw = np.full_like(self.t, .22)

    def passes(self, speed, yaw, travel, heading):
        return overlap(self.t, speed, yaw, travel, heading, "left")[0]

    def test_measured_arc(self):
        self.assertTrue(self.passes(self.speed, self.yaw, self.travel, self.yaw*self.t))

    def test_turn_entirely_before_lift(self):
        heading = np.minimum(1.4, self.t)
        self.assertFalse(self.passes(self.speed, np.gradient(heading, self.t), self.travel, heading))

    def test_pivot_or_opposite_turn(self):
        self.assertFalse(self.passes(0*self.speed, self.yaw, self.travel, self.yaw*self.t))
        self.assertFalse(self.passes(self.speed, -self.yaw, self.travel, -self.yaw*self.t))

    def test_oscillation_cannot_accumulate_full_stroke(self):
        travel = .08*(1-np.cos(4*math.pi*self.t/self.t[-1]))
        self.assertFalse(self.passes(self.speed, self.yaw, travel, self.t*self.yaw))


class GripWrench(unittest.TestCase):
    def test_rotated_grip_preserves_force_torque_and_reciprocal_work(self):
        # Four wall points give each elastic region a different load and velocity.
        sides = np.array([0, 0, 1, 1])
        arm = np.array([[-.08,-.17,0],[.08,-.17,0],[-.08,.17,0],[.08,.17,0]])
        normal = np.array([[0,1,0],[0,1,0],[0,-1,0],[0,-1,0]], dtype=float)
        tangent = np.tile([1., 0., 0.], (4,1))
        def solve(rotation):
            geom = dict(normalWorld=normal@rotation.T, tangentWorld=tangent@rotation.T,
                        normalN=np.array([3.,4.,4.,2.]), motor=sides,
                        cradleLeverWorldM=(arm+[.005,0,0])@rotation.T,
                        noteLeverWorldM=arm@rotation.T, potentialJ=.001)
            return belt_wrenches(geom, note_v=rotation@[.2,.02,.03], note_w=rotation@[.01,-.12,.4],
                    cradle_v=rotation@[.18,.005,.03], cradle_w=rotation@[0.,-.14,.32],
                    belt_w=np.array([.1,-.1]), cradle_basis=rotation, forward=rotation[:,0],
                    shear_m=np.array([.0001,-.0002,.0002,.00015]), motor_radius=.025, mu=.8, slip_scale=.02)
        angle=.8
        rotation=np.array([[math.cos(angle),-math.sin(angle),0],[math.sin(angle),math.cos(angle),0],[0,0,1.]])
        base,turned=solve(np.eye(3)),solve(rotation)
        for field in ("force","cradleMoment","noteMoment"):
            np.testing.assert_allclose(turned[field], rotation@base[field], atol=1e-12)
        for field in ("heat","potential","shearRate","rotor"):
            np.testing.assert_allclose(turned[field], base[field], atol=1e-12)
        self.assertGreater(abs(base["noteMoment"][0]),.01)
        self.assertLess(abs(turned["powerResidual"]),1e-12)


class NativeTurningSweep(unittest.TestCase):
    def test_world_turn_roll_and_inside_chord_in_robot_frame(self):
        # Robot translates and yaws by 90 degrees. The free ring rolls about world X.
        a=np.array([0.,0.,0.,0.,0.,.18,.035,.0254,1.,0.,0.,0.])
        b=np.array([.03,.04,math.pi/2,.2,.8,.22,.235,.23,math.cos(.3),math.sin(.3),0.,0.])
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp)
            (folder/"result.json").write_text(json.dumps(dict(dt=.01,initialQ=a.tolist(),outcome="HORIZON",end=.01)))
            (folder/"trace.jsonl").write_text(json.dumps(dict(t=.01,q=b.tolist())))
            path=NativeTurnPath(folder)
        interval=path.interval(0,1)
        for fraction in np.linspace(0,1,21):
            q=a+fraction*(b-a)
            yaw,pitch=q[2],q[4]
            c,s=math.cos(yaw),math.sin(yaw)
            into_robot=np.array([[c,s,0],[-s,c,0],[0,0,1]])
            c,s=math.cos(pitch),math.sin(pitch)
            into_cradle=np.array([[c,0,s],[0,1,0],[-s,0,c]])
            pivot=np.array([178.,0.,25.4])
            for circle_angle in np.linspace(0,2*math.pi,24):
                angle=.6*fraction
                dx,dy=152.4*math.cos(circle_angle),152.4*math.sin(circle_angle)
                point=1000*(q[5:8]-[q[0],q[1],0])+[dx,dy*math.cos(angle),dy*math.sin(angle)]
                fixed=into_robot@point
                car=fixed-[0,0,1000*q[3]]
                for group,actual in (("fixed",fixed),("carriage",car),("cradle",pivot+into_cradle@(car-pivot))):
                    env=interval["frames"][group]
                    v=actual-env["center"];n=np.array(env["normal"])
                    axial=v@n;radial=np.linalg.norm(v-axial*n)
                    self.assertLessEqual(math.hypot(axial,radial-152.4),env["padMm"])
                self.assertLessEqual(interval["yBoundsMm"][0],fixed[1]-25.4)
                self.assertGreaterEqual(interval["yBoundsMm"][1],fixed[1]+25.4)


class TurningEntryPoint(unittest.TestCase):
    def test_other_archive_is_rejected_before_output_creation(self):
        with tempfile.TemporaryDirectory() as tmp:
            archive, output = Path(tmp)/"candidate.f3d", Path(tmp)/"result"
            archive.write_bytes(b"wrong archive identity")
            run = subprocess.run([sys.executable, "-B", str(ROOT/"sim/pressure/run_floor_pickup.py"),
                                  "--archive", str(archive), "--turn", "left", "--output", str(output)],
                                 capture_output=True, text=True)
            self.assertNotEqual(run.returncode, 0)
            self.assertIn("Native archive differs", run.stderr)
            self.assertFalse(output.exists())

    @unittest.skipUnless(importlib.util.find_spec("mujoco"), "requires the physics runtime")
    def test_mass_partition_and_free_note(self):
        import mujoco
        from turning_plant import TurningPlant
        seed = dict(chassis=dict(x=0., y=0., heading=0., vx=0., vy=0., yaw_rate=0., wheel=[0.,0.]),
                    note=dict(p=[.185,0.], yaw=0., v=[0.,0.], spin=0., w=[0.,0.]))
        plant = TurningPlant(seed, 512)
        plant.data.qpos[:] = plant.q
        mujoco.mj_forward(plant.robot, plant.data)
        mass_matrix = np.empty((plant.robot.nv, plant.robot.nv))
        mujoco.mj_fullM(plant.robot, plant.data, mass_matrix)
        self.assertAlmostEqual(sum(plant.robot.body_mass) - plant.nc["pieceMassKg"], 45.)
        self.assertAlmostEqual(mass_matrix[2,2], 6.)
        self.assertEqual(plant.robot.nv, 11)
        self.assertEqual(plant.robot.neq, 0)
        self.assertEqual(plant.robot.nmocap, 0)
        self.assertEqual(plant.robot.jnt_type[-1], mujoco.mjtJoint.mjJNT_FREE)
