"""Four REV modules in the planar 2024 chassis/lift/intake MuJoCo plant."""
import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from r7_dynamics import BUDGET, install, verify_binding
from run_axial_shear_lift import ROOT, load
from second_wing_pickup import ReturnPlant, angle_error
from wing_note_delivery import DeliveryPlant


STATIONS = ("Front left", "Front right", "Rear left", "Rear right")
TAGS = ("fl", "fr", "rl", "rr")
LAYOUT = ROOT / "cad/drivetrain/maxswerve-layout.json"

# WPILib v2024.3.2 DCMotor and REV's 2024 MAXSwerve template current settings.
STALL_A = np.array([105., 100.])
KT = np.array([2.6, .97]) / STALL_A
RESISTANCE = 12. / STALL_A
KE = (12. - RESISTANCE * [1.8, 1.4]) / (np.array([5676., 11000.]) * math.tau / 60)
LIMIT_A = np.array([50., 20.])


def numbers(values):
    return " ".join(map(str, values))


class SwervePickup(DeliveryPlant):
    """Wheel angles and speeds are attained coordinates; commands request a body twist."""

    @classmethod
    def from_floor_receipt(cls, pickup, dt, grid, segments, *, ring_phase_deg=0.):
        """Reconstruct the four-module topology before restoring an attained state."""
        verify_binding(pickup)
        q = pickup["initialQ"]
        start = dict(timeS=0.,chassis=dict(x=q[0],y=q[1],heading=q[2],vx=0.,vy=0.,yaw_rate=0.,wheel=[0.,0.]),
                     note=dict(p=q[-7:-5],v=[0.,0.],yaw=0.,spin=0.,w=[0.,0.]))
        state = dict(q=q[:5]+(q[-7:] if cls.carries_note else []),v=[0.]*(11 if cls.carries_note else 5),busV=12.,clock=0.,rotor=[0.]*4,shear=[0.]*4)
        calibration = load(ROOT/"sim/mujoco/evidence/launcher-quadrature-verification-20261118.json")
        return cls(start,state,dt,grid,segments,calibration["runs"]["grid-16384"]["parameters"],
                   ring_phase_deg=ring_phase_deg, mass_case=pickup.get("r7MassCase"))

    def drivetrain_xml(self, xml):
        layout = load(LAYOUT)
        module = layout["module"]
        self.radius = module["wheelDiameterMm"] / 2000
        self.drive_ratio = module["driveReduction"]
        num, den = module["steeringReduction"]
        self.steer_ratio = num / den
        self.stations = np.array([layout["stations"][s]["wheelCenterMm"] for s in STATIONS]) / 1000
        # Fusion positive roll moves about the negative of our canonical wheel axle.
        axes = layout["articulation"]["nativeJointTest"]["positiveAxes"]
        self.mount = np.array([math.atan2(axes[s]["roll"][0], -axes[s]["roll"][1]) for s in STATIONS])
        tree = ET.fromstring(xml)
        chassis = tree.find(".//body[@name='chassis']")
        tendons = ET.SubElement(tree, "tendon") if tree.find("tendon") is None else tree.find("tendon")
        # Assumed rotating masses and shaft inertias. Conserve the original robot's
        # total mass, horizontal first moment and floor-pose yaw inertia.
        fork_mass, tire_mass = .55, .35
        fork_yaw, tire_axial, tire_radial = .0012, .00032, .00022
        module_mass = fork_mass + tire_mass
        fixed = chassis.find("inertial")
        mass = float(fixed.get("mass"))
        center = np.fromstring(fixed.get("pos"), sep=" ")
        inertia = np.fromstring(fixed.get("diaginertia"), sep=" ")
        reduced = mass - 4 * module_mass
        new_center = (mass * center - module_mass * self.stations.sum(axis=0)) / reduced
        inertia[2] += mass * (center[:2] @ center[:2]) - reduced * (new_center[:2] @ new_center[:2])
        inertia[2] -= module_mass * np.square(self.stations[:, :2]).sum() + 4 * (fork_yaw + tire_radial)
        fixed.set("mass", str(reduced))
        fixed.set("pos", numbers(new_center))
        fixed.set("diaginertia", numbers(inertia))
        for tag, station, heading in zip(TAGS, self.stations, self.mount):
            fork = ET.SubElement(chassis, "body", name=f"fork_{tag}", pos=numbers(station),
                                 quat=numbers([math.cos(heading / 2), 0, 0, math.sin(heading / 2)]))
            azimuth, roll = f"azimuth_{tag}", f"roll_{tag}"
            ET.SubElement(fork, "joint", name=azimuth, axis="0 0 1", armature=str(1e-5 * self.steer_ratio**2))
            ET.SubElement(fork, "inertial", pos="0 0 0", mass=str(fork_mass), diaginertia=numbers([fork_yaw]*3))
            wheel = ET.SubElement(fork, "body", name=f"wheel_{tag}")
            ET.SubElement(wheel, "joint", name=roll, axis="0 1 0")
            ET.SubElement(wheel, "inertial", pos="0 0 0", mass=str(tire_mass),
                          diaginertia=numbers([tire_radial, tire_axial, tire_radial]))
            # Carrier-frame gearing. One shaft coordinate sets reflected inertia,
            # back EMF and the transpose torque map. Coupling sign is inferred.
            shaft = ET.SubElement(tendons, "fixed", name=f"drive_shaft_{tag}", armature="0.000035")
            ET.SubElement(shaft, "joint", joint=roll, coef=str(self.drive_ratio))
            ET.SubElement(shaft, "joint", joint=azimuth, coef="1")
        if self.mass_case is not None:
            install(chassis, self.mass_case)
        return ET.tostring(tree, encoding="unicode")

    def __init__(self, *args, mass_case=None, **kwargs):
        self.mass_case = mass_case
        super().__init__(*args, **kwargs)
        if mass_case is not None:
            # Use the same nominal controller calibration across all three plants.
            groups = load(BUDGET)["cases"]["nominal"]
            cradle = groups["cradle"]
            controller_mass = dict(carriageMassKg=groups["carriage"]["massKg"], rotatingMassKg=cradle["massKg"],
                                   comPitchInertiaKgM2=cradle["diagonalAtCenterKgM2"][1],
                                   comLocalXZ_M=[cradle["centerM"][0]-self.base.x0, cradle["centerM"][2]-self.base.z0])
            self.base.ac.update(controller_mass)
            self.base.act.c.update(controller_mass)
        self.az = np.array([self.model.joint("azimuth_"+s).dofadr[0] for s in TAGS])
        self.roll = np.array([self.model.joint("roll_"+s).dofadr[0] for s in TAGS])
        self.az_pos = np.array([self.model.joint("azimuth_"+s).qposadr[0] for s in TAGS])
        self.wheels = [self.model.body("wheel_"+s).id for s in TAGS]
        self.angle_target = self.mount.copy()
        self.minimum_bus = self.bus
        self.peak_current = np.zeros(2)
        self.peak_slip = 0.
        self.touch_load = self.model.body_subtreemass[self.chassis] * abs(self.model.opt.gravity[2]) / 4

    def module_duties(self, vx, vy, yaw_rate):
        x, y = self.stations[:, 0], self.stations[:, 1]
        ux, uy = vx - yaw_rate*y, vy + yaw_rate*x
        speed = np.hypot(ux, uy)
        current = self.mount + self.data.qpos[self.az_pos]
        moving = speed > 1e-6
        target = np.where(moving, np.arctan2(uy, ux), self.angle_target)
        error = (target - current + np.pi) % math.tau - np.pi
        reverse = moving & (np.abs(error) > np.pi/2)
        error = np.where(reverse, error - np.sign(error)*np.pi, error)
        self.angle_target = current + error
        surface = speed * np.where(reverse, -1., 1.) * np.cos(error)
        omega = surface / self.radius
        az_rate = np.clip(9*error, -6., 6.)
        v = self.data.qvel
        drive = KE[0]*(self.drive_ratio*omega + az_rate) + .19*(omega-v[self.roll])
        steer = KE[1]*self.steer_ratio*az_rate + .8*(az_rate-v[self.az])
        return np.clip(np.column_stack((drive, steer)) / self.bus, -1., 1.)

    def controls(self, belt, launcher_duty, feed_enabled):
        if hasattr(self, "seated"):
            return self.carry_controls(belt, launcher_duty)
        _, actuator, pinch, launcher = ReturnPlant.controls(self, belt, launcher_duty, feed_enabled)
        # Known field mark guides the approach. Actual piece contact controls the
        # final longitudinal approach using the same pickup observation as ReturnPlant.
        rel, yaw, _ = self.observe()
        c, s = math.cos(yaw), math.sin(yaw)
        delta = self.mark_xy - self.data.qpos[:2]
        side = -s*delta[0] + c*delta[1]
        vx = np.clip(1.5*(rel[0]-.178), -.12, self.capture_speed)
        vy = np.clip(2.4*side, -.38, .38)
        negative, positive = self.last_loads
        if max(negative, positive) > .02:
            # Once a belt touches, shift toward the loaded side instead of
            # pushing the ring across the floor on that belt alone.
            vy = np.clip(.08*(positive-negative), -.12, .12)
            if min(negative, positive) < .02:
                vx = min(vx, .07)
        heading_error = angle_error(self.pickup_heading, yaw) if hasattr(self, "pickup_heading") else -math.atan2(s, c)
        turn = np.clip(3.*heading_error, -.9, .9)
        return self.module_duties(vx, vy, turn), actuator, pinch, launcher

    def carry_controls(self, belt, launcher_duty):
        d = self.data
        elapsed = d.time - self.seated
        demand = self.retained_command(belt, round(elapsed/.01))
        self.brakes = demand["pinchBrakeFraction"]
        goal = self.shoot_position
        if self.transit_path is not None:
            goal, _ = self.transit_path.target(d.qpos[:2])
        heading = angle_error(self.shoot_heading, d.qpos[2])
        if getattr(self, "stage_exit", None) is not None:
            upfield = -np.array([math.cos(self.shoot_heading), math.sin(self.shoot_heading)])
            if (d.qpos[:2]-self.stage_exit)@upfield > .05:
                goal = self.stage_exit
                heading = angle_error(self.pickup_heading, d.qpos[2])
            else:
                self.stage_exit = None
        if getattr(self, "wall_approach", False) and abs(heading) > .09:
            # Stay at least 0.20 m upfield of the final station while yawing.
            away = -np.array([math.cos(self.shoot_heading), math.sin(self.shoot_heading)])
            goal = goal + away * max(0., .20 - (goal-self.shoot_position)@away)
        error = goal - d.qpos[:2]
        target = error * min(2.4, self.transit_speed / max(np.linalg.norm(error), 1e-12))
        # Bound acceleration while the ring initially shares support with the floor.
        if elapsed < .55:
            target[:] = 0.
        change = target - self.field_velocity
        accel = self.lifting_drive_accel if elapsed < 3. else 1.2
        self.field_velocity += change * min(1., .01*accel/max(np.linalg.norm(change), 1e-12))
        rate = self.turn_rate
        if getattr(self, "wall_approach", False):
            rate = min(rate, math.sqrt(1.5*self.turn_accel*abs(heading)))
        turn = np.clip(2.5*heading, -rate, rate) if elapsed > .65 else 0.
        self.turn_reference += np.clip(turn-self.turn_reference, -.01*self.turn_accel, .01*self.turn_accel)
        body_velocity = d.xmat[self.chassis].reshape(3, 3)[:2, :2].T @ self.field_velocity
        return self.module_duties(*body_velocity, self.turn_reference), demand["actuator"], demand["pinch"], launcher_duty

    def drive_ports(self, voltage, drive):
        v = self.data.qvel
        shaft = np.column_stack((self.drive_ratio*v[self.roll] + v[self.az], self.steer_ratio*v[self.az]))
        emf = shaft * KE
        amps = np.clip((drive*voltage - emf) / RESISTANCE, -LIMIT_A, LIMIT_A)
        terminal = emf + amps * RESISTANCE
        power = terminal * amps
        # Empirical torque and EMF constants differ. Bound motoring torque by
        # the reciprocal constant so electrical input covers copper and shaft work.
        torque = amps * np.where(amps*shaft > 0, np.minimum(KT, KE), KE)
        return dict(torque=torque, perMotor=[dict(busPowerW=p) for p in power.flat], amps=amps,
                    infeasibleCurrentLimitVoltage=bool(np.max(np.abs(terminal)) > voltage+1e-8))

    def apply_drive(self, voltage, cmd, drive):
        d = self.data
        motor = drive["torque"]
        d.qfrc_applied[self.roll] = self.drive_ratio * motor[:, 0]
        d.qfrc_applied[self.az] = motor[:, 0] + self.steer_ratio * motor[:, 1]
        self.peak_current = np.maximum(self.peak_current, np.abs(drive["amps"]).max(axis=0))
        self.minimum_bus = min(self.minimum_bus, voltage)
        # Flat floor, equal static normal loads, assumed mu and slip speed.
        # Apply the force once on the wheel so roll and chassis see its reactions.
        local = self.stations[:, :2] @ d.xmat[self.chassis].reshape(3, 3)[:2, :2].T
        yaw = d.qpos[2] + self.mount + d.qpos[self.az_pos]
        direction = np.column_stack((np.cos(yaw), np.sin(yaw)))
        slip = d.qvel[:2] + d.qvel[2]*np.column_stack((-local[:, 1], local[:, 0]))
        slip -= (self.radius*d.qvel[self.roll])[:, None] * direction
        mag = np.linalg.norm(slip, axis=1)
        self.peak_slip = max(self.peak_slip, float(max(mag)))
        factor = -.9*self.touch_load*np.tanh(mag/.10) / np.maximum(mag, 1e-15)
        for wheel, xy in zip(self.wheels, slip*factor[:, None]):
            point = d.xpos[wheel] - [0., 0., self.radius]
            mujoco.mj_applyFT(self.model, d, np.r_[xy, 0.], np.zeros(3), point, wheel, d.qfrc_applied)
        # The legacy two differential rotor slots carry zero throughout a swerve run.
        return (0., 0.)
