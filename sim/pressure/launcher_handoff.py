"""Native 2024 raised-feed study. One MuJoCo solve carries the chassis, lift, ring and wheels."""
import math
import sys
import xml.etree.ElementTree as ET
from types import SimpleNamespace

import mujoco
import numpy as np

from run_axial_shear_lift import EVIDENCE, ROOT, load
from native_pressure_arrays import prepare
from signed_pressure_acquisition import motor_electrical
from turning_contact import belt_wrenches, floor_wrench
from turning_plant import TurningPlant

sys.path.insert(0, str(ROOT / "sim/mujoco"))
from native_launcher_quadrature import model_xml, motor_query


class RaisedFeed:
    carries_note = True

    def __init__(self, acquired, row, dt, grid, segments, calibration):
        self.base = TurningPlant(acquired, grid)
        self.launcher = load(ROOT / "sim/component-launcher/six-drive-config.json")
        binding = load(ROOT / "sim/mujoco/evidence/native-launcher-binding-20261118.json")
        tree = ET.fromstring(self.base.xml())
        world = tree.find("worldbody")
        world.remove(world.find("body[@name='note']"))
        tree.find("option").set("timestep", str(dt))
        args = SimpleNamespace(segments=segments, noslip_iterations=0, floating=False,
                               start=0., surface="capsules", ring_phase_deg=0., no_contact=False)
        scale = 1 / (.9 + segments / 128 * .1)
        xml = model_xml(SimpleNamespace(xml=ET.tostring(tree, encoding="unicode")),
                        binding, self.launcher, args, calibration["referenceStiffness"] * scale,
                        self.launcher["friction"], damping=calibration["referenceDamping"] * scale)
        if not self.carries_note:
            # The released NOTE is integrated by the field solver after handoff.
            tree=ET.fromstring(xml)
            world=tree.find("worldbody")
            world.remove(world.find("body[@name='note']"))
            xml=ET.tostring(tree,encoding="unicode")
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)
        self.dt = dt
        self.chassis = self.model.body("chassis").id
        self.cradle = self.model.body("cradle").id
        self.note = None
        self.nq,self.nv=self.model.nq,self.model.nv
        if self.carries_note:
            self.note = self.model.body("note").id
            joint = self.model.joint("note_free")
            self.nq = int(joint.qposadr[0])
            self.nv = int(joint.dofadr[0])
        self.wheel_dofs = [int(self.model.joint(f"launcher_{i}").dofadr[0]) for i in range(4)]
        self.data.qpos[:5] = row["q"][:5]
        self.data.qpos[self.nq:] = row["q"][5:]
        self.data.qvel[:5] = row["v"][:5]
        self.data.qvel[self.nv:] = row["v"][5:]
        self.initial_position = self.data.qpos.tolist()
        self.initial_velocity = self.data.qvel.tolist()
        self.rotors = np.array(row["rotor"])
        self.shear = np.array(row["shear"])
        self.bus = row["busV"]
        self.data.time = row["clock"]
        self.grid = grid
        self.plastic_heat = 0.
        self.source_work = 0.
        self.last_command = None
        self.start_time = self.data.time
        self.feed_start = None
        self.start_height = row["q"][3]
        self.start_pitch = row["q"][4]
        mujoco.mj_forward(self.model, self.data)

    def contact(self):
        b, d = self.base, self.data
        pos = d.xpos[self.note]
        nr = d.xmat[self.note].reshape(3, 3)
        cr = d.xmat[self.cradle].reshape(3, 3)
        yr = d.xmat[self.chassis].reshape(3, 3)
        v = d.qvel
        nw = nr @ v[self.nv+3:self.nv+6]
        cv = np.array([v[0], v[1], v[3]]) + v[2]*b.x0*yr[:, 1]
        cw = np.array([0., 0., v[2]]) - v[4]*yr[:, 1]
        points = prepare(b.cad, b.binding, note_center=pos.tolist(), note_basis=nr.tolist(),
                         cradle_origin=d.xpos[self.cradle].tolist(), cradle_basis=cr.tolist(),
                         resolution=(self.grid, self.grid), k_area=1e7)
        # As the ring leaves each belt patch, falling normal load releases its
        # stored shear. Project onto the current friction limit and book the loss.
        walls = np.abs(points["normalWorld"] @ cr[:, 2]) < 1e-8
        regions = 2*points["motor"] + (points["noteLeverWorldM"] @ yr[:, 0] >= 0)
        # Match the reduction used to distribute spring force to each patch.
        load_by_region = np.array([points["normalN"][walls & (regions == i)].sum() for i in range(4)])
        limit = b.nc["beltMu"] * load_by_region / 1250. * (1. - 16.*np.finfo(float).eps)
        projected = np.clip(self.shear, -limit, limit)
        self.plastic_heat += 625. * (self.shear @ self.shear - projected @ projected)
        self.shear = projected
        belt = belt_wrenches(points, note_v=v[self.nv:self.nv+3], note_w=nw, cradle_v=cv,
                             cradle_w=cw, belt_w=self.rotors[2:], cradle_basis=cr, forward=yr[:, 0],
                             shear_m=self.shear, motor_radius=b.nc["radiusM"], mu=b.nc["beltMu"],
                             slip_scale=b.nc["slipRegularizationMps"])
        floor, moment, _, _ = floor_wrench(pos, nr, v[self.nv:self.nv+3], nw, b.ring,
                                           b.tube, b.contract, b.nc["groundMu"],
                                           b.nc["slipRegularizationMps"])
        return belt, floor, moment

    def controls(self, belt, launcher_duty, feed_enabled):
        b, d = self.base, self.data
        yaw = d.xmat[self.chassis].reshape(3, 3)
        forward = d.qvel[:2] @ yaw[:2, 0]
        common = .8*(.32-forward) + .32*b.drive.c["motorK"]*b.drive.c["gearRatio"]/b.drive.c["wheelRadiusM"]/12
        diff = .65*(.24-d.qvel[2]) + .20*.24
        drive = np.clip([common-diff, common+diff], -.27, .27).tolist()
        q = [0., b.z0+float(d.qpos[3]), float(d.qpos[4])]
        vel = [0., float(d.qvel[3]), float(d.qvel[4])]
        terms = b.act.terms(q, vel)
        acc = [0., 225*(self.start_height-d.qpos[3])-30*d.qvel[3],
               225*(self.start_pitch-d.qpos[4])-30*d.qvel[4]]
        force = yaw.T @ belt["force"]
        moment = yaw.T @ belt["cradleMoment"]
        effort = np.array(terms["massMatrix"]) @ acc - terms["biasForce"] - [-force[0], -force[2], -moment[1]]
        actuator = []
        for i, gain in enumerate(terms["transmissionGains"]):
            amps = effort[i+1]/(gain*b.ac["motorK"])
            volts = terms["rotorOmegaRadps"][i]*b.ac["motorK"] + amps*b.ac["resistanceOhm"]
            actuator.append(float(np.clip(volts/self.bus, -.95, .95)))
        rpm = d.qvel[self.wheel_dofs]*60/(2*math.pi)
        if feed_enabled and self.feed_start is None and min(rpm) >= self.launcher["readyRpm"]:
            self.feed_start = d.time
        if self.feed_start is None or d.time + 1e-9 < self.feed_start:
            # Match the retained state's finite gravity-compensating hold.
            relative_v = (yaw.T @ (d.qvel[self.nv:self.nv+3]-np.r_[d.qvel[:2],0.])
                          - np.cross([0.,0.,d.qvel[2]], yaw.T @ (d.xpos[self.note]-d.xpos[self.chassis])))
            direction = np.array([math.cos(q[2]),0.,math.sin(q[2])])
            speed = float(relative_v @ direction)
            offset = float((yaw.T @ (d.xpos[self.note]-d.xpos[self.cradle])) @ direction)
            requested = b.nc["pieceMassKg"]*(9.80665*direction[2]-150*offset-24*speed)
            traction = max(1e-12, b.nc["beltMu"]*sum(belt["normal"]))
            slip = b.nc["slipRegularizationMps"]*math.atanh(float(np.clip(requested/traction,-.94,.94)))
            omega = -(speed+slip)/b.nc["radiusM"]
            amps = b.nc["kvRadpsPerV"]*(b.nc["rotorInertiaKgM2"]*150*(omega-self.rotors[2:])
                                        - b.nc["radiusM"]*requested/2)
            pinch = np.clip((self.rotors[2:]/b.nc["kvRadpsPerV"] + amps*b.nc["motorResistanceOhm"])/self.bus,-.025,.025)
        else:
            ramp = min(1., (d.time-self.feed_start+.01)/.1)
            pinch = [-self.launcher["pinchDuty"]*ramp]*2
        return drive, actuator, pinch, launcher_duty

    def ports(self, voltage, cmd):
        b, d = self.base, self.data
        drive, actuator, pinch, duty = cmd
        dr = b.drive.electrical(voltage, drive, modes=["drive"]*2,
                                state=dict(wheel=self.rotors[:2], t=d.time))
        act = b.act.electrical(voltage, actuator, ["drive"]*2,
                               [0., b.z0+float(d.qpos[3]), float(d.qpos[4])], [0., float(d.qvel[3]), float(d.qvel[4])])
        intake = [motor_electrical(u, voltage, w, b.nc) for u,w in zip(pinch,self.rotors[2:])]
        wheels = motor_query(self.launcher, d.qvel[self.wheel_dofs], duty, voltage)
        watts = dr["busPowerW"]+act["busPowerW"]+sum(x["terminalPowerW"] for x in intake)+wheels["busPowerW"]
        return watts, dr, act, intake, wheels

    def step(self, tick, launcher_duty, feed_enabled):
        b, m, d = self.base, self.model, self.data
        mujoco.mj_step1(m, d)
        belt, floor, floor_moment = self.contact()
        if tick or self.last_command is None:
            self.last_command = self.controls(belt, launcher_duty, feed_enabled)
        cmd = self.last_command
        # Solve V + R*P(V)/V = Voc on the operating branch. Every motor port
        # evaluates its own current limit and terminal power at the same voltage.
        v0 = self.bus
        for _ in range(35):
            power, drive, act, intake, wheels = self.ports(v0,cmd)
            residual = v0 + .02*power/v0 - 12.
            if abs(residual) < 1e-9:
                break
            v0 = float(v0 - .65*residual)
        else:
            raise ValueError("Shared supply failed to converge")
        if v0 > 12.000001 or v0 <= 0 or drive["infeasibleCurrentLimitVoltage"] or act["infeasibleCurrentLimitVoltage"]:
            raise ValueError("Motor supply outside modeled domain")
        self.bus = v0
        state = dict(x=d.qpos[0],y=d.qpos[1],heading=d.qpos[2],vx=d.qvel[0],
                     vy=d.qvel[1],yaw_rate=d.qvel[2],wheel=self.rotors[:2],t=d.time)
        rates = b.drive._rates(state,v0,cmd[0],["drive"]*2,b.loads,[0.,0.],0.)
        yaw = d.xmat[self.chassis].reshape(3,3)
        applied = d.qfrc_applied
        applied[:] = 0
        applied[:2] = yaw[:2,:2] @ rates["contactForceBodyN"]
        applied[2] = rates["contactYawTorqueNm"]
        applied[3:5] = act["generalizedMotorForce"][1:]
        applied[self.wheel_dofs] = [p["torqueNm"] for p in wheels["motors"]]
        if self.carries_note:
            mujoco.mj_applyFT(m,d,belt["force"]+floor,belt["noteMoment"]+floor_moment,d.xpos[self.note],self.note,applied)
            mujoco.mj_applyFT(m,d,-belt["force"],belt["cradleMoment"],d.xpos[self.cradle],self.cradle,applied)
        self.source_work += self.dt*12.*power/v0
        accel = (np.array([p["torqueNm"] for p in intake])+belt["rotor"])/b.nc["rotorInertiaKgM2"]
        self.rotors += self.dt*np.r_[rates["dw"],accel]
        self.shear += self.dt*belt["shearRate"]
        mujoco.mj_step2(m,d)
        contact = np.zeros(6)
        normal = 0.
        for i in range(d.ncon):
            mujoco.mj_contactForce(m,d,i,contact)
            normal += contact[0]
        return float(sum(belt["normal"])), normal
