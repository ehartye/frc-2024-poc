"""Carry the moving robot from a checked release to the next marked floor NOTE."""
import argparse
import json
import math
from pathlib import Path

import mujoco
import numpy as np

from launcher_handoff import RaisedFeed
from native_launcher_quadrature import motor_query
from retained_route import short_brake, target
from run_axial_shear_lift import ROOT, EVIDENCE, load, digest
from signed_pressure_acquisition import motor_electrical
from wing_field_frame import SUPPLY_MARKS, WingFrame, WING_MARKS_INCHES, piece_id
from field_transit import TransitPath
from turning_plant import TurningPlant


MIDDLE_WING = np.array([.658+57*.0254, 0.])  # In the original pickup's local field frame.


def angle_error(target, current):
    return math.atan2(math.sin(target-current), math.cos(target-current))


class ReturnPlant(RaisedFeed):
    def resume(self, last):
        # Only the robot and its own control/rotor state transfer. The released
        # NOTE continues in the separate field solve while this floor NOTE rests.
        d = self.data
        if (last["beltNormalN"] or last["wheelNormalN"] or any(last["shearM"])
                or d.time != last["time"]):
            raise ValueError("A previous NOTE is still constrained or its clock changed")
        d.qpos[:self.nq] = last["q"][:-7]
        d.qvel[:self.nv] = last["v"][:-6]
        self.source_work = last["sourceJ"]
        self.begin_pickup()

    def begin_pickup(self, *, starts_lowered=False):
        d = self.data
        self.starts_lowered = starts_lowered
        self.contact_steering = False
        self.encoder_steering = False
        self.release = d.time
        self.turn_heading = None
        self.pinched_since = None
        self.capture_at = None
        self.brakes = [0., 0.]
        self.brake_power = 0.
        self.last_loads = [0., 0.]
        self.departure_xy = d.qpos[:2].copy()
        self.departure_heading = float(d.qpos[2])
        self.retreat_distance = 0.
        self.approach_heading = None
        self.approach_waypoint = None
        self.capture_standoff = 0.
        self.waypoints=[]
        self.reverse_transit=False
        self.transit_speed=1.1
        self.capture_speed=.26
        self.transit_path=None
        mujoco.mj_forward(self.model, d)

    def contact(self):
        belt, floor, moment = super().contact()
        self.last_loads = belt["normal"].copy()
        return belt, floor, moment

    def controls(self, belt, launcher_duty, feed_enabled):
        b, d = self.base, self.data
        rel, heading, forward = self.observe()
        delta = self.mark_xy - d.qpos[:2]
        distance = np.linalg.norm(delta)
        bearing = math.atan2(delta[1], delta[0])
        if self.turn_heading is None and min(self.last_loads) > .02:
            self.turn_heading = d.qpos[2]
        target_heading = bearing if self.turn_heading is None else self.turn_heading
        error = angle_error(target_heading, d.qpos[2])
        want_turn = np.clip(4*error, -1.65, 1.65)
        approach = min(self.transit_speed, max(0., 1.7*(distance-.178)))
        capture_gain=1.5*(self.capture_speed/.26)
        if distance < .65 or self.turn_heading is not None:
            approach = min(self.capture_speed, max(0., capture_gain*(rel[0]-.178)))
        elapsed = d.time-self.release
        want_speed = approach*max(0., math.cos(error))**6
        if self.waypoints:
            offset=self.waypoints[0]-d.qpos[:2]
            gap=np.linalg.norm(offset)
            if self.transit_path is not None:
                point,gap=self.transit_path.target(d.qpos[:2])
                offset=point-d.qpos[:2]
            course=math.atan2(offset[1],offset[0])
            error=angle_error(course+math.pi if self.reverse_transit else course,heading)
            want_turn=np.clip(4*error,-1.65,1.65)
            want_speed=min(self.transit_speed,2*gap)*max(0.,math.cos(error))**6
            if self.reverse_transit:
                want_speed=-want_speed
            if gap<.14:
                if self.transit_path is not None:
                    self.waypoints.clear()
                else:
                    self.waypoints.pop(0)
        elif self.approach_waypoint is not None:
            offset=self.approach_waypoint-d.qpos[:2]
            gap=np.linalg.norm(offset)
            target_heading=math.atan2(offset[1],offset[0]) if gap>.045 else self.approach_heading
            error=angle_error(target_heading,heading)
            want_turn=np.clip(4*error,-1.65,1.65)
            want_speed=min(self.transit_speed,1.7*gap)*max(0.,math.cos(error))**6 if gap>.045 else 0.
            if gap<=.045 and abs(error)<.035 and abs(d.qvel[2])<.07:
                self.approach_waypoint=None
        elif self.capture_standoff:
            axis=np.array([math.cos(self.approach_heading),math.sin(self.approach_heading)])
            distance_left=delta@axis-self.capture_standoff
            want_speed=min(want_speed,float(np.clip(2.5*distance_left,-.08,1.1)))
        if not self.starts_lowered and elapsed < 2.8 and distance < .72:
            want_speed = 0.
        if self.retreat_distance:
            axis=np.array([math.cos(self.departure_heading),math.sin(self.departure_heading)])
            remaining=self.retreat_distance+(d.qpos[:2]-self.departure_xy)@axis
            if remaining>.015:
                want_turn=np.clip(3*angle_error(self.departure_heading,d.qpos[2]),-.4,.4)
                want_speed=-min(.55,max(.06,2*remaining))
            else:
                self.retreat_distance=0.
        common = .8*(want_speed-forward) + want_speed*b.drive.c["motorK"]*b.drive.c["gearRatio"]/b.drive.c["wheelRadiusM"]/12
        diff = .65*(want_turn-d.qvel[2]) + .21*want_turn
        drive = np.clip([common-diff, common+diff], -.65, .65).tolist()
        if self.contact_steering or self.encoder_steering:
            if self.encoder_steering:
                # Sample the intake rotor velocities in whole rpm on the controller tick.
                negative_rpm, positive_rpm = np.rint(self.rotors[2:] * (60 / (2*np.pi)))
                correction = .008 * (negative_rpm-positive_rpm)
            else:
                negative_y, positive_y = self.last_loads
                correction = .12 * (positive_y-negative_y)
            differential = np.clip(correction, -.14, .14)
            drive = np.clip(np.asarray(drive) + [-differential, differential], -.65, .65).tolist()
        # Retrace the established pitch/lift clearance route before pickup.
        # Finite motors track these references while the chassis turns.
        lowering_target = 0. if self.starts_lowered else max(0., 500*(1-elapsed/2.8))
        zref, pref = target(b, lowering_target)
        z = zref[0]-b.z0
        pitch = pref[0]
        actuator = self.actuator_command(belt, z, pitch)
        traction_speed = min(self.capture_speed, max(-.04, capture_gain*(rel[0]-.178))) if self.turn_heading is not None else .22
        goal = traction_speed/b.nc["radiusM"]
        pinch = []
        self.brakes = []
        for w in self.rotors[2:]:
            amps = b.nc["kvRadpsPerV"]*b.nc["rotorInertiaKgM2"]*115*(goal-w)
            emf = w/b.nc["kvRadpsPerV"]
            voltage = emf + amps*b.nc["motorResistanceOhm"]
            brake = amps*emf < 0 and abs(amps*b.nc["motorResistanceOhm"]) <= abs(emf)
            self.brakes.append(min(1., abs(amps*b.nc["motorResistanceOhm"]/emf)) if brake else 0.)
            pinch.append(0. if brake else float(np.clip(voltage/self.bus, -1., 1.)))
        return drive, actuator, pinch, launcher_duty

    def actuator_command(self, belt, z, pitch):
        b, d = self.base, self.data
        yaw = d.xmat[self.chassis].reshape(3, 3)
        coord = [0., float(b.z0+d.qpos[3]), float(d.qpos[4])]
        speed = [0., float(d.qvel[3]), float(d.qvel[4])]
        terms = b.act.terms(coord, speed)
        acc = [0., 225*(z-d.qpos[3])-30*d.qvel[3], 225*(pitch-d.qpos[4])-30*d.qvel[4]]
        force = yaw.T @ belt["force"]
        moment = yaw.T @ belt["cradleMoment"]
        effort = np.asarray(terms["massMatrix"]) @ acc - terms["biasForce"] - [-force[0], -force[2], -moment[1]]
        duty = []
        for i, gain in enumerate(terms["transmissionGains"]):
            amps = effort[i+1]/(gain*b.ac["motorK"])
            volts = terms["rotorOmegaRadps"][i]*b.ac["motorK"] + amps*b.ac["resistanceOhm"]
            duty.append(float(np.clip(volts/self.bus, -.95, .95)))
        return duty

    def ports(self, voltage, cmd):
        b, d = self.base, self.data
        drive, actuator, pinch, duty = cmd
        dr = b.drive.electrical(voltage, drive, modes=["drive"]*2, state=dict(wheel=self.rotors[:2], t=d.time))
        act = b.act.electrical(voltage, actuator, ["drive"]*2,
                               [0., float(b.z0+d.qpos[3]), float(d.qpos[4])],
                               [0., float(d.qvel[3]), float(d.qvel[4])])
        intake = [short_brake(w, fraction, b.nc) if fraction else motor_electrical(u, voltage, w, b.nc)
                  for w, fraction, u in zip(self.rotors[2:], self.brakes, pinch)]
        wheels = motor_query(self.launcher, d.qvel[self.wheel_dofs], duty, voltage)
        # Dissipative controller hypothesis, separately for each motor. At a
        # generating operating point the drive absorbs the negative terminal
        # power locally. Applied torque still comes from the limited current.
        ports = [x["busPowerW"] for x in dr["perMotor"]+act["motors"]]
        self.brake_power = sum(max(0., -p) for p in ports)
        watts = sum(max(0., p) for p in ports)+sum(x["terminalPowerW"] for x in intake)+wheels["busPowerW"]
        return watts, dr, act, intake, wheels

    def observe(self):
        d = self.data
        yaw = d.qpos[2]
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = d.qpos[self.nq:self.nq+2]-d.qpos[:2]
        return [c*dx+s*dy, -s*dx+c*dy], yaw, c*d.qvel[0]+s*d.qvel[1]


def build_seated(args, plant_type, segments):
    """Lift the first acquired floor NOTE into the combined drive and launcher model."""
    pins = load(EVIDENCE/"turning-lift-20261118/refined-end-to-end.json")["freshInputDigests"]
    stop_path = args.pickup/"stop/receipt.json"
    if (digest(args.archive) != pins["native"] or digest(stop_path) != pins["stop"]
            or digest(args.pickup/"ground/receipt.json") != pins["ground"]):
        raise ValueError("Expected the registered floor acquisition and braking trajectory")
    packet = load(stop_path)
    acquired = packet["final"]
    original = TurningPlant(acquired, args.grid)
    state = dict(q=original.q, v=original.v, rotor=original.rotors, shear=original.shear,
                 clock=acquired["timeS"], busV=packet["trace"][-1]["busV"])
    source = load(ROOT/"sim/mujoco/evidence/launcher-quadrature-verification-20261118.json")
    plant = plant_type(acquired, state, args.dt, args.grid, segments,
                       source["runs"]["grid-16384"]["parameters"])
    plant.brakes = [0., 0.]
    plant.source_work = acquired["ledger"]["sourceWork"]
    return plant, pins


def build(args, plant_type=ReturnPlant, segments=32):
    receipt = load(args.release/"result.json")
    comparison = load(EVIDENCE/"speaker-counter-20261118.json")
    if (digest(args.release/"trace.jsonl") != receipt["traceSha256"]
            or receipt["outcome"] != "UPWARD_FREE_RELEASE"
            or digest(args.archive) != receipt["nativeSha256"]
            or receipt["nativeSha256"] != comparison["nativeArchive"]):
        raise ValueError("Expected an attained release from the registered native robot")
    if receipt["traceSha256"] != comparison["runs"]["fine"]["releaseTrace"]:
        if (receipt.get("schema")!="wing-raised-feed/1" or
                (receipt.get("firstMark"),receipt.get("fieldYawDeg"))!=(args.first_mark,args.field_yaw_deg)):
            raise ValueError("Previous release belongs to a different field route")
    with (args.release/"trace.jsonl").open("rb") as stream:
        for line in stream:
            last = line
    last = json.loads(last)
    if (last["beltNormalN"] or last["wheelNormalN"] or any(last["shearM"])
            or abs(last["time"]-receipt["end"])>1e-8):
        raise ValueError("Invalid release boundary")
    stop = args.pickup/"stop/receipt.json"
    reference = load(EVIDENCE/"turning-lift-20261118/refined-end-to-end.json")
    if digest(stop) != reference["freshInputDigests"]["stop"]:
        raise ValueError("Pickup packet mismatch")
    source = load(ROOT/"sim/mujoco/evidence/launcher-quadrature-verification-20261118.json")
    calibration = source["runs"]["grid-16384"]["parameters"]
    config = load(ROOT/"sim/component-launcher/six-drive-config.json")
    floor = load(EVIDENCE/"native-lift-20261118/native-pressure-lift-contract.json")
    z = .0254 - config["pieceMassKg"]*9.80665/floor["floorStiffnessNpm"]
    frame = WingFrame(args.first_mark,args.field_yaw_deg)
    state = dict(q=last["q"][:5],v=last["v"][:5],clock=last["time"],
                 rotor=last["externalRotors"], shear=last["shearM"], busV=last["busV"])
    if plant_type.carries_note:
        next_mark = args.next_mark
        if next_mark in receipt.get("collectedMarks",[args.first_mark]):
            raise ValueError("Target floor NOTE already collected")
        mark_xy = frame.to_local(frame.mark(next_mark))[:2]
        physical_xy = mark_xy
        if getattr(args, "placement_roster", None):
            from speaker_trial_inputs import local_offset, read_attempt
            setup = read_attempt(args.placement_roster, args.attempt)
            if (args.first_mark, args.field_yaw_deg) != ("middle", 245.):
                raise ValueError("The placement roster uses the middle mark and 245 degree registration")
            if receipt.get("attempt") != args.attempt or receipt.get("rosterSha256") != digest(args.placement_roster):
                raise ValueError("Cannot mix physical setups within an attempt")
            physical_xy = mark_xy + local_offset(setup, next_mark)
        state["q"] += [*physical_xy, z, 1., 0., 0., 0.]
        state["v"] += [0.]*6
    plant = plant_type(load(stop)["final"], state, args.dt, args.grid, segments, calibration)
    plant.resume(last)
    if plant_type.carries_note:
        plant.mark_xy = mark_xy
    return plant, last, receipt


def advance_pickup(args, plant):
    d = plant.data
    args.output.mkdir(exist_ok=False)
    period = round(.01/args.dt)
    first_touch = None
    worst_brake = 0.
    minimum_bus = plant.bus
    outcome = "HORIZON"
    trace = args.output/"trace.jsonl"
    with trace.open("x") as f:
        for step in range(round(args.horizon/args.dt)):
            tick = step % period == 0
            before = d.time
            try:
                _, wheel = plant.step(tick, 0., False)
            except (ValueError, ArithmeticError, RuntimeError) as e:
                outcome = str(e)
                break
            if wheel > 1e-9:
                raise ValueError("Reduced wheel quadrature encountered the second NOTE")
            minimum_bus = min(minimum_bus, plant.bus)
            worst_brake = max(worst_brake,plant.brake_power)
            loads = plant.last_loads
            rel, yaw, forward = plant.observe()
            if min(loads) > .02 and first_touch is None:
                first_touch = before
            seated = (min(loads)>.02 and abs(rel[0]-.178)<.012 and abs(rel[1])<.004
                      and np.linalg.norm(d.qvel[plant.nv:plant.nv+2]-d.qvel[:2])<.022)
            if seated:
                if plant.pinched_since is None:
                    plant.pinched_since = d.time
            else:
                plant.pinched_since = None
            if plant.pinched_since is not None and d.time-plant.pinched_since > .15:
                plant.capture_at = d.time
                outcome = "SEATED"
            if step % args.trace_stride == 0 or outcome=="SEATED" or step+1 == round(args.horizon/args.dt):
                row = dict(t=d.time,q=d.qpos.tolist(),v=d.qvel.tolist(),rotor=plant.rotors.tolist(),
                           shear=plant.shear.tolist(),busV=plant.bus,sourceJ=plant.source_work,loads=loads.tolist(),
                           relative=rel,forward=forward,brakePower=plant.brake_power)
                f.write(json.dumps(row,allow_nan=False)+"\n")
            if outcome=="SEATED":break
    if any(w.number for w in d.warning):outcome="MUJOCO_WARNING"
    return dict(trace=digest(trace),end=d.time,firstBilateral=first_touch,seated=plant.capture_at,
                minimumBusV=minimum_bus,peakLocalBrakingW=worst_brake,outcome=outcome,dt=args.dt,grid=args.grid)


def run(args):
    if args.horizon<=0 or not math.isfinite(args.horizon) or args.trace_stride<1:
        raise ValueError("Positive horizon and trace stride required")
    plant, last, release = build(args)
    if "attempt" in release and not getattr(args, "placement_roster", None):
        raise ValueError("Continuation of a varied mission requires its original placement roster")
    if not math.isfinite(args.retreat) or args.retreat<0.:
        raise ValueError("Invalid departure distance")
    plant.retreat_distance=args.retreat
    frame=WingFrame(args.first_mark,args.field_yaw_deg)
    plant.waypoints=[frame.to_local([x,y,0.])[:2] for x,y in args.via]
    plant.reverse_transit=args.reverse_transit
    if plant.reverse_transit and not plant.waypoints:
        raise ValueError("Reverse transit requires at least one approach waypoint")
    if not 0.<args.transit_speed<=4. or not np.isfinite(args.via).all():
        raise ValueError("Invalid transit command")
    plant.transit_speed=args.transit_speed
    if not 0.<args.capture_speed<=1. or not math.isfinite(args.lookahead) or args.lookahead<0.:
        raise ValueError("Invalid transit lookahead or capture speed")
    plant.capture_speed=args.capture_speed
    if args.lookahead:
        if not plant.waypoints:
            raise ValueError("A lookahead transit needs at least one field waypoint")
        plant.transit_path=TransitPath([plant.data.qpos[:2],*plant.waypoints],args.lookahead)
    if args.approach_heading is not None:
        if not math.isfinite(args.approach_heading) or not 0.<=args.capture_standoff<.35:
            raise ValueError("Invalid floor approach heading or pickup standoff")
        plant.approach_heading=math.radians(args.approach_heading-args.field_yaw_deg)
        direction=np.array([math.cos(plant.approach_heading),math.sin(plant.approach_heading)])
        plant.approach_waypoint=plant.mark_xy-.95*direction
        plant.capture_standoff=args.capture_standoff
    elif args.capture_standoff:
        raise ValueError("A pickup standoff needs a field approach heading")
    plant.contact_steering = getattr(args, "contact_steering", False)
    plant.encoder_steering = getattr(args, "encoder_steering", False)
    initial_note_xy = plant.data.qpos[plant.nq:plant.nq+2].tolist()
    stats = advance_pickup(args, plant)
    result=dict(schema="second-wing-acquisition/1", firstReleaseTrace=release["traceSha256"],
                nativeSha256=release["nativeSha256"], trace=stats.pop("trace"), initialRobotQ=last["q"][:-7],
                initialRobotV=last["v"][:-6], initialRotors=last["externalRotors"], initialBusV=last["busV"],
                start=last["time"], **stats,
                traceStride=args.trace_stride,firstMark=args.first_mark,fieldYawDeg=args.field_yaw_deg,
                retreatM=args.retreat,approachHeadingDeg=args.approach_heading,captureStandoffM=args.capture_standoff,
                viaFieldXY=args.via,transitSpeedMps=args.transit_speed,reverseTransit=args.reverse_transit,
                waitingNoteLocalXY=plant.mark_xy.tolist(),pieceId=piece_id(args.next_mark),
                collectedMarks=[*release.get("collectedMarks",[args.first_mark]),args.next_mark])
    if getattr(args, "placement_roster", None):
        result.update(attempt=args.attempt, rosterSha256=digest(args.placement_roster),
                      waitingNoteLocalXY=initial_note_xy, targetLocalXY=plant.mark_xy.tolist())
    if plant.contact_steering:
        result["contactSteering"] = True
    if plant.encoder_steering:
        result["encoderSteering"] = True
    if args.lookahead or args.capture_speed!=.26:
        result.update(lookaheadM=args.lookahead,captureSpeedMps=args.capture_speed)
    (args.output/"result.json").write_text(json.dumps(result,indent=2,allow_nan=False)+"\n")
    print(json.dumps(result),flush=True)


if __name__ == "__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for name in ("release","pickup","archive","output"):
        p.add_argument("--"+name,type=Path,required=True)
    p.add_argument("--dt",type=float,choices=(.001,.0005,.00025),default=.0005)
    p.add_argument("--grid",type=int,choices=(128,256,512,1024,2048),default=256)
    p.add_argument("--trace-stride",type=int,default=1)
    p.add_argument("--first-mark",choices=WING_MARKS_INCHES,default="inner")
    p.add_argument("--placement-roster",type=Path,help="Physical NOTE setup in the registered blue field frame")
    p.add_argument("--attempt",help="Attempt ID in the physical setup roster")
    steering = p.add_mutually_exclusive_group()
    steering.add_argument("--encoder-steering",action="store_true",help="Steer from differential measured intake rotor rpm")
    steering.add_argument("--contact-steering",action="store_true",help="Use modeled differential intake loads during pickup")
    p.add_argument("--next-mark",choices=SUPPLY_MARKS,default="middle")
    p.add_argument("--retreat",type=float,default=0.,help="Back away from the Speaker before turning, in metres")
    p.add_argument("--approach-heading",type=float,help="Align before approaching the floor mark at this field heading in degrees")
    p.add_argument("--capture-standoff",type=float,default=0.,help="Stop the chassis this far short of the mark while the intake draws in the NOTE")
    p.add_argument("--via",type=float,nargs=2,action="append",default=[],metavar=("X","Y"),help="Traverse a field waypoint before pickup alignment")
    p.add_argument("--reverse-transit",action="store_true",help="Drive backward through the approach waypoints, then collect forward")
    p.add_argument("--transit-speed",type=float,default=1.1,help="Empty travel speed limit in m/s")
    p.add_argument("--lookahead",type=float,default=0.,help="Lookahead distance on the approach waypoints in metres")
    p.add_argument("--capture-speed",type=float,default=.26,help="Limit for the closure speed and first bilateral intake in m/s")
    p.add_argument("--field-yaw-deg",type=float,default=90.)
    p.add_argument("--horizon",type=float,default=11.)
    run(p.parse_args())
