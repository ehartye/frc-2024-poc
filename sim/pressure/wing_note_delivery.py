"""Lift a seated wing NOTE and drive to the previously attained shooting pose."""
import argparse
import json
import math
from pathlib import Path

import mujoco
import numpy as np

from second_wing_pickup import ReturnPlant, angle_error, build, build_seated
from wing_field_frame import WingFrame, piece_id
from run_axial_shear_lift import digest, load


class DeliveryPlant(ReturnPlant):
    def contact(self):
        belt, floor, moment = super().contact()
        self.floor = floor
        return belt, floor, moment

    def retained_command(self, belt, tick):
        b, d = self.base, self.data
        b.q = np.r_[d.qpos[:5], d.qpos[self.nq:]]
        b.v = np.r_[d.qvel[:5], d.qvel[self.nv:]]
        b.rotors = self.rotors
        b.bus = self.bus
        b.data.qpos[:] = b.q
        mujoco.mj_kinematics(b.robot, b.data)
        body = d.xmat[self.chassis].reshape(3,3)
        nr = d.xmat[self.note].reshape(3,3)
        local = body.T @ (d.xpos[self.note]-d.xpos[self.chassis])
        result = dict(belt,centerRobot=local,normalRobot=body.T@nr[:,2],floor=self.floor)
        return b.command(tick, result, 1, 0, 0., 0.)

    def controls(self, belt, launcher_duty, feed_enabled):
        b, d = self.base, self.data
        _, heading, forward = self.observe()
        body = d.xmat[self.chassis].reshape(3,3)
        elapsed = d.time-self.seated
        demand = self.retained_command(belt,round(elapsed/.01))
        self.brakes = demand["pinchBrakeFraction"]
        target = self.shoot_position if self.approach_position is None else self.approach_position
        if self.waypoints:
            target=self.waypoints[0]
        delta = target-d.qpos[:2]
        distance = np.linalg.norm(delta)
        if self.waypoints and distance<.14:
            self.waypoints.pop(0)
        elif (not self.waypoints and self.approach_position is not None and distance<.035 and
                abs(angle_error(self.shoot_heading, heading))<.035 and abs(d.qvel[2])<.05):
            self.approach_position = None
            delta = self.shoot_position-d.qpos[:2]
            distance = np.linalg.norm(delta)
        path_heading = math.atan2(delta[1],delta[0]) if distance > .035 else self.shoot_heading
        error = angle_error(path_heading, heading)
        accel_ratio=self.turn_accel/.7
        stopping_speed = math.sqrt(2*.58*accel_ratio*abs(error))
        desired_turn = math.copysign(min(3*abs(error),self.turn_rate,stopping_speed),error) if elapsed>.65 else 0.
        # The floor resists sideways motion until the lift supports the ring.
        # Limit angular acceleration to avoid driving it into the tread backing.
        increment=.007*accel_ratio
        self.turn_reference += float(np.clip(desired_turn-self.turn_reference,-increment,increment))
        want_turn = self.turn_reference
        want_speed = min(self.transit_speed, 1.7*distance)*max(0.,math.cos(error))**6
        if self.transit_speed>.6:
            # Bound loaded acceleration while lifting and negotiating corners.
            want_speed=min(want_speed,self.speed_reference+.012)
            self.speed_reference=want_speed
        if distance <= .035:
            want_speed = 1.7*delta@body[:2,0]
        common = .8*(want_speed-forward)+want_speed*b.drive.c["motorK"]*b.drive.c["gearRatio"]/b.drive.c["wheelRadiusM"]/12
        diff = .65*(want_turn-d.qvel[2])+.21*want_turn
        drive = np.clip([common-diff,common+diff],-.5,.5).tolist()
        return drive,demand["actuator"],demand["pinch"],launcher_duty


def tail(folder, receipt):
    path=folder/"trace.jsonl"
    if digest(path)!=receipt["trace"]:
        raise ValueError("Trace differs from its continuation receipt")
    with path.open("rb") as stream:
        for line in stream:
            last=line
    row=json.loads(last)
    if abs(row["t"]-receipt["end"])>1e-8:
        raise ValueError("Missing terminal state")
    return row


def restore(plant, row):
    d=plant.data
    d.qpos[:]=row["q"]
    d.qvel[:]=row["v"]
    d.time=row["t"]
    plant.rotors=np.asarray(row["rotor"])
    plant.shear=np.asarray(row["shear"])
    plant.bus=row["busV"]
    plant.source_work=row["sourceJ"]
    plant.last_command=None
    mujoco.mj_forward(plant.model,d)


def run(args):
    if args.horizon<=0 or not math.isfinite(args.horizon) or args.trace_stride<1 or not math.isfinite(args.forward_offset):
        raise ValueError("Invalid delivery study parameters")
    if not args.acquisition:
        p, pins = build_seated(args, DeliveryPlant, 32)
        frame = WingFrame(args.first_mark, args.field_yaw_deg)
        x, y, heading = args.field_station
        p.shoot_position = frame.to_local([x, y, 0.])[:2]
        p.shoot_heading = math.radians(heading - frame.yaw_degrees)
        rel, _, _ = p.observe()
        row = dict(q=p.data.qpos.tolist(), v=p.data.qvel.tolist(), t=p.data.time, relative=rel)
        return deliver(args, p, row, pins["stop"], pins["native"], [args.first_mark])
    got=load(args.acquisition/"result.json")
    collected=got.get("collectedMarks",[args.first_mark,"middle"])
    args.next_mark=collected[-1]
    p,last,release=build(args, DeliveryPlant)
    if (got["outcome"]!="SEATED" or got["firstReleaseTrace"]!=release["traceSha256"]
            or (got.get("firstMark","inner"),got.get("fieldYawDeg",90.))!=(args.first_mark,args.field_yaw_deg)):
        raise ValueError("Expected continuous second-wing pickup")
    row=tail(args.acquisition,got)
    restore(p,row)
    p.shoot_position=np.array(last["q"][:2])
    p.shoot_heading=last["q"][2]
    p.shoot_position += args.forward_offset*np.array([math.cos(p.shoot_heading),math.sin(p.shoot_heading)])
    if args.field_station:
        x, y, heading = args.field_station
        frame = WingFrame(args.first_mark, args.field_yaw_deg)
        p.shoot_position = frame.to_local([x, y, 0.])[:2]
        p.shoot_heading = math.radians(heading - frame.yaw_degrees)
    return deliver(args, p, row, got["trace"], release["nativeSha256"], collected)


def deliver(args, p, row, source, native, collected):
    p.seated=p.data.time
    cad=p.base.cad
    # The carry controller returns to its registered handoff datum. A previous
    # shot may have moved away from that datum when aligning its own NOTE.
    target_station=[(cad["handoffCenterZ"]-cad["floorCenterZ"])/1000.,math.radians(cad["pitchDeg"])]
    if (not 0.<args.transit_speed<=4. or not np.isfinite(args.via).all()
            or not 0.<args.turn_rate<=2.5 or not 0.<args.turn_accel<=3.):
        raise ValueError("Invalid loaded transit command")
    p.transit_speed=args.transit_speed
    p.turn_rate=args.turn_rate
    p.turn_accel=args.turn_accel
    p.speed_reference=max(0.,p.observe()[2])
    frame=WingFrame(args.first_mark,args.field_yaw_deg)
    p.waypoints=[frame.to_local([x,y,0.])[:2] for x,y in args.via]
    if not math.isfinite(args.final_straight) or args.final_straight<0.:
        raise ValueError("Invalid final straight approach length")
    direction=np.array([math.cos(p.shoot_heading),math.sin(p.shoot_heading)])
    p.approach_position=p.shoot_position-args.final_straight*direction if args.final_straight else None
    p.turn_reference=float(p.data.qvel[2])
    p.base.initial={"note":{"p":[row["relative"][0]]},"chassis":{"x":0.}}
    period=round(.01/args.dt)
    args.output.mkdir(exist_ok=False)
    trace=args.output/"trace.jsonl"
    hold=None
    min_load=math.inf
    outcome="HORIZON"
    with trace.open("x") as stream:
        for i in range(round(args.horizon/args.dt)):
            d=p.data
            try:
                _, wheel=p.step(i%period==0,0.,False)
            except (ValueError,ArithmeticError,RuntimeError) as e:
                outcome=str(e);break
            if wheel>1e-9:
                raise ValueError("Reduced wheel quadrature encountered the raised NOTE")
            min_load=min(min_load,min(p.last_loads))
            rel,yaw,forward=p.observe()
            pos_error=np.linalg.norm(d.qpos[:2]-p.shoot_position)
            heading_error=angle_error(p.shoot_heading,yaw)
            station=np.linalg.norm(d.qpos[3:5]-target_station)
            ready=(d.time-p.seated>5.25 and pos_error<.035 and abs(heading_error)<.012
                   and abs(forward)<.035 and abs(d.qvel[2])<.035 and station<.005 and min(p.last_loads)>.02)
            if ready:
                if hold is None:hold=d.time
                if d.time-hold >= .15:outcome="AT_SHOOTING_POSE"
            else:hold=None
            if i%args.trace_stride==0 or outcome=="AT_SHOOTING_POSE" or i+1==round(args.horizon/args.dt):
                out=dict(t=d.time,q=d.qpos.tolist(),v=d.qvel.tolist(),rotor=p.rotors.tolist(),
                         shear=p.shear.tolist(),busV=p.bus,sourceJ=p.source_work,loads=p.last_loads.tolist(),
                         relative=rel,forward=forward,positionError=pos_error,headingError=heading_error)
                stream.write(json.dumps(out,allow_nan=False)+"\n")
            if outcome=="AT_SHOOTING_POSE":break
    if any(w.number for w in p.data.warning):outcome="MUJOCO_WARNING"
    result=dict(schema="wing-note-delivery/1",nativeSha256=native,acquisitionTrace=source,
                trace=digest(trace),initialQ=row["q"],initialV=row["v"],start=row["t"],end=p.data.time,
                outcome=outcome,minimumSideLoadN=min_load if math.isfinite(min_load) else None,
                dt=args.dt,grid=args.grid,traceStride=args.trace_stride,shootPosition=p.shoot_position.tolist(),
                shootHeading=p.shoot_heading,forwardOffsetM=0. if args.field_station else args.forward_offset,
                pieceId=piece_id(collected[-1]),collectedMarks=collected,
                fieldStation=args.field_station,finalStraightM=args.final_straight,
                viaFieldXY=args.via,transitSpeedMps=args.transit_speed,
                turnRateRadPerS=args.turn_rate,turnAccelRadPerS2=args.turn_accel,
                firstMark=args.first_mark,fieldYawDeg=args.field_yaw_deg)
    (args.output/"result.json").write_text(json.dumps(result,indent=2,allow_nan=False)+"\n")
    print(outcome,p.data.time,flush=True)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ("pickup","archive","output"):
        parser.add_argument("--"+name,type=Path,required=True)
    parser.add_argument("--acquisition",type=Path)
    parser.add_argument("--field-station",type=float,nargs=3,metavar=("X","Y","HEADING_DEG"))
    parser.add_argument("--release",type=Path)
    parser.add_argument("--dt",type=float,choices=(.001,.0005,.00025),default=.0005)
    parser.add_argument("--grid",type=int,choices=(128,256,512,1024,2048),default=256)
    parser.add_argument("--horizon",type=float,default=12.)
    parser.add_argument("--trace-stride",type=int,default=1)
    parser.add_argument("--forward-offset",type=float,default=.11,help="move the shooting reference forward in metres")
    parser.add_argument("--final-straight",type=float,default=0.,help="Align this many metres behind the shooting position")
    parser.add_argument("--via",type=float,nargs=2,action="append",default=[],metavar=("X","Y"),help="Traverse a field waypoint before final alignment")
    parser.add_argument("--transit-speed",type=float,default=.6,help="Loaded travel speed limit in m/s")
    parser.add_argument("--turn-rate",type=float,default=.95,help="Loaded turning reference limit in rad/s")
    parser.add_argument("--turn-accel",type=float,default=.7,help="Loaded turning reference slew in rad/s²")
    parser.add_argument("--first-mark",choices=("inner","middle","outer"),default="inner")
    parser.add_argument("--field-yaw-deg",type=float,default=90.)
    args = parser.parse_args()
    if not args.acquisition and not args.field_station:
        parser.error("A first floor pickup needs a field shooting station")
    if args.acquisition and not args.release:
        parser.error("--acquisition requires its previous --release")
    run(args)
