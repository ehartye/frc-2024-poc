"""Feed the same retained wing NOTE from the attained second shooting pose."""
import argparse
import json
from pathlib import Path

import numpy as np

from launcher_handoff import RaisedFeed
from mission_runtime import runtime_versions
from run_axial_shear_lift import digest, load
from second_wing_pickup import angle_error, build_seated
from wing_note_delivery import DeliveryPlant, restore, tail


class ShotPlant(DeliveryPlant):
    def controls(self, belt, launcher_duty, feed_enabled):
        if self.align_seconds:
            phase = min(1., (self.data.time-self.start_time)/self.align_seconds)
            fraction = phase**2*(3.-2.*phase)
            self.start_height,self.start_pitch = self.station_pose + fraction*self.trim_pose
            feed_enabled = feed_enabled and phase>=1. and min(belt["normal"])>.02
        if (self.pitch_trim or self.height_trim) and self.feed_start is None:
            # The adjustment is a finite motor command beginning from the
            # attained station state. Let the mechanism settle before feeding.
            at_pitch=(abs(self.data.qpos[4]-self.start_pitch)<np.deg2rad(.012)
                      and abs(self.data.qvel[4])<np.deg2rad(.15))
            at_height = (not self.height_trim or
                         (abs(self.data.qpos[3]-self.start_height)<.000075 and abs(self.data.qvel[3])<.001))
            self.pose_ticks=self.pose_ticks+1 if at_pitch and at_height else 0
            feed_enabled=feed_enabled and self.pose_ticks>=3
        if self.target_rpm is not None:
            # Positive drive/coast servo. The motor port still enforces its phase
            # current limit on the shared bus at every integration step.
            rpm=self.data.qvel[self.wheel_dofs]*60/(2*np.pi)
            reference=self.target_rpm*2*np.pi/60
            kv=self.launcher["backEmfRadPerSV"]
            resistance=self.launcher["windingResistanceOhm"]
            inertia=self.launcher["rotatingInertiaEachKgM2"]
            speed=min(rpm)*2*np.pi/60
            amps=inertia*kv*(reference-speed)/.025
            launcher_duty=float(np.clip((speed/kv+resistance*amps)/self.bus,0.,launcher_duty))
            ready=np.max(np.abs(rpm-self.target_rpm))<=.025*self.target_rpm
            self.ready_ticks=self.ready_ticks+1 if ready else 0
            feed_enabled=feed_enabled and (self.feed_start is not None or self.ready_ticks>=3)
        _, actuator, pinch, duty = RaisedFeed.controls(self,belt,launcher_duty,feed_enabled)
        if self.align_seconds:
            if self.feed_start is None:
                hold=self.retained_command(belt,500)
                self.brakes=hold["pinchBrakeFraction"]
                pinch=hold["pinch"]
            else:
                self.brakes=[0.,0.]
        _, heading, speed = self.observe()
        direction = np.array([np.cos(heading), np.sin(heading)])
        speed_ref = (self.shoot_position-self.data.qpos[:2])@direction
        common = .8*(speed_ref-speed)
        turn = 3*angle_error(self.shoot_heading,heading)
        diff = .65*(turn-self.data.qvel[2]) + .21*turn
        return np.clip([common-diff,common+diff],-.3,.3).tolist(),actuator,pinch,duty


def run(args):
    if not 0.<args.duty<=1.:
        raise ValueError("Invalid launcher duty")
    phase = args.ring_phase_deg
    if not 0. <= phase < 360. / args.segments:
        raise ValueError("Use a ring phase within one sampling interval")
    p, pins = build_seated(args, ShotPlant, args.segments, ring_phase_deg=phase)
    p.target_rpm=args.target_rpm
    p.align_seconds=args.align_seconds
    if not np.isfinite(p.align_seconds) or not 0.<=p.align_seconds<=2.:
        raise ValueError("Invalid loaded alignment duration")
    p.ready_ticks=0
    p.pitch_trim=args.pitch_trim_deg
    p.height_trim=args.height_trim_mm
    if not np.isfinite(p.height_trim) or abs(p.height_trim)>6.:
        raise ValueError("Height adjustment outside the simulated handoff window")
    p.pose_ticks=0
    if not np.isfinite(p.pitch_trim) or abs(p.pitch_trim)>1.5:
        raise ValueError("Pitch adjustment outside the simulated handoff window")
    if p.target_rpm is not None and not p.launcher["readyRpm"]<=p.target_rpm<=6000.:
        raise ValueError("Target speed outside the modeled launcher operating range")
    if args.pinch_duty is not None:
        if not 0.<args.pinch_duty<=1.:
            raise ValueError("Invalid intake feed duty")
        p.launcher["pinchDuty"]=args.pinch_duty
    rc = load(args.delivery/"result.json")
    if (rc["outcome"]!="AT_SHOOTING_POSE" or rc["nativeSha256"]!=pins["native"]
            or (rc.get("firstMark","inner"),rc.get("fieldYawDeg",90.))!=(args.first_mark,args.field_yaw_deg)):
        raise ValueError("Missing the retained NOTE at the shooting pose")
    row=tail(args.delivery,rc)
    restore(p,row)
    p.shoot_position=np.array(rc["shootPosition"])
    p.shoot_heading=rc["shootHeading"]
    p.start_height,p.start_pitch=row["q"][3:5]
    p.start_pitch+=np.deg2rad(p.pitch_trim)
    p.start_height+=.001*p.height_trim
    p.start_time=p.data.time
    p.station_pose=np.array(row["q"][3:5])
    p.trim_pose=np.array([p.height_trim*.001,np.deg2rad(p.pitch_trim)])
    p.base.initial={"note":{"p":[p.base.cad["stageCenterX"]/1000]},"chassis":{"x":0.}}
    period=round(.01/args.dt)
    first=last_touch=both=None
    outcome="HORIZON"
    minimum_bus=p.bus
    args.output.mkdir(exist_ok=False)
    trace=args.output/"trace.jsonl"
    with trace.open("x") as stream:
        duration = p.align_seconds + (.9 if p.target_rpm or p.pitch_trim or p.height_trim else .5)
        for i in range(round(duration/args.dt)):
            t=p.data.time
            try:
                belt,wheel = p.step(i%period==0,args.duty,True)
            except (ArithmeticError,RuntimeError,ValueError) as e:
                outcome=str(e);break
            if wheel>1e-6:
                if first is None:first=t
                last_touch=t
                if belt>1e-6 and both is None:both=t
            d=p.data
            minimum_bus=min(minimum_bus,p.bus)
            if (both is not None and last_touch is not None and d.time-last_touch>.03
                    and belt<1e-6 and d.qvel[p.nv+2]>0):
                outcome="UPWARD_FREE_RELEASE"
            sample=dict(time=d.time,q=d.qpos.tolist(),v=d.qvel.tolist(),externalRotors=p.rotors.tolist(),
                        shearM=p.shear.tolist(),beltNormalN=belt,wheelNormalN=wheel,
                        forceTime=t,busV=p.bus,sourceJ=p.source_work)
            stream.write(json.dumps(sample,allow_nan=False)+"\n")
            if outcome=="UPWARD_FREE_RELEASE":break
    if any(w.number for w in p.data.warning):outcome="MUJOCO_WARNING"
    result=dict(schema="wing-raised-feed/1",pieceId=rc.get("pieceId","blue-wing-middle"),sourceTraceSha256=rc["trace"],
                nativeSha256=pins["native"],initialQ=row["q"],initialV=row["v"],traceStride=1,
                start=row["t"],end=p.data.time,feedStart=p.feed_start,firstLauncherContact=first,
                lastLauncherContact=last_touch,firstSimultaneousContact=both,minimumBusV=minimum_bus,
                outcome=outcome,traceSha256=digest(trace),grid=args.grid,dt=args.dt,segments=args.segments,duty=args.duty,
                firstMark=args.first_mark,fieldYawDeg=args.field_yaw_deg,pinchDuty=p.launcher["pinchDuty"],
                targetRpm=p.target_rpm,pitchTrimDeg=p.pitch_trim,heightTrimMm=p.height_trim,
                alignmentDurationS=p.align_seconds, ringPhaseDeg=phase,
                runtime=runtime_versions())
    result["collectedMarks"]=rc.get("collectedMarks",[args.first_mark] if rc.get("fieldStation") else [args.first_mark,"middle"])
    if "attempt" in rc:
        result.update(attempt=rc["attempt"], rosterSha256=rc["rosterSha256"])
    (args.output/"result.json").write_text(json.dumps(result,indent=2,allow_nan=False)+"\n")
    print(outcome,p.data.time,flush=True)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ("pickup","archive","delivery","output"):
        parser.add_argument("--"+name,type=Path,required=True)
    parser.add_argument("--dt",type=float,choices=(.0001,.00005,.000025),default=.0001)
    parser.add_argument("--grid",type=int,choices=(256,512,1024,2048),default=512)
    parser.add_argument("--segments",type=int,choices=(4096,8192,16384),default=8192)
    parser.add_argument("--ring-phase-deg",type=float,default=0.,
                        help="Rotate the contact samples around the same physical NOTE for a numerical check")
    parser.add_argument("--duty",type=float,default=.72)
    parser.add_argument("--pinch-duty",type=float,help="Override the nominal powered intake feed duty")
    parser.add_argument("--target-rpm",type=float,help="Hold all four wheels within 2.5 percent for three control samples before feeding")
    parser.add_argument("--pitch-trim-deg",type=float,default=0.,help="Adjust the cradle with its loaded pitch motor before feed")
    parser.add_argument("--height-trim-mm",type=float,default=0.,help="Register lift height at the launcher using the loaded lift motor")
    parser.add_argument("--align-seconds",type=float,default=0.,help="Move through the trim with a smooth reference and the loaded travel pinch controller")
    parser.add_argument("--first-mark",choices=("inner","middle","outer"),default="inner")
    parser.add_argument("--field-yaw-deg",type=float,default=90.)
    run(parser.parse_args())
