"""Feed the same retained wing NOTE from the attained second shooting pose."""
import argparse
import json
from pathlib import Path

import numpy as np

from launcher_handoff import RaisedFeed
from run_axial_shear_lift import digest, load
from second_wing_pickup import ReturnPlant, angle_error, build_seated
from wing_note_delivery import restore, tail


class ShotPlant(ReturnPlant):
    def controls(self, belt, launcher_duty, feed_enabled):
        _, actuator, pinch, duty = RaisedFeed.controls(self,belt,launcher_duty,feed_enabled)
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
    p, pins = build_seated(args, ShotPlant, args.segments)
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
    p.start_time=p.data.time
    period=round(.01/args.dt)
    first=last_touch=both=None
    outcome="HORIZON"
    minimum_bus=p.bus
    args.output.mkdir(exist_ok=False)
    trace=args.output/"trace.jsonl"
    with trace.open("x") as stream:
        for i in range(round(.5/args.dt)):
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
                firstMark=args.first_mark,fieldYawDeg=args.field_yaw_deg,pinchDuty=p.launcher["pinchDuty"])
    result["collectedMarks"]=rc.get("collectedMarks",[args.first_mark] if rc.get("fieldStation") else [args.first_mark,"middle"])
    (args.output/"result.json").write_text(json.dumps(result,indent=2,allow_nan=False)+"\n")
    print(outcome,p.data.time,flush=True)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ("pickup","archive","delivery","output"):
        parser.add_argument("--"+name,type=Path,required=True)
    parser.add_argument("--dt",type=float,choices=(.0001,.00005),default=.0001)
    parser.add_argument("--grid",type=int,choices=(256,512,1024,2048),default=512)
    parser.add_argument("--segments",type=int,choices=(4096,8192,16384),default=8192)
    parser.add_argument("--duty",type=float,default=.72)
    parser.add_argument("--pinch-duty",type=float,help="Override the nominal powered intake feed duty")
    parser.add_argument("--first-mark",choices=("inner","middle","outer"),default="inner")
    parser.add_argument("--field-yaw-deg",type=float,default=90.)
    run(parser.parse_args())
