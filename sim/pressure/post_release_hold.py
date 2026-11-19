"""Continue the unloaded robot at the final scoring station on the release clock."""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from second_wing_pickup import ReturnPlant, angle_error, build
from run_axial_shear_lift import digest, load
from wing_field_frame import WING_MARKS_INCHES
from mission_runtime import runtime_versions


class StationHold(ReturnPlant):
    carries_note=False

    def contact(self):
        zero=np.zeros(3)
        empty=dict(force=zero,cradleMoment=zero,rotor=np.zeros(2),
                   normal=np.zeros(2),shearRate=np.zeros_like(self.shear))
        return empty,zero,zero

    def controls(self,belt,launcher_duty,feed_enabled):
        d=self.data
        axis=np.array([math.cos(d.qpos[2]),math.sin(d.qpos[2])])
        speed=(self.departure_xy-d.qpos[:2])@axis
        common=.8*(speed-d.qvel[:2]@axis)
        turn=3*angle_error(self.departure_heading,d.qpos[2])
        differential=.65*(turn-d.qvel[2])+.21*turn
        drive=np.clip([common-differential,common+differential],-.3,.3).tolist()
        actuator=self.actuator_command(belt,self.start_height,self.start_pitch)
        return drive,actuator,[0.,0.],0.


def build_swerve(args, shot):
    from maxswerve import SwervePickup
    from r7_dynamics import verify_binding
    from mission_runtime import verify_runtime_chain

    class ParkedSwerve(SwervePickup):
        carries_note = False
        contact = StationHold.contact

        def controls(self, belt, launcher_duty, feed_enabled):
            d = self.data
            forward, side = d.xmat[self.chassis].reshape(3,3)[:2,:2].T @ (self.departure_xy-d.qpos[:2])
            turn = angle_error(self.departure_heading, d.qpos[2])
            return (self.module_duties(2*forward, 2*side, 2*turn),
                    self.actuator_command(belt, self.start_height, self.start_pitch), [0., 0.], 0.)

    floor = load(args.pickup/"result.json")
    verify_runtime_chain([floor, shot])
    if (verify_binding(shot) != verify_binding(floor) or shot["outcome"] != "UPWARD_FREE_RELEASE" or
        (shot["firstMark"], shot["fieldYawDeg"]) != (args.first_mark, args.field_yaw_deg) or
        shot["nativeSha256"] != digest(args.archive) or shot["traceSha256"] != digest(args.release/"trace.jsonl") or
        floor["trace"] != digest(args.pickup/"trace.jsonl") or
        any(shot[k] != floor[k] for k in ("attempt","rosterSha256","pieceId","collectedMarks","nativeSha256"))):
        raise ValueError("Final hold requires the released swerve state and its pickup")
    with (args.release/"trace.jsonl").open() as stream:
        for line in stream:
            last = line
    last = json.loads(last)
    if last["time"] != shot["end"]:
        raise ValueError("Incomplete final release")
    p = ParkedSwerve.from_floor_receipt(floor, args.dt, args.grid, 32)
    p.rotors = np.array(last["externalRotors"])
    p.shear = np.array(last["shearM"])
    p.bus, p.data.time = last["busV"], last["time"]
    p.resume(last)
    p.start_height, p.start_pitch = last["q"][3:5]
    p.angle_target = p.mount + p.data.qpos[p.az_pos]
    return p, last, shot


def verify_hold(folder,receipt):
    """Independently measure the station error from every unloaded integration state."""
    initial=np.asarray(receipt["initialRobotQ"])
    size = 17 if receipt.get("drivetrain") == "maxswerve" else 9
    if len(initial)!=size or len(receipt["initialRobotV"])!=size or receipt["traceStride"]!=1:
        raise ValueError("Expected the five chassis/mechanism and four wheel coordinates")
    trace=folder/"trace.jsonl"
    if digest(trace)!=receipt["trace"]:
        raise ValueError("Parked robot trace changed")
    drift=heading=0.
    time=receipt["start"]
    with trace.open("rb") as stream:
        for tick,line in enumerate(stream,1):
            row=json.loads(line)
            q=np.asarray(row["q"])
            time=row["t"]
            if (q.shape!=(size,) or len(row["v"])!=size or not np.isfinite([*q,*row["v"],time]).all()
                    or abs(time-receipt["start"]-tick*receipt["dt"])>1e-8):
                raise ValueError("Discontinuous or loaded final chassis state")
            drift=max(drift,float(np.linalg.norm(q[:2]-initial[:2])))
            heading=max(heading,abs(angle_error(initial[2],q[2])))
    if (time<=receipt["start"] or abs(time-receipt["end"])>1e-8 or drift>.035 or heading>.012
            or abs(drift-receipt["maximumDriftM"])>1e-10
            or abs(heading-receipt["maximumHeadingErrorRad"])>1e-10):
        raise ValueError("Unverified final chassis position or heading")


def run(args):
    if not math.isfinite(args.horizon) or args.horizon<=0.:
        raise ValueError("The parked observation needs a positive horizon")
    source = load(args.release/"result.json")
    p,last,shot = build_swerve(args, source) if source.get("drivetrain") == "maxswerve" else build(args,StationHold)
    d=p.data
    if p.model.nq!=len(last["q"])-7 or p.model.nv!=len(last["v"])-6:
        raise ValueError("Expected exactly the unloaded robot coordinates")
    args.output.mkdir()
    period=round(.01/args.dt)
    trace=args.output/"trace.jsonl"
    max_drift=max_heading=0.
    with trace.open("x") as stream:
        for tick in range(math.ceil(args.horizon/args.dt)):
            belt,wheel=p.step(tick%period==0,0.,False)
            if belt or wheel or any(w.number for w in d.warning):
                raise ValueError("Unexpected contact or dynamics warning in the unloaded robot")
            max_drift=max(max_drift,float(np.linalg.norm(d.qpos[:2]-p.departure_xy)))
            max_heading=max(max_heading,abs(angle_error(p.departure_heading,d.qpos[2])))
            row=dict(t=d.time,q=d.qpos.tolist(),v=d.qvel.tolist(),rotor=p.rotors.tolist(),
                     shear=p.shear.tolist(),busV=p.bus,sourceJ=p.source_work)
            stream.write(json.dumps(row,allow_nan=False)+"\n")
    if max_drift>.035 or max_heading>.012:
        raise ValueError("Unloaded robot left the shooting pose")
    result=dict(schema="post-release-station-hold/1",firstReleaseTrace=shot["traceSha256"],runtime=runtime_versions(),
                nativeSha256=shot["nativeSha256"],trace=digest(trace),traceStride=1,
                start=last["time"],end=d.time,dt=args.dt,grid=args.grid,
                initialRobotQ=last["q"][:-7],initialRobotV=last["v"][:-6],
                initialRotors=last["externalRotors"],initialBusV=last["busV"],
                collectedMarks=shot["collectedMarks"],firstMark=args.first_mark,fieldYawDeg=args.field_yaw_deg,
                maximumDriftM=max_drift,maximumHeadingErrorRad=max_heading,outcome="HELD")
    if shot.get("drivetrain") == "maxswerve":
        result.update({k:shot[k] for k in ("drivetrain","r7MassCase","massBudgetSha256","routePolicy") if k in shot})
        result.update(initialShearM=last["shearM"], initialSourceJ=last["sourceJ"])
    if "attempt" in shot:
        result.update(attempt=shot["attempt"], rosterSha256=shot["rosterSha256"])
    (args.output/"result.json").write_text(json.dumps(result,indent=2,allow_nan=False)+"\n")
    return result


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ("release","pickup","archive","output"):
        parser.add_argument("--"+key,type=Path,required=True)
    parser.add_argument("--dt",type=float,choices=(.001,.0005,.00025),default=.0005)
    parser.add_argument("--grid",type=int,choices=(128,256,512,1024),default=512)
    parser.add_argument("--first-mark",choices=WING_MARKS_INCHES,required=True)
    parser.add_argument("--field-yaw-deg",type=float,required=True)
    parser.add_argument("--horizon",type=float,default=5.)
    result=run(parser.parse_args())
    print(result["outcome"],result["end"],result["maximumDriftM"])
