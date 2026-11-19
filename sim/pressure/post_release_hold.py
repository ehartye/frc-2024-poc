"""Continue the unloaded robot at the final scoring station on the release clock."""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from second_wing_pickup import ReturnPlant, angle_error, build
from run_axial_shear_lift import digest
from wing_field_frame import WING_MARKS_INCHES


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


def verify_hold(folder,receipt):
    """Independently measure the station error from every unloaded integration state."""
    initial=np.asarray(receipt["initialRobotQ"])
    if len(initial)!=9 or len(receipt["initialRobotV"])!=9 or receipt["traceStride"]!=1:
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
            if (q.shape!=(9,) or len(row["v"])!=9 or not np.isfinite([*q,*row["v"],time]).all()
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
    p,last,shot=build(args,StationHold)
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
    result=dict(schema="post-release-station-hold/1",firstReleaseTrace=shot["traceSha256"],
                nativeSha256=shot["nativeSha256"],trace=digest(trace),traceStride=1,
                start=last["time"],end=d.time,dt=args.dt,grid=args.grid,
                initialRobotQ=last["q"][:-7],initialRobotV=last["v"][:-6],
                initialRotors=last["externalRotors"],initialBusV=last["busV"],
                collectedMarks=shot["collectedMarks"],firstMark=args.first_mark,fieldYawDeg=args.field_yaw_deg,
                maximumDriftM=max_drift,maximumHeadingErrorRad=max_heading,outcome="HELD")
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
