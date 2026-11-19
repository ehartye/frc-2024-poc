"""Bound a released NOTE against the moving robot on their common mission clock."""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from run_axial_shear_lift import digest, load
from turning_sweep import NativeTurnPath
from wing_field_frame import WingFrame, SUPPLY_MARKS
from field_route_clearance import SURFACES, GAME_PIECES


def synchronized_path(field, robot, frame):
    flight=load(field/"result.json")
    returning=load(robot/"result.json")
    params=flight["parameters"]
    if (not params.get("trace_every_step") or returning["traceStride"]!=1
            or flight["sourceTrace"]!=returning["firstReleaseTrace"]
            or flight["nativeSha256"]!=returning["nativeSha256"]
            or (params["first_mark"],params["field_yaw_deg"])!=(frame.first_mark,frame.yaw_degrees)
            or abs(flight["releaseTimeS"]-returning["start"])>1e-8):
        raise ValueError("Expected synchronized field and robot trajectories from the same release")
    samples=[]
    for folder,receipt in ((field,flight),(robot,returning)):
        path=folder/"trace.jsonl"
        if digest(path)!=receipt["trace"]:
            raise ValueError("Trace differs from the recorded trajectory")
        with path.open("rb") as source:
            rows=[json.loads(line) for line in source]
        times=np.array([r["t"] for r in rows])-returning["start"]
        states=np.array([r["q"] for r in rows])
        if folder==robot:
            times=np.r_[0.,times]
            states=np.vstack((returning["initialRobotQ"][:5],states[:,:5]))
        dt=params["dt"] if folder==field else returning["dt"]
        if not np.allclose(times,np.arange(len(times))*dt,atol=1e-8,rtol=0):
            raise ValueError("Nonuniform or incomplete integration history")
        samples.append(states)
    note,body=samples
    df,dr=params["dt"],returning["dt"]
    if (len(body)-1)*dr+1e-8<(len(note)-1)*df:
        raise ValueError("Robot trajectory ends before the field trajectory")
    # These supported simulation periods are integer microseconds. Include every
    # integrator boundary in the combined interpolation.
    periods=np.rint(np.array([df,dr])*1e6).astype(int)
    if not np.allclose(periods/1e6,[df,dr],rtol=0,atol=1e-12) or min(periods)<1:
        raise ValueError("Unsupported integration clocks")
    step=math.gcd(*periods)/1e6
    t=np.arange(round((len(note)-1)*df/step)+1)*step
    chassis=np.column_stack([np.interp(t,np.arange(len(body))*dr,b) for b in body.T])
    world=np.column_stack([np.interp(t,np.arange(len(note))*df,b) for b in note[:,:3].T])
    xyz=(world-frame.origin_mark)@frame.rotation+[.658,0.,0.]
    i=np.minimum(np.floor(t/df).astype(int),len(note)-2)
    u=np.clip(t/df-i,0.,1.)
    a,b=note[i,3:],note[i+1,3:].copy()
    dot=(a*b).sum(axis=1)
    b[dot<0]*=-1.
    theta=np.arccos(np.clip(abs(dot),0.,1.))/np.pi
    # Interpolate the shortest rotation on each solver step.
    quat=(a*((1-u)*np.sinc((1-u)*theta))[:,None]+b*(u*np.sinc(u*theta))[:,None])/np.sinc(theta)[:,None]
    w,x,y,z=quat.T
    half=math.radians(frame.yaw_degrees)/2
    c,s=math.cos(half),math.sin(half)
    local_quat=np.column_stack((c*w+s*z,c*x+s*y,c*y-s*x,c*z-s*w))
    return NativeTurnPath.from_poses(np.column_stack((chassis,xyz,local_quat)),step)


def separation(path,inventory):
    boxes={g:np.array([p["boundsMm"] for p in inventory if p["motionGroup"]==g]).reshape(-1,2,3)
           for g in path.frames}
    pending=[(0,path.length)]
    minimum=math.inf
    for stride in (100,20,5,1):
        unresolved=[]
        for span in path.intervals(pending,stride):
            gap=math.inf
            for name,box in boxes.items():
                body=span["frames"][name]
                center=np.array(body["center"])
                normal=np.array(body["normal"])
                extent=152.4*np.sqrt(np.maximum(0.,1-normal**2))+25.4+body["padMm"]
                gap=min(gap,float(np.maximum(box[:,0]-center-extent,center-extent-box[:,1]).max(axis=1).min()))
            if gap<=0:
                unresolved.append((span["first"],span["last"]))
            else:
                minimum=min(minimum,gap)
        pending=unresolved
        if not pending:
            break
    return dict(intervals=path.length, unresolvedIntervals=len(pending),
                firstUnresolved=pending[0] if pending else None,
                minimumCertifiedGapMm=minimum if math.isfinite(minimum) else None)


def flight_environment(field,robot,frame,installed,roster=None):
    """Check the bodies outside the Speaker contact solver and the other NOTES."""
    from speaker_trial_inputs import placed_marks
    run=load(robot/"result.json")
    marks=placed_marks(frame, run, roster)
    with (field/"trace.jsonl").open("rb") as stream:
        flight=np.array([[r["t"],*r["q"][:3]] for r in map(json.loads,stream)])
    moving=None
    times=flight[:,0]
    if run.get("schema")!="post-release-station-hold/1":
        with (robot/"trace.jsonl").open("rb") as stream:
            moving=np.array([[r["t"],*frame.to_field(r["q"][-7:-4])] for r in map(json.loads,stream)])
        mark=marks[run["collectedMarks"][-1]]
        moving=np.vstack(([run["start"],*mark[:2],.0254-.235*9.80665/20000],moving))
        times=np.unique(np.r_[times,moving[moving[:,0]<=flight[-1,0],0]])
    centers=np.column_stack([np.interp(times,flight[:,0],v) for v in flight[:,1:].T])
    # Both centers interpolate linearly between solver boundaries. Their closest
    # approach on a common interval is the point on that relative-motion segment.
    def pair_gap(other):
        rel=centers-other
        delta=np.diff(rel,axis=0)
        alpha=np.clip(-np.sum(rel[:-1]*delta,axis=1)/np.maximum(1e-30,np.sum(delta**2,axis=1)),0.,1.)
        return float(np.min(np.linalg.norm(rel[:-1]+alpha[:,None]*delta,axis=1))-.3556)
    gaps=[]
    if moving is not None:
        next_note=np.column_stack([np.interp(times,moving[:,0],v) for v in moving[:,1:].T])
        gaps.append(pair_gap(next_note))
    for name in SUPPLY_MARKS.keys()-set(run["collectedMarks"]):
        gaps.append(pair_gap(marks[name]+[0,0,.0254]))
    exempt=SURFACES|GAME_PIECES|{"GE-24300_Briweld Field Blue"}
    boxes=np.array([part["bounds"] for part in installed["parts"] if part["path"][1] not in exempt])
    lower=centers.min(axis=0)-.177803
    upper=centers.max(axis=0)+.177803
    axis_gap=np.maximum(boxes[:,0]-upper,lower-boxes[:,1]).max(axis=1)
    result=dict(minimumOtherNoteGapM=min(gaps) if gaps else None,otherFieldEnvelopeOverlaps=int(np.sum(axis_gap<=0)))
    if moving is None:
        result["remainingFloorNotes"]=sorted(SUPPLY_MARKS.keys()-set(run["collectedMarks"]))
    return result


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ("field","robot","inventory","output"):
        parser.add_argument(name,type=Path)
    parser.add_argument("--placement-roster",type=Path)
    parser.add_argument("--installed-field",type=Path)
    args=parser.parse_args()
    robot=load(args.robot/"result.json")
    inventory=load(args.inventory)
    if robot["nativeSha256"]!=inventory["sourceSha256"]:
        raise ValueError("Native body inventory mismatch")
    frame=WingFrame(robot["firstMark"],robot["fieldYawDeg"])
    path=synchronized_path(args.field,args.robot,frame)
    result=separation(path,inventory["inventory"])
    result.update(robot=robot["trace"],field=digest(args.field/"trace.jsonl"),native=robot["nativeSha256"])
    if args.installed_field:
        field=load(args.installed_field)
        result["environment"]=flight_environment(args.field,args.robot,frame,field,args.placement_roster)
        result["fieldBounds"]=digest(args.installed_field)
        if "attempt" in robot:
            result.update(attempt=robot["attempt"], rosterSha256=robot["rosterSha256"])
    with args.output.open("x") as stream:
        json.dump(result,stream,indent=2,allow_nan=False)
    print(json.dumps(result))
