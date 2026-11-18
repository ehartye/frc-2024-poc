"""Bound the registered robot and dormant NOTES against installed field structures."""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from run_axial_shear_lift import EVIDENCE, digest, load
from wing_field_frame import SUPPLY_MARKS, WingFrame


SURFACES = {"FE-2024-01","Cache Zone","Starting Line_Mid Line","Starting Line_Launch Line",
            "Retrieval Zone","Actual Starting Line","End of Cache Zone","RRZ end","Alliance Area",
            "Alliance Area 2","Endgame Tape","Boost Area End","Boost Area Side","Boost Area Back",
            "Note Staging Tape","Coaches Line","CenterSpike"}
GAME_PIECES = {"GE-24500","GE-24501"}
PIVOT = np.array([.178,0.,.0254])


def ground_prefix(pickup):
    ground=load(pickup/"ground/receipt.json")
    stop=load(pickup/"stop/receipt.json")
    if stop["sourceTrajectorySha256"]!=digest(pickup/"ground/receipt.json"):
        raise ValueError("Pickup and stop have different sources")
    start=stop["trace"][0]["timeS"]
    rows=[[0.,0.,0.,0.,0.,0.,.658,0.,.0254]]
    for r in (x for segment in (ground["trace"],stop["trace"]) for x in segment
              if segment is stop["trace"] or x["timeS"]<start-1e-9):
        c=r["chassis"]
        rows.append([r["timeS"],c["x"],c["y"],c["heading"],0.,0.,*r["note"]["p"],.0254])
    return np.asarray(rows)


def prefix(pickup, release):
    rows = ground_prefix(pickup).tolist()
    shot=load(release/"result.json")
    for path,key,expected in ((pickup/"lift/trace.jsonl","clock",shot["sourceTraceSha256"]),
                              (release/"trace.jsonl","time",shot["traceSha256"])):
        if digest(path)!=expected:
            raise ValueError("Native mission source mismatch")
        with path.open("rb") as stream:
            for line in stream:
                r=json.loads(line)
                rows.append([r[key],*r["q"][:5],*r["q"][-7:-4]])
    return np.asarray(rows)


def phase(folder):
    result=load(folder/"result.json")
    path=folder/"trace.jsonl"
    if digest(path)!=result.get("trace",result.get("traceSha256")) or result.get("traceStride")!=1:
        raise ValueError("Field audit needs every integration pose of the phase")
    with path.open("rb") as stream:
        data=[json.loads(line) for line in stream]
    return np.array([[r.get("time",r.get("t")),*r["q"][:5],*r["q"][-7:-4]] for r in data])


def audit(rows,frame,inventory,field,waiting):
    boxes={}
    for group in ("fixed","carriage","cradle"):
        items=[x for x in inventory if x["motionGroup"]==group]
        bounds=np.array([x["boundsMm"] for x in items]).reshape(-1,2,3)/1000
        boxes[group]=(bounds.mean(axis=1),(bounds[:,1]-bounds[:,0])/2,[x["key"] for x in items])
    # Floor coverings form the flat driving surface in this model. Physical NOTES
    # in this mission are represented by their marked supply, not duplicated CAD instances.
    obstacles=[dict(path=x["path"], bounds=box) for x in field if x["path"][1] not in SURFACES|GAME_PIECES
               for box in x.get("tightBounds",[x["bounds"]])]
    obs_names=["/".join(x["path"]) for x in obstacles]
    ob=np.array([x["bounds"] for x in obstacles])
    idle=np.array([[*xy,.0254] for xy in waiting],dtype=float).reshape(-1,3)
    idle_size=np.array([.1778,.1778,.0254])
    if len(idle):
        ob=np.concatenate([ob,np.stack((idle-idle_size,idle+idle_size),axis=1)])
        obs_names += [f"idle-{i}" for i in range(len(idle))]
    xyz=np.column_stack((rows[:,1:3],np.zeros(len(rows))))
    fieldxy=(xyz-[.658,0.,0.])@frame.rotation.T+frame.origin_mark
    active=(rows[:,6:9]-[.658,0.,0.])@frame.rotation.T+frame.origin_mark
    if np.any(np.diff(rows[:,0])<=0) or np.max(np.diff(rows[:,0]))>.0010001:
        raise ValueError("Robot trajectory is discontinuous or too sparse")
    # Prefix sums bound all intermediate integration steps in a block, including changes of direction.
    increments=np.column_stack([np.linalg.norm(np.diff(fieldxy,axis=0),axis=1),
                                np.abs(np.diff(rows[:,3:6],axis=0)),
                                np.linalg.norm(np.diff(active,axis=0),axis=1)])
    travel=np.vstack([np.zeros(5),np.cumsum(increments,axis=0)])
    unresolved=[]
    minimum=math.inf
    remaining=[(i,min(i+100,len(rows)-1)) for i in range(0,len(rows)-1,100)]
    while remaining:
        lo,hi=remaining.pop()
        move,yaw_arc,lift_move,pitch_arc,piece_move=travel[hi]-travel[lo]
        c,s=np.cos(rows[lo,3]),np.sin(rows[lo,3])
        yaw=frame.rotation@np.array([[c,-s,0.],[s,c,0.],[0.,0.,1.]])
        c,s=np.cos(rows[lo,5]),np.sin(rows[lo,5])
        pitch=np.array([[c,0.,-s],[0.,1.,0.],[s,0.,c]])
        center=fieldxy[lo]
        # Independently bounded robot reach from the outermost corners of every
        # registered body at any cradle pitch. Lift is purely vertical.
        radius=2.0
        nearby=np.flatnonzero(((center+radius+move>=ob[:,0]) & (center-radius-move<=ob[:,1])).all(axis=1))
        obstacles_here=ob[nearby]
        pairs=[]
        gap=math.inf
        for group,(local,half,names) in boxes.items():
            rot=yaw
            offset=center
            pad=move+.000003
            if group!="fixed":
                offset=center+np.array([0.,0.,rows[lo,4]])
                pad+=lift_move
            if group=="cradle":
                rot=yaw@pitch
                offset=offset+yaw@(PIVOT-pitch@PIVOT)
                pad+=pitch_arc*(np.linalg.norm(np.abs(local-PIVOT)+half,axis=1).max())
            rotation_reach=np.linalg.norm(np.abs(local)+half,axis=1).max()
            if group=="cradle":rotation_reach+=2*np.linalg.norm(PIVOT)
            if rotation_reach+abs(rows[lo,4])+lift_move>radius:
                raise ValueError("Field broad phase does not enclose the registered robot")
            pad+=yaw_arc*rotation_reach
            placed=offset+local@rot.T
            extent=half@np.abs(rot.T)+pad
            lower=placed-extent;upper=placed+extent
            if not len(nearby):continue
            # Compare a group first; almost all field parts are metres away.
            broad=np.maximum(obstacles_here[:,0]-upper.max(axis=0),lower.min(axis=0)-obstacles_here[:,1]).max(axis=1)
            outside=broad>0
            if np.any(outside):gap=min(gap,float(broad[outside].min()))
            for index in nearby[~outside]:
                clear=np.maximum(ob[index,0]-upper,lower-ob[index,1]).max(axis=1)
                gap=min(gap,float(clear.min()))
                pairs.extend((names[i],obs_names[index]) for i in np.flatnonzero(clear<=0))
        active_extent=.1778+piece_move+.000003
        static=ob[:len(obstacles)]
        near_piece=np.flatnonzero(((active[lo]+active_extent>=static[:,0])
                                    & (active[lo]-active_extent<=static[:,1])).all(axis=1))
        pairs.extend(("moving NOTE",obs_names[i]) for i in near_piece)
        if len(idle):
            gap=min(gap,float(np.linalg.norm(idle-active[lo],axis=1).min()-.3556-piece_move))
            if np.any(np.linalg.norm(idle-active[lo],axis=1)<=.3556+piece_move):
                pairs.append(("moving NOTE","idle NOTE envelopes"))
        if pairs and hi-lo>1:
            middle=(lo+hi)//2
            remaining.extend([(lo,middle),(middle,hi)])
        elif pairs:
            unresolved.append(dict(t=rows[lo,0],pairs=pairs))
        else:
            minimum=min(minimum,gap)
    return dict(begin=rows[0,0],end=rows[-1,0],intervals=len(rows)-1,unresolvedIntervals=len(unresolved),
                firstUnresolved=min(unresolved,key=lambda r:r["t"]) if unresolved else None,
                minimumReportedAxisGapM=minimum if math.isfinite(minimum) else None)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ("pickup","inventory","field","output"):
        parser.add_argument("--"+name,type=Path,required=True)
    parser.add_argument("--release", type=Path)
    parser.add_argument("--first-delivery", type=Path, help="Continue the first floor pickup from its braked state")
    parser.add_argument("--first-mark",choices=("outer","middle","inner"),required=True)
    parser.add_argument("--field-yaw-deg",type=float,required=True)
    parser.add_argument("--next-phase",type=Path,nargs="*")
    args=parser.parse_args()
    frame=WingFrame(args.first_mark,args.field_yaw_deg)
    robot=load(args.inventory)
    field=load(args.field)
    pins=load(EVIDENCE/"turning-lift-20261118/refined-end-to-end.json")["freshInputDigests"]
    if (robot["sourceSha256"]!=pins["native"] or field["leafCount"]!=len(field["parts"])
            or field["source"]!="42c91b3efb9521d059c13cf25e2d7dda6315bdd6b7eb1202847bfe84ffb43c1a"
            or digest(args.pickup/"stop/receipt.json")!=pins["stop"]):
        raise ValueError("Wrong registered field or robot")
    if args.first_delivery:
        carry=load(args.first_delivery/"result.json")
        if (carry["acquisitionTrace"]!=pins["stop"] or carry["nativeSha256"]!=pins["native"]
                or (carry["firstMark"],carry["fieldYawDeg"])!=(args.first_mark,args.field_yaw_deg)):
            raise ValueError("Delivery differs from the acquired state or field registration")
        rows=np.vstack((ground_prefix(args.pickup), phase(args.first_delivery)))
        if args.release:
            shot=load(args.release/"result.json")
            if shot["sourceTraceSha256"]!=carry["trace"] or shot["nativeSha256"]!=pins["native"]:
                raise ValueError("Release does not continue this delivery")
            rows=np.vstack((rows, phase(args.release)))
    else:
        if not args.release:
            parser.error("Supply --release or --first-delivery")
        rows=prefix(args.pickup,args.release)
    waiting=[frame.mark(n)[:2] for n in SUPPLY_MARKS if n!=args.first_mark]
    result=dict(firstMark=args.first_mark,yaw=args.field_yaw_deg,native=pins["native"],
                field=field["source"],fieldBounds=digest(args.field),stop=pins["stop"],results=[])
    if args.release:
        result["release"]=digest(args.release/"trace.jsonl")
    if args.first_delivery:
        result["delivery"]=carry["trace"]
    result["results"].append(audit(rows,frame,robot["inventory"],field["parts"],waiting))
    if args.next_phase:
        acquisition = load(args.next_phase[0]/"result.json")
        if not args.release or acquisition["firstReleaseTrace"]!=result["release"]:
            raise ValueError("Return does not continue the audited release")
        collected=acquisition.get("collectedMarks",[args.first_mark,"middle"])
        if collected[:-1]!=[args.first_mark] or len(set(collected))!=2:
            raise ValueError("Expected a distinct second floor NOTE")
        field_mark=frame.mark(collected[-1])
        mark=frame.to_local(field_mark)
        if not np.allclose(acquisition["waitingNoteLocalXY"],mark[:2],rtol=0.,atol=1e-10):
            raise ValueError("Waiting piece was placed at a different mark")
        start=np.r_[rows[-1,:6],mark[:2],.0254-.235*9.80665/20000]
        pieces=[xy for xy in waiting if not np.array_equal(xy,field_mark[:2])]
        following=np.vstack([start,*[phase(f) for f in args.next_phase]])
        result["followingTraces"]=[digest(f/"trace.jsonl") for f in args.next_phase]
        result["results"].append(audit(following,frame,robot["inventory"],field["parts"],pieces))
    with args.output.open("x") as stream:json.dump(result,stream,indent=2,allow_nan=False)
    print(json.dumps(result["results"]))
