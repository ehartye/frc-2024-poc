"""Check the previous NOTE leaving the sensor and its separation from a later shot."""
import argparse
import json
from pathlib import Path

import numpy as np

from run_axial_shear_lift import digest,load
from wing_speaker_passage import MAJOR,TUBE


def centers(folder):
    rc=load(folder/"result.json")
    trace=folder/"trace.jsonl"
    if not rc["parameters"]["trace_every_step"] or digest(trace)!=rc["trace"]:
        raise ValueError("Counter reset needs a dense field trajectory with matching digest")
    first=last=None
    values=[]
    with trace.open("rb") as stream:
        for line in stream:
            last=json.loads(line)
            if first is None:first=last
            values.append([last["t"],*last["q"]])
    data=np.array(values)
    expected=np.full(max(0,len(data)-1),float(rc["parameters"]["dt"]))
    if "finalStepS" in rc:
        final=rc["finalStepS"]
        if not len(expected) or not 0.<final<=expected[-1]+1e-8:
            raise ValueError("Invalid final integration interval")
        expected[-1]=final
    if (len(data)<2 or data.shape[1]!=8 or not np.isfinite(data).all()
            or not np.allclose(np.linalg.norm(data[:,4:],axis=1),1.,rtol=0.,atol=1e-8) or
            not np.allclose(np.diff(data[:,0]),expected,atol=1e-8,rtol=0)
            or abs(data[0,0]-rc.get("segmentStartS",rc["releaseTimeS"]))>1e-8
            or abs(data[-1,0]-rc["endS"])>1e-8):
        raise ValueError("Incomplete field time history")
    return rc,data,first,last


def collection_path(scored,segments):
    joined=[]
    spans=[]
    last_receipt=last_state=None
    for folder in segments:
        rc,positions,first,last=centers(folder)
        coupled=rc.get("coupledManifest")
        if coupled:
            manifest=folder.parent/"manifest.json"
            if digest(manifest)!=coupled:
                raise ValueError("Coupled collector manifest changed")
            group=load(manifest)
            index=group["shots"].index(rc["sourceTrace"])
            if (group["schema"]!="coupled-note-collection/1" or group["traces"][index]!=rc["trace"]
                    or group["inputs"][index]!=rc["previousFieldTrace"]
                    or abs(group["start"]-first["t"])>1e-8 or abs(group["end"]-last["t"])>1e-8):
                raise ValueError("Trajectory differs from its shared collection solve")
        spans.append((first["t"],last["t"],coupled))
        if rc["sourceTrace"]!=scored["shot"] or rc["nativeSha256"]!=scored["native"]:
            raise ValueError("Earlier NOTE identity changed")
        if last_receipt is None:
            if rc["trace"]!=scored["field"]:raise ValueError("Previous score uses a different field trace")
        else:
            if (rc.get("previousFieldTrace")!=last_receipt["trace"] or first["q"]!=last_state["q"]
                    or first["v"]!=last_state["v"] or first["t"]!=last_state["t"]
                    or rc["field"]!=last_receipt["field"]):
                raise ValueError("Collection trajectory is discontinuous")
            positions=positions[1:]
        joined.append(positions)
        last_receipt,last_state=rc,last
    return np.vstack(joined), last_receipt, spans


def orientations(rows,times):
    """Shortest-arc interpolation inside a recorded MuJoCo integration step."""
    index=np.clip(np.searchsorted(rows[:,0],times,side="right")-1,0,len(rows)-2)
    weight=np.clip((times-rows[index,0])/(rows[index+1,0]-rows[index,0]),0.,1.)
    qa,qb=rows[index,4:8],rows[index+1,4:8]
    dot=np.sum(qa*qb,axis=1)
    qb=qb*np.where(dot<0.,-1.,1.)[:,None]
    angle=np.arccos(np.clip(abs(dot),0.,1.))
    # sinc avoids division by zero when successive orientations coincide.
    left=(1.-weight)*np.sinc((1.-weight)*angle/np.pi)/np.sinc(angle/np.pi)
    right=weight*np.sinc(weight*angle/np.pi)/np.sinc(angle/np.pi)
    return left[:,None]*qa+right[:,None]*qb


def ring_gap(relative,qa,qb,nodes):
    """Sample circle A against the exact centerline of B, including angular sampling error."""
    w,x,y,z=qa.T
    a=np.column_stack((1-2*(y*y+z*z),2*(x*y+w*z),2*(x*z-w*y)))
    b=np.column_stack((2*(x*y-w*z),1-2*(x*x+z*z),2*(y*z+w*x)))
    w,x,y,z=qb.T
    axis=np.column_stack((2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y)))
    angle=np.arange(nodes)*2*np.pi/nodes
    points=relative[:,None,:]+MAJOR*(a[:,None,:]*np.cos(angle)[None,:,None]+b[:,None,:]*np.sin(angle)[None,:,None])
    height=np.sum(points*axis[:,None,:],axis=2)
    radius=np.sqrt(np.maximum(0.,np.sum(points*points,axis=2)-height*height))
    return np.hypot(radius-MAJOR,height).min(axis=1)-2*TUBE-MAJOR*np.pi/nodes


def separate_paths(prior,new,start,end):
    if start>=end or any(rows[0,0]>start or rows[-1,0]+1e-8<end for rows in (prior,new)):
        raise ValueError("Missing concurrent field occupancy")
    times=np.unique(np.concatenate(([start,end],*[rows[(rows[:,0]>start)&(rows[:,0]<end),0]
                                                  for rows in (prior,new)])))
    def interp(rows):
        return np.column_stack([np.interp(times,rows[:,0],axis) for axis in rows[:,1:4].T])
    rel=interp(prior)-interp(new)
    delta=np.diff(rel,axis=0)
    alpha=np.clip(-np.sum(rel[:-1]*delta,axis=1)/np.maximum(1e-30,np.sum(delta*delta,axis=1)),0.,1.)
    gaps=np.linalg.norm(rel[:-1]+alpha[:,None]*delta,axis=1)-2*(MAJOR+TUBE+1e-7)
    uncertain=np.flatnonzero(gaps<=0.)
    for batch in range(0,len(uncertain),128):
        indices=uncertain[batch:batch+128]
        begin,end=times[indices],times[indices+1]
        qa,qb=[orientations(rows,begin) for rows in (prior,new)]
        pad=np.linalg.norm(delta[indices],axis=1)+2e-7
        for rows,quats in ((prior,qa),(new,qb)):
            dot=np.sum(orientations(rows,end)*quats,axis=1)
            pad+=2*MAJOR*np.arccos(np.clip(abs(dot),0.,1.))
        values=ring_gap(rel[indices],qa,qb,256)-pad
        near=values<=0.
        if near.any():
            values[near]=ring_gap(rel[indices[near]],qa[near],qb[near],2048)-pad[near]
        if values.min()<=0.:
            t=begin[np.argmin(values)]
            raise ValueError(f"Released NOTES need a coupled contact solve at {t:.6f} s")
        gaps[indices]=values
    return float(gaps.min())


def collection_pair(prior,spans,other,other_spans,start,end):
    # Between emission and a joint solve, both one-body trajectories must be
    # independent. Within a joint solve contact is part of both integrated paths.
    if any(start<group[0][0]-1e-8 or end>group[-1][1]+1e-8 for group in (spans,other_spans)):
        raise ValueError("Missing earlier collection coverage")
    breaks=sorted({start,end,*(t for a,b,_ in [*spans,*other_spans] for t in (a,b) if start<t<end)})
    gaps=[]
    shared=[]
    for a,b in zip(breaks,breaks[1:]):
        if b-a<=1e-8:
            continue
        midpoint=(a+b)/2
        active=[next(tag for lo,hi,tag in group if lo<=midpoint<=hi) for group in (spans,other_spans)]
        if active[0] and active[0]==active[1]:
            shared.append(active[0])
        else:
            gaps.append(separate_paths(prior,other,a,b))
    return dict(gapM=min(gaps) if gaps else None,coupledManifests=shared,verifiedThroughS=end)


def check(previous,segments,current,sensor,earlier_collections=()):
    scored=load(previous)
    target=load(sensor)
    points=np.array([point for item in target["sensorCaseEnds"] for point in item["caseEndCornersM"]])
    low,high=points.min(axis=0),points.max(axis=0)
    prior,last_receipt,spans=collection_path(scored,segments)
    later,new,_,_=centers(current)
    if (later["sourceTrace"]==scored["shot"] or new[0,0]<=scored["scoreS"]
            or abs(new[0,0]-later["releaseTimeS"])>1e-8
            or new[0,0]<prior[0,0] or new[-1,0]>prior[-1,0]+1e-8):
        raise ValueError("Distinct concurrent flights with full previous NOTE coverage required")
    if (later["field"]!=last_receipt["field"] or later["nativeSha256"]!=scored["native"]
            or digest(sensor)!=scored["sensorObservation"]["sensorFile"]):
        raise ValueError("Field, counter or robot registration changed")
    # Contain all possible optical rays and the translating torus on each interval.
    a,b=prior[:-1,1:4],prior[1:,1:4]
    extent=MAJOR+TUBE+1e-7
    gaps=np.maximum(low-np.maximum(a,b)-extent,np.minimum(a,b)-extent-high).max(axis=1)
    overlap=np.flatnonzero((prior[1:,0]>=scored["scoreS"]) & (gaps<=0))
    if not len(overlap):raise ValueError("Earlier score never occupied the sensor region")
    clear_time=prior[overlap[-1]+1,0]
    if clear_time>=new[0,0]:raise ValueError("Earlier NOTE has not cleared the sensor before the next release")
    gap=separate_paths(prior,new,new[0,0],new[-1,0])
    history=scored.get("precedingScores",[])
    if len(earlier_collections)!=len(history):
        raise ValueError("Need every earlier collection trajectory through the new flight")
    pair_checks=[]
    for reference,(path,*folders) in zip(history,earlier_collections):
        if digest(path)!=reference["receiptSha256"] or not folders:
            raise ValueError("Earlier score or collection trajectory missing")
        other,receipt,other_spans=collection_path(load(path),folders)
        if receipt["field"]!=later["field"] or receipt["nativeSha256"]!=later["nativeSha256"]:
            raise ValueError("Earlier collection used different field or robot geometry")
        occupancy=collection_pair(prior,spans,other,other_spans,prior[0,0],new[-1,0])
        pair_checks.append(dict(score=digest(path),segments=[digest(f/"trace.jsonl") for f in folders],**occupancy))
    result=dict(previousScore=digest(previous),olderShot=scored["shot"],laterShot=later["sourceTrace"],
                field=later["trace"],priorSegments=[digest(f/"trace.jsonl") for f in segments],
                sensorClearByS=float(clear_time),minimumPieceSeparationM=gap,verifiedThroughS=float(new[-1,0]))
    if pair_checks:result["earlierCollectionSeparation"]=pair_checks
    return result


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for name in ("previous","current","sensor","output"):
        p.add_argument("--"+name,type=Path,required=True)
    p.add_argument("--prior-segments",type=Path,nargs="+",required=True)
    p.add_argument("--earlier-collection",type=Path,nargs="+",action="append",default=[],
                   help="A preceding score receipt followed by its contiguous collection segments, in score order")
    args=p.parse_args()
    rc=check(args.previous,args.prior_segments,args.current,args.sensor,args.earlier_collection)
    with args.output.open("x") as stream:json.dump(rc,stream,indent=2,allow_nan=False)
    print(rc["sensorClearByS"],rc["minimumPieceSeparationM"])
