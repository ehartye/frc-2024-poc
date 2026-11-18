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
            values.append([last["t"],*last["q"][:3]])
    data=np.array(values)
    if (len(data)<2 or not np.isfinite(data).all() or
            not np.allclose(np.diff(data[:,0]),rc["parameters"]["dt"],atol=1e-8,rtol=0)
            or abs(data[0,0]-rc.get("segmentStartS",rc["releaseTimeS"]))>1e-8
            or abs(data[-1,0]-rc["endS"])>1e-8):
        raise ValueError("Incomplete field time history")
    return rc,data,first,last


def check(previous,segments,current,sensor):
    scored=load(previous)
    target=load(sensor)
    points=np.array([point for item in target["sensorCaseEnds"] for point in item["caseEndCornersM"]])
    low,high=points.min(axis=0),points.max(axis=0)
    joined=[]
    last_receipt=last_state=None
    for folder in segments:
        rc,positions,first,last=centers(folder)
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
    prior=np.vstack(joined)
    later,new,_,_=centers(current)
    if (later["sourceTrace"]==scored["shot"] or new[0,0]<=scored["scoreS"]
            or abs(new[0,0]-later["releaseTimeS"])>1e-8
            or new[0,0]<prior[0,0] or new[-1,0]>prior[-1,0]+1e-8):
        raise ValueError("Distinct concurrent flights with full previous NOTE coverage required")
    if (later["field"]!=last_receipt["field"] or later["nativeSha256"]!=scored["native"]
            or digest(sensor)!=scored["sensorObservation"]["sensorFile"]):
        raise ValueError("Field, counter or robot registration changed")
    # Contain all possible optical rays and the translating torus on each interval.
    a,b=prior[:-1,1:],prior[1:,1:]
    extent=MAJOR+TUBE+1e-7
    gaps=np.maximum(low-np.maximum(a,b)-extent,np.minimum(a,b)-extent-high).max(axis=1)
    overlap=np.flatnonzero((prior[1:,0]>=scored["scoreS"]) & (gaps<=0))
    if not len(overlap):raise ValueError("Earlier score never occupied the sensor region")
    clear_time=prior[overlap[-1]+1,0]
    if clear_time>=new[0,0]:raise ValueError("Earlier NOTE has not cleared the sensor before the next release")
    times=np.unique(np.r_[new[:,0],prior[(prior[:,0]>new[0,0])&(prior[:,0]<new[-1,0]),0]])
    def interp(rows):
        return np.column_stack([np.interp(times,rows[:,0],axis) for axis in rows[:,1:].T])
    rel=interp(prior)-interp(new)
    delta=np.diff(rel,axis=0)
    alpha=np.clip(-np.sum(rel[:-1]*delta,axis=1)/np.maximum(1e-30,np.sum(delta*delta,axis=1)),0.,1.)
    gap=float(np.linalg.norm(rel[:-1]+alpha[:,None]*delta,axis=1).min()-2*extent)
    if gap<=0.:raise ValueError("Released NOTES need a coupled contact solve")
    return dict(previousScore=digest(previous),olderShot=scored["shot"],laterShot=later["sourceTrace"],
                field=later["trace"],priorSegments=[digest(f/"trace.jsonl") for f in segments],
                sensorClearByS=float(clear_time),minimumPieceSeparationM=gap,verifiedThroughS=float(new[-1,0]))


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for name in ("previous","current","sensor","output"):
        p.add_argument("--"+name,type=Path,required=True)
    p.add_argument("--prior-segments",type=Path,nargs="+",required=True)
    args=p.parse_args()
    rc=check(args.previous,args.prior_segments,args.current,args.sensor)
    with args.output.open("x") as stream:json.dump(rc,stream,indent=2,allow_nan=False)
    print(rc["sensorClearByS"],rc["minimumPieceSeparationM"])
