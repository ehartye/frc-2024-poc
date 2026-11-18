"""Join the counter event to the native handoff and whole-field route that produced it."""
import argparse
import json
from pathlib import Path

from run_axial_shear_lift import EVIDENCE, digest, load
from speaker_score_observer import observe
from wing_field_frame import WingFrame, piece_id


def verify(a):
    carry,shot,field,ret=[load(folder/"result.json") for folder in (a.delivery,a.shot,a.flight,a.acquisition)]
    route,flight_clear,carry_clear,native,refined=map(load,(
        a.route_clearance,a.flight_clearance,a.delivery_clearance,a.shot_clearance,a.shell_refinement))
    pins=load(EVIDENCE/"turning-lift-20261118/refined-end-to-end.json")["freshInputDigests"]
    if any(r["nativeSha256"]!=pins["native"] for r in (carry,shot,field,ret)):
        raise ValueError("Native model mismatch")
    frame=WingFrame(carry["firstMark"],carry["fieldYawDeg"])
    previous_path=getattr(a,"previous",None)
    history=[]
    prior_start=0.
    pickup_trace=pins["stop"]
    target_piece=piece_id(frame.first_mark)
    if previous_path:
        prev=load(previous_path)
        entry=load(a.entry_acquisition/"result.json")
        marks=entry["collectedMarks"]
        history=[*prev.get("precedingScores",[]),dict(piece=prev["piece"],shot=prev["shot"],scoreS=prev["scoreS"],
                                                    receiptSha256=digest(previous_path))]
        if ([piece_id(mark) for mark in marks[:-1]]!=[r["piece"] for r in history]
                or len(set(marks))!=len(marks) or entry["pieceId"]!=piece_id(marks[-1])
                or entry["trace"]!=prev["nextPickup"] or entry["firstReleaseTrace"]!=prev["shot"]
                or route["entryAcquisition"]!=entry["trace"] or route["entryRelease"]!=prev["shot"]
                or entry["outcome"]!="SEATED" or entry["nativeSha256"]!=prev["native"]
                or abs(entry["start"]-prev["releaseS"])>1e-8 or abs(entry["end"]-carry["start"])>1e-8
                or digest(a.entry_acquisition/"trace.jsonl")!=entry["trace"]):
            raise ValueError("Pickup does not continue the verified mission")
        with (a.entry_acquisition/"trace.jsonl").open("rb") as stream:
            for line in stream:last=line
        held=json.loads(last)
        pickup_clear=load(a.pickup_clearance)
        if (carry["initialQ"]!=held["q"] or carry["initialV"]!=held["v"]
                or pickup_clear["source"]!=entry["trace"] or pickup_clear["unresolved"]):
            raise ValueError("Unverified retained pickup state or native geometry")
        resets=[load(path) for path in a.counter_reset]
        if len(resets)!=len(history):
            raise ValueError("Every preceding NOTE needs a verified clear counter and separate flight")
        for index,(earlier,reset) in enumerate(zip(history,resets)):
            if (reset["previousScore"]!=earlier["receiptSha256"] or reset["olderShot"]!=earlier["shot"]
                    or reset["laterShot"]!=shot["traceSha256"] or reset["field"]!=field["trace"]
                    or reset["minimumPieceSeparationM"]<=0. or reset["sensorClearByS"]>=shot["end"]
                    or reset["verifiedThroughS"]+1e-8<field["endS"]):
                raise ValueError("Wrong or incomplete counter reset")
            links=reset.get("earlierCollectionSeparation",[])
            if (len(links)!=index or any(link["score"]!=other["previousScore"]
                    or link["verifiedThroughS"]+1e-8<field["endS"]
                    or (link["gapM"] is None and not link["coupledManifests"])
                    or (link["gapM"] is not None and link["gapM"]<=0.)
                    or link["segments"]!=other["priorSegments"] for link,other in zip(links,resets[:index]))):
                raise ValueError("Prior NOTES lack mutually consistent collection trajectories")
        target_piece=entry["pieceId"]
        pickup_trace=entry["trace"]
        prior_start=entry["start"]
    for mark,heading in [(shot["firstMark"],shot["fieldYawDeg"]),(ret["firstMark"],ret["fieldYawDeg"]),
                         (route["firstMark"],route["yaw"]),(field["parameters"]["first_mark"],field["parameters"]["field_yaw_deg"])]:
        if (mark,heading)!=(frame.first_mark,frame.yaw_degrees):
            raise ValueError("Field frames differ across the mission")
    for folder,expected in ((a.delivery,carry["trace"]),(a.shot,shot["traceSha256"]),
                            (a.acquisition,ret["trace"]),(a.flight,field["trace"])):
        if digest(folder/"trace.jsonl")!=expected:
            raise ValueError("Trace hash mismatch")
    with (a.delivery/"trace.jsonl").open("rb") as stream:
        for line in stream:
            last_carry=line
    seated=json.loads(last_carry)
    with (a.shot/"trace.jsonl").open("rb") as stream:
        for line in stream:
            last_shot=line
    terminal=json.loads(last_shot)
    x,y,_=frame.to_field([*terminal["q"][:2],0.])
    if (seated["q"]!=shot["initialQ"] or seated["v"]!=shot["initialV"]
            or terminal["q"][:-7]!=ret["initialRobotQ"] or terminal["v"][:-6]!=ret["initialRobotV"]
            or ret["initialBusV"]!=terminal["busV"] or ret["initialRotors"]!=terminal["externalRotors"]
            or carry["pieceId"]!=target_piece or not 0.<x<=1.50
            or carry["minimumSideLoadN"]<=.02):
        raise ValueError("Close-range station or retained physical state mismatch")
    if (carry["acquisitionTrace"]!=pickup_trace or carry["outcome"]!="AT_SHOOTING_POSE"
            or shot["outcome"]!="UPWARD_FREE_RELEASE" or shot["pieceId"]!=carry["pieceId"]
            or shot["sourceTraceSha256"]!=carry["trace"] or ret["firstReleaseTrace"]!=shot["traceSha256"]
            or field["sourceTrace"]!=shot["traceSha256"] or ret["pieceId"]==shot["pieceId"]
            or abs(carry["end"]-shot["start"])>1e-8 or abs(shot["end"]-ret["start"])>1e-8):
        raise ValueError("Discontinuous or incomplete attempt")
    if (carry_clear["source"]!=carry["trace"] or carry_clear["unresolved"]
            or native["source"]!=shot["traceSha256"] or refined["nativeSweep"]!=digest(a.shot_clearance)
            or refined["trace"]!=shot["traceSha256"] or refined["native"]!=pins["native"]
            or refined["initialUnresolved"]!=len(native["unresolved"]) or refined["unresolved"]):
        raise ValueError("Unresolved robot hard-part clearance")
    if (route["native"]!=pins["native"] or route["stop"]!=pins["stop"] or route["delivery"]!=carry["trace"]
            or route["release"]!=shot["traceSha256"] or route["followingTraces"][0]!=ret["trace"]
            or len(route["results"])<2 or any(r["unresolvedIntervals"] for r in route["results"])
            or abs(route["results"][0]["begin"]-prior_start)>1e-8 or abs(route["results"][0]["end"]-shot["end"])>1e-8
            or abs(route["results"][1]["begin"]-ret["start"])>1e-8):
        raise ValueError("Whole-field route has unverified intervals")
    if (flight_clear["robot"]!=ret["trace"] or flight_clear["field"]!=field["trace"]
            or flight_clear["fieldBounds"]!=route["fieldBounds"] or flight_clear["unresolvedIntervals"]
            or flight_clear["environment"]["otherFieldEnvelopeOverlaps"]
            or flight_clear["environment"]["minimumOtherNoteGapM"]<=0.):
        raise ValueError("Released NOTE conflicts with other field participants")
    observation=observe(a.flight,a.sensor)
    time=observation["firstSimultaneousFullShadowS"]
    if (time is None or not observation["opening"] or observation["opening"]["fullyThroughByS"]>=time
            or time>min(120.,route["results"][1]["end"],field["endS"])):
        raise ValueError("Missing an in-order sensor event inside the verified route and time window")
    if history and time<=history[-1]["scoreS"]:
        raise ValueError("Counter events are out of order")
    result = dict(schema="continued-full-field-score/1" if history else "first-wing-full-field-score/1",
                precedingScores=history,scoreNumber=len(history)+1,piece=shot["pieceId"],scoreS=time,
                openedByS=observation["opening"]["fullyThroughByS"],releaseS=shot["end"],native=pins["native"],
                firstMark=carry["firstMark"],fieldYawDeg=carry["fieldYawDeg"],chassisFieldXY=[x,y],
                firstPickupStop=pins["stop"],carry=carry["trace"],shot=shot["traceSha256"],field=field["trace"],
                nextPickup=ret["trace"],fieldRoute=digest(a.route_clearance),flightClearance=digest(a.flight_clearance),
                shellRefinement=digest(a.shell_refinement),sensorObservation=observation)
    if history:
        result.update(pickupClearance=digest(a.pickup_clearance), deliveryClearance=digest(a.delivery_clearance),
                      shotClearance=digest(a.shot_clearance), counterResets=[digest(path) for path in a.counter_reset])
    return result


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for key in ("delivery","shot","flight","acquisition","sensor","route-clearance","flight-clearance",
                "delivery-clearance","shot-clearance","shell-refinement","output"):
        p.add_argument("--"+key,type=Path,required=True)
    for key in ("previous","entry-acquisition","pickup-clearance"):
        p.add_argument("--"+key,type=Path)
    p.add_argument("--counter-reset",type=Path,action="append",default=[])
    args=p.parse_args()
    chain=[bool(v) for v in (args.previous,args.entry_acquisition,args.pickup_clearance,args.counter_reset)]
    if any(chain) and not all(chain):p.error("A continued score needs its preceding score, pickup, native sweep and counter reset")
    result=verify(args)
    with args.output.open("x") as stream:
        json.dump(result,stream,indent=2,allow_nan=False)
    print(result["piece"],result["scoreS"])
