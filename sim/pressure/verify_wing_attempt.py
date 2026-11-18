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
            or carry["pieceId"]!=piece_id(frame.first_mark) or not 0.<x<=1.50
            or carry["minimumSideLoadN"]<=.02):
        raise ValueError("Close-range station or retained physical state mismatch")
    if (carry["acquisitionTrace"]!=pins["stop"] or carry["outcome"]!="AT_SHOOTING_POSE"
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
            or route["results"][0]["begin"]!=0. or abs(route["results"][0]["end"]-shot["end"])>1e-8
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
    return dict(schema="first-wing-full-field-score/1",piece=shot["pieceId"],scoreS=time,
                openedByS=observation["opening"]["fullyThroughByS"],releaseS=shot["end"],native=pins["native"],
                firstMark=carry["firstMark"],fieldYawDeg=carry["fieldYawDeg"],chassisFieldXY=[x,y],
                firstPickupStop=pins["stop"],carry=carry["trace"],shot=shot["traceSha256"],field=field["trace"],
                nextPickup=ret["trace"],fieldRoute=digest(a.route_clearance),flightClearance=digest(a.flight_clearance),
                shellRefinement=digest(a.shell_refinement),sensorObservation=observation)


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for key in ("delivery","shot","flight","acquisition","sensor","route-clearance","flight-clearance",
                "delivery-clearance","shot-clearance","shell-refinement","output"):
        p.add_argument("--"+key,type=Path,required=True)
    args=p.parse_args()
    result=verify(args)
    with args.output.open("x") as stream:
        json.dump(result,stream,indent=2,allow_nan=False)
    print(result["piece"],result["scoreS"])
