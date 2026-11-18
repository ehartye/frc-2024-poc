"""Independently check continuity and count the two distinct wing pieces on one clock."""
import argparse
import json
from pathlib import Path

import numpy as np

from run_axial_shear_lift import EVIDENCE, digest, load
from speaker_score_observer import observe
from wing_field_frame import WingFrame, piece_id


def load_trace(folder, field="trace"):
    result = load(folder/"result.json")
    path = folder/"trace.jsonl"
    if digest(path) != result[field]:
        raise ValueError("Source trajectory hash mismatch")
    with path.open("rb") as f:
        rows = [json.loads(line) for line in f]
    if not rows:
        raise ValueError("Empty trajectory")
    return result, rows


def equal_state(a, b):
    if not np.array_equal(a,b):
        raise ValueError("Physical state discontinuity")


def distinct_events(attempts):
    pieces = set()
    traces = set()
    scores = []
    for piece, release_trace, result in attempts:
        if piece in pieces or release_trace in traces or result["sourceTrace"] != release_trace:
            raise ValueError("Duplicate piece or unrelated field history")
        pieces.add(piece)
        traces.add(release_trace)
        event = result["firstSimultaneousFullShadowS"]
        if event is not None:
            if not np.isfinite(event) or not result["opening"] or result["opening"]["fullyThroughByS"] >= event:
                raise ValueError("Sensor event precedes complete opening passage")
            scores.append(dict(piece=piece, timeS=event))
    return sorted(scores, key=lambda score:score["timeS"])


def validate(args):
    reference = load(EVIDENCE/"speaker-counter-20261118.json")
    first, initial = load_trace(args.first_release,"traceSha256")
    pickup, picked = load_trace(args.acquisition)
    carried, delivered = load_trace(args.delivery)
    shot, fired = load_trace(args.second_release,"traceSha256")
    first_hash = first["traceSha256"]
    if not (reference["runs"]["fine"]["releaseTrace"] == pickup["firstReleaseTrace"] == first_hash
            and pickup["trace"] == carried["acquisitionTrace"] and carried["trace"] == shot["sourceTraceSha256"]
            and pickup["outcome"] == "SEATED" and carried["outcome"] == "AT_SHOOTING_POSE"
            and shot["outcome"] == "UPWARD_FREE_RELEASE" and shot["pieceId"] == "blue-wing-middle"):
        raise ValueError("Unrelated or unfinished physical stages")
    if not all(r["nativeSha256"] == reference["nativeArchive"] for r in (first,pickup,carried,shot)):
        raise ValueError("Native assembly mismatch")
    frame = WingFrame(pickup.get("firstMark","inner"), pickup.get("fieldYawDeg",90.))
    if frame.first_mark=="middle":
        raise ValueError("This two-piece comparison requires two distinct wing marks")
    for receipt in (carried,shot):
        if (receipt.get("firstMark","inner"),receipt.get("fieldYawDeg",90.))!=(frame.first_mark,frame.yaw_degrees):
            raise ValueError("Robot stages use different field registrations")
    for folder in (args.first_field,args.second_field):
        params=load(folder/"result.json")["parameters"]
        if (params.get("first_mark","inner"),params.get("field_yaw_deg",90.))!=(frame.first_mark,frame.yaw_degrees):
            raise ValueError("Counter and robot use different field registrations")
    equal_state(pickup["initialRobotQ"],initial[-1]["q"][:-7])
    equal_state(pickup["initialRobotV"],initial[-1]["v"][:-6])
    equal_state(pickup["initialRotors"],initial[-1]["externalRotors"])
    equal_state(pickup["initialBusV"],initial[-1]["busV"])
    # Euler's first translation step independently identifies the waiting NOTE.
    position = np.array(picked[0]["q"][-7:-4])-pickup["dt"]*np.array(picked[0]["v"][-6:-3])
    expected = [*frame.to_local(frame.mark("middle"))[:2],.0254-.235*9.80665/20000]
    if not np.allclose(position,expected,rtol=0.,atol=1e-10):
        raise ValueError("Second pickup did not start on the middle wing mark")
    for before, after in ((picked[-1],carried),(delivered[-1],shot)):
        equal_state(before["q"],after["initialQ"])
        equal_state(before["v"],after["initialV"])
    equal_state([first["end"],pickup["end"],carried["end"]],[pickup["start"],carried["start"],shot["start"]])
    for result,rows,key in ((pickup,picked,"t"),(carried,delivered,"t"),(shot,fired,"time")):
        times=np.array([r[key] for r in rows])
        stride=result.get("traceStride",round(.01/result["dt"]))
        deltas=np.diff(times)
        if not (abs(times[0]-result["start"]-result["dt"])<1e-8 and abs(times[-1]-result["end"])<1e-8
                and np.all(deltas>0) and np.all(deltas<=stride*result["dt"]+1e-8)
                and all(np.isfinite(r["q"]).all() and np.isfinite(r["v"]).all() for r in rows)):
            raise ValueError("Timing or state coverage failure")
    dwell=[r for r in picked if r["t"]>=pickup["end"]-.15-1e-8]
    if (min(min(r["loads"]) for r in dwell)<=.02
            or max(abs(r["relative"][0]-.178) for r in dwell)>=.012
            or max(abs(r["relative"][1]) for r in dwell)>=.004):
        raise ValueError("Recorded bilateral seating dwell failed")
    if min(min(r["loads"]) for r in delivered)<=.02 or delivered[-1]["positionError"]>=.035:
        raise ValueError("NOTE lost during transport or shooting pose missed")
    if not (any(r["beltNormalN"]>1e-6 and r["wheelNormalN"]>1e-6 for r in fired)
            and fired[-1]["beltNormalN"]==fired[-1]["wheelNormalN"]==0.
            and fired[-1]["v"][-4]>0.):
        raise ValueError("Missing powered handoff and free upward release")
    obs1=observe(args.first_field,args.sensor)
    obs2=observe(args.second_field,args.sensor)
    scores=distinct_events([(piece_id(frame.first_mark),first_hash,obs1),(shot["pieceId"],shot["traceSha256"],obs2)])
    if len(scores)!=2 or scores[-1]["timeS"]>120.:
        raise ValueError("Two distinct on-time scores not observed")
    return dict(schema="two-wing-counter-events/1",counterEvents=scores,wholeFieldClearanceVerified=False,pickupStartedS=pickup["start"],
                secondSeatedS=pickup["end"],arrivedAtS=carried["end"],secondReleaseS=shot["end"],
                minimumLoadedSideForceN=carried["minimumSideLoadN"],firstRelease=first_hash,
                pickupTrace=pickup["trace"],carryTrace=carried["trace"],secondRelease=shot["traceSha256"],
                firstField=obs1["fieldTrace"],secondField=obs2["fieldTrace"],secondObservation=obs2)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ("first-release","acquisition","delivery","second-release","first-field","second-field","sensor","output"):
        parser.add_argument("--"+name,type=Path,required=True)
    args=parser.parse_args()
    result=validate(args)
    with args.output.open("x") as stream:
        json.dump(result,stream,indent=2,allow_nan=False)
        stream.write("\n")
    print(json.dumps(result["counterEvents"]))
