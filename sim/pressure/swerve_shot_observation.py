"""Trace a swerve development shot from the seated NOTE to both Speaker sensors."""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from mission_runtime import verify_runtime_chain
from run_axial_shear_lift import digest, load
from speaker_score_observer import observe
from verify_swerve_carry import verify
from wing_field_frame import WingFrame
from wing_note_delivery import tail
from wing_speaker_passage import rotation


def measure(pickup, carry, shot, field, sensor):
    chain = verify(pickup,carry)
    delivered,released,flight = [load(folder/"result.json") for folder in (carry,shot,field)]
    verify_runtime_chain([delivered,released])
    start = tail(carry,delivered)
    handoff = {"initialQ":"q","initialV":"v","initialBusV":"busV","initialSourceJ":"sourceJ",
               "initialRotors":"rotor","initialShearM":"shear","start":"t"}
    if (released.get("drivetrain")!="maxswerve" or released["outcome"]!="UPWARD_FREE_RELEASE"
            or released["sourceTraceSha256"]!=delivered["trace"]
            or any(released[k]!=delivered[k] for k in ("nativeSha256","attempt","rosterSha256",
                                                       "pieceId","fieldYawDeg","firstMark"))
            or any(released[key]!=start[value] for key,value in handoff.items())):
        raise ValueError("Discontinuous swerve shooting state")
    trace = shot/"trace.jsonl"
    if (digest(trace)!=released["traceSha256"] or flight["sourceTrace"]!=released["traceSha256"]
            or flight["segmentStartS"]!=released["end"] or flight["releaseTimeS"]!=released["end"]):
        raise ValueError("Field replay belongs to a different release")
    with trace.open() as stream:
        for line in stream:
            last = line
    end = json.loads(last)
    if end["time"]!=released["end"] or end["wheelNormalN"] or end["beltNormalN"]:
        raise ValueError("Missing the first free-flight state")
    with (field/"trace.jsonl").open() as stream:
        first = json.loads(next(stream))
    frame = WingFrame(released["firstMark"],released["fieldYawDeg"])
    pos = frame.to_field(end["q"][-7:-4])
    axes = frame.rotation @ rotation(end["q"][-4:])
    velocity = np.r_[frame.rotation @ end["v"][-6:-3],end["v"][-3:]]
    if (not math.isclose(first["t"],end["time"],rel_tol=0,abs_tol=1e-10)
            or not np.allclose(pos,first["q"][:3],rtol=0,atol=1e-10)
            or not np.allclose(axes,rotation(first["q"][3:]),rtol=0,atol=1e-10)
            or not np.allclose(velocity,first["v"],rtol=0,atol=1e-10)):
        raise ValueError("Field handoff changes NOTE pose, velocity or time")
    observed = observe(field,sensor)
    event = observed["firstSimultaneousFullShadowS"]
    if event is not None and (not observed["opening"] or observed["opening"]["fullyThroughByS"]>=event):
        raise ValueError("Sensor event precedes full NOTE passage")
    return dict(attempt=chain["attempt"],carryTrace=chain["carryTrace"],sourceNativeSha256=chain["nativeSha256"],
                stationS=start["t"],releaseS=end["time"],minimumBusV=released["minimumBusV"],
                peakDriveAndSteerCurrentA=released["peakDriveAndSteerCurrentA"],**observed)


if __name__=="__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("pickup","carry","shot","field","sensor"):
        parser.add_argument(name,type=Path)
    args = parser.parse_args()
    print(json.dumps(measure(args.pickup,args.carry,args.shot,args.field,args.sensor),indent=2,allow_nan=False))
