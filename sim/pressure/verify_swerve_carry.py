"""Summarize a continuous four-module development pickup and loaded carry."""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from mission_runtime import verify_runtime_chain
from run_axial_shear_lift import ROOT, digest, load


def verify(pickup_dir, carry_dir):
    pickup = load(pickup_dir/"result.json")
    carry = load(carry_dir/"result.json")
    verify_runtime_chain([pickup, carry])
    layout = ROOT/"cad/drivetrain/maxswerve-layout.json"
    if (pickup["schema"] != "maxswerve-first-wing-acquisition/1" or pickup["outcome"] != "SEATED"
            or carry["schema"] != "maxswerve-wing-note-delivery/1" or carry["outcome"] != "AT_SHOOTING_POSE"
            or pickup["layoutSha256"] != digest(layout)
            or pickup["nativeSha256"] != load(layout)["privateFusionArchiveSha256"]
            or any(pickup[k] != carry[k] for k in ("nativeSha256", "attempt", "rosterSha256", "pieceId", "grid", "dt"))
            or carry["acquisitionTrace"] != pickup["trace"]):
        raise ValueError("Mismatched robot, setup, piece or stage outcome")
    paths = [p/"trace.jsonl" for p in (pickup_dir, carry_dir)]
    if any(digest(p) != s["trace"] for p, s in zip(paths, (pickup, carry))):
        raise ValueError("Trace checksum mismatch")
    with paths[0].open() as stream:
        for line in stream:
            last = line
    seated = json.loads(last)
    if (seated["q"] != carry["initialQ"] or seated["v"] != carry["initialV"]
            or seated["t"] != carry["start"] or pickup["end"] != seated["t"]):
        raise ValueError("Discontinuous state at pickup-to-carry boundary")
    previous = seated
    duration = 0.
    min_load = math.inf
    min_bus = pickup["minimumBusV"]
    with paths[1].open() as stream:
        for line in stream:
            row = json.loads(line)
            dt = row["t"] - previous["t"]
            if not math.isclose(dt, carry["dt"], abs_tol=1e-9):
                raise ValueError("Expected a carry row at every integration step")
            vx, vy, turn, lift, pitch = row["v"][:5]
            yaw = row["q"][2]
            sideways = vy*math.cos(yaw) - vx*math.sin(yaw)
            if abs(sideways) > .05 and abs(turn) > .05 and (abs(lift) > .002 or abs(pitch) > .02):
                duration += dt
            min_load = min(min_load, *row["loads"])
            min_bus = min(min_bus, row["busV"])
            previous = row
    if (duration < .25 or min_load <= .02 or min_bus <= 0.
            or previous["positionError"] >= .035 or abs(previous["headingError"]) >= .012
            or np.linalg.norm(previous["v"][:2]) >= .035 or abs(previous["v"][2]) >= .035
            or previous["t"] != carry["end"]):
        raise ValueError("Simultaneous lift, strafe and turn, retention or station arrival failed")
    return dict(attempt=pickup["attempt"], dt=carry["dt"], grid=carry["grid"], runtime=carry["runtime"],
                rosterSha256=pickup["rosterSha256"], nativeSha256=pickup["nativeSha256"],
                pickupTrace=pickup["trace"], carryTrace=carry["trace"], firstBilateralS=pickup["firstBilateral"],
                seatedS=seated["t"], shootingPoseS=carry["end"], simultaneousStrafeTurnLiftS=duration,
                minimumBusV=min_bus, minimumCarrySideLoadN=min_load,
                peakDriveAndSteerCurrentA=carry["peakDriveAndSteerCurrentA"],
                stationPositionErrorM=previous["positionError"], stationHeadingErrorRad=previous["headingError"])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pickup", type=Path)
    parser.add_argument("carry", type=Path)
    args = parser.parse_args()
    print(json.dumps(verify(args.pickup, args.carry), indent=2, allow_nan=False))
