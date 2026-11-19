"""Summarize a continuous four-module development pickup and loaded carry."""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from mission_runtime import verify_runtime_chain
from run_axial_shear_lift import ROOT, digest, load


def check_departure(pickup, release_dir, roster):
    """Reconcile the released robot and the next resting NOTE on the same field clock."""
    from speaker_trial_inputs import FRAME, local_offset, read_attempt
    from wing_field_frame import piece_id
    source = load(release_dir/"result.json")
    trace = release_dir/"trace.jsonl"
    if (source["outcome"] != "UPWARD_FREE_RELEASE" or source.get("drivetrain") != "maxswerve"
            or digest(trace) != source["traceSha256"] or pickup["firstReleaseTrace"] != source["traceSha256"]
            or source["rosterSha256"] != digest(roster) or source["firstMark"] != FRAME.first_mark
            or source["fieldYawDeg"] != FRAME.yaw_degrees
            or any(source[k] != pickup[k] for k in ("nativeSha256","rosterSha256","attempt","firstMark","fieldYawDeg"))):
        raise ValueError("Release or field setup differs from the pickup")
    marks = pickup["collectedMarks"]
    if (not marks or marks[:-1] != source["collectedMarks"] or len(set(marks)) != len(marks)
            or pickup["pieceId"] != piece_id(marks[-1])):
        raise ValueError("The next pickup must add a distinct NOTE")
    with trace.open("rb") as stream:
        for line in stream:
            last_line = line
    last = json.loads(last_line)
    if (pickup["initialQ"][:-7] != last["q"][:-7] or pickup["initialV"][:-6] != last["v"][:-6]
            or pickup["start"] != source["end"] or pickup["start"] != last["time"]
            or pickup["initialRotors"] != last["externalRotors"] or pickup["initialShearM"] != last["shearM"]
            or pickup["initialBusV"] != last["busV"] or pickup["initialSourceJ"] != last["sourceJ"]
            or last["wheelNormalN"] or last["beltNormalN"] or any(last["shearM"])):
        raise ValueError("Discontinuous swerve robot at the released NOTE")
    nominal = FRAME.to_local(FRAME.mark(marks[-1]))[:2]
    waiting = nominal + local_offset(read_attempt(roster,pickup["attempt"]),marks[-1])
    floor = load(ROOT/"sim/pressure/evidence/native-lift-20261118/native-pressure-lift-contract.json")
    mass = load(ROOT/"sim/component-launcher/six-drive-config.json")["pieceMassKg"]
    expected = [*waiting,.0254-mass*9.80665/floor["floorStiffnessNpm"],1.,0.,0.,0.]
    if (not np.allclose(pickup["initialQ"][-7:],expected,atol=1e-12,rtol=0)
            or any(pickup["initialV"][-6:]) or not np.allclose(nominal,pickup["targetLocalXY"],atol=1e-12,rtol=0)):
        raise ValueError("Next NOTE differs from its registered resting placement")


def verify(pickup_dir, carry_dir, *, release_dir=None, roster=None):
    pickup = load(pickup_dir/"result.json")
    carry = load(carry_dir/"result.json")
    verify_runtime_chain([pickup, carry])
    layout = ROOT/"cad/drivetrain/maxswerve-layout.json"
    if (release_dir is None) != (roster is None):
        raise ValueError("Provide both the preceding release and placement roster for a later NOTE")
    expected_schema = "maxswerve-first-wing-acquisition/1"
    if release_dir is not None:
        expected_schema = "maxswerve-next-wing-acquisition/1"
        check_departure(pickup,release_dir,roster)
    if (pickup["schema"] != expected_schema or pickup["outcome"] != "SEATED"
            or carry["schema"] != "maxswerve-wing-note-delivery/1" or carry["outcome"] != "AT_SHOOTING_POSE"
            or pickup["layoutSha256"] != digest(layout)
            or pickup["nativeSha256"] != load(layout)["privateFusionArchiveSha256"]
            or any(pickup[k] != carry[k] for k in ("nativeSha256", "attempt", "rosterSha256", "pieceId", "grid", "dt"))
            or pickup["collectedMarks"] != carry["collectedMarks"] or carry["acquisitionTrace"] != pickup["trace"]):
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
    parser.add_argument("--release",type=Path)
    parser.add_argument("--roster",type=Path)
    args = parser.parse_args()
    print(json.dumps(verify(args.pickup, args.carry, release_dir=args.release, roster=args.roster), indent=2, allow_nan=False))
