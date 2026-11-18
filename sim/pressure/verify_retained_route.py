"""Measure possession throughout the finite-effort lift from a loose floor NOTE."""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np


def file_hash(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def assess(folder):
    result = json.loads((folder/"result.json").read_bytes())
    dt, start = result["dt"], result["startTimeS"]
    if dt not in (.0005,.00025,.000125) or not math.isfinite(start):
        raise ValueError("Invalid simulation clock")
    rows = 0
    lifted = False
    support_lost = False
    hold_clearance = math.inf
    max_position_error = max_pitch_error = max_energy_drift = 0.
    max_relative_travel = 0.
    states = []
    first_clear = None
    initial_relative = None
    initial_residual = None
    min_hold_traction = math.inf
    for line in (folder/"trace.jsonl").open(encoding="utf-8"):
        row = json.loads(line)
        rows += 1
        s = row["state"]
        elapsed = rows*dt
        if abs(s["timeS"]-start-elapsed) > 1e-8 or s["timeS"]!=row["timeS"]:
            raise ValueError("Noncontiguous physical trace")
        clearance = s["noteZ"]-.0254-.1524*abs(math.sin(s["notePitch"]))
        observed = [s["timeS"], row["minimumFloorClearanceM"], row["energyJ"]["residual"],
                    *row["floorForceN"], *row["normalN"], row["pressureWorkResidualW"],
                    row["portMidpointPowerDifferenceW"], row["sourcePowerResidualW"]]
        if not all(math.isfinite(value) for value in observed):
            raise ValueError("Nonfinite contact, energy or clock value")
        if abs(clearance-row["minimumFloorClearanceM"])>1e-12:
            raise ValueError("Inconsistent floor clearance")
        if clearance > .00001 and not lifted:
            first_clear = elapsed
            lifted = True
        if len(row["normalN"]) != 2 or min(row["normalN"]) <= 0:
            support_lost = True
        if lifted and (clearance<=0 or row["floorForceN"][2]!=0):
            support_lost = True
        relative = ((s["note"]["p"][0]-s["chassis"]["x"]-.178)*math.cos(s["theta"])
                    + (s["noteZ"]-s["zPivot"])*math.sin(s["theta"]))
        if initial_relative is None:
            initial_relative=relative
            initial_residual=row["energyJ"]["residual"]
        max_relative_travel=max(max_relative_travel,abs(relative-initial_relative))
        max_energy_drift=max(max_energy_drift,abs(row["energyJ"]["residual"]-initial_residual))
        if elapsed>=5.-1e-12:
            hold_clearance=min(hold_clearance,clearance)
            max_position_error=max(max_position_error,abs(s["zPivot"]-.4748333949243112))
            max_pitch_error=max(max_pitch_error,abs(s["theta"]-math.pi/3),abs(s["notePitch"]-math.pi/3))
            min_hold_traction=min(min_hold_traction,sum(row["normalN"]))
        if max(abs(row[k]) for k in ("pressureWorkResidualW","portMidpointPowerDifferenceW","sourcePowerResidualW"))>1e-8:
            raise ValueError("Unbalanced power ports")
        states.append([s["chassis"]["x"],s["note"]["p"][0],s["zPivot"],s["noteZ"],
                       s["chassis"]["vx"],s["note"]["v"][0],s["vPivot"],s["noteVz"],
                       s["theta"],s["notePitch"],s["thetaRate"],s["notePitchRate"]])
    complete=result["outcome"]=="HORIZON" and rows==round(5.8/dt)
    data=np.array(states)
    if len(data)==0 or not np.isfinite(data).all():raise ValueError("Missing or nonfinite motion")
    retained=(complete and lifted and not support_lost and hold_clearance>=.30 and
              max_position_error<=.005 and max_pitch_error<=math.radians(2) and
              max_relative_travel<=.015 and max_energy_drift<=.002)
    return data,dict(dt=dt,grid=result["grid"],steps=rows,outcome=result["outcome"],retained=retained,
                     firstAirborneS=first_clear,returnedOrLostContact=support_lost,
                     holdMinimumClearanceM=hold_clearance if math.isfinite(hold_clearance) else None,
                     holdPivotMaxErrorM=max_position_error,holdMaxAngleErrorDegrees=math.degrees(max_pitch_error),
                     holdMinimumTotalNormalN=min_hold_traction if math.isfinite(min_hold_traction) else None,
                     finalPivotHeightM=data[-1,2],finalNoteCenterHeightM=data[-1,3],
                     finalNotePitchDegrees=math.degrees(data[-1,9]),
                     maxRelativeTravelM=max_relative_travel,maxEnergyDriftJ=max_energy_drift,
                     traceSha256=file_hash(folder/"trace.jsonl"),inputs=result["inputs"])


def assess_study(packet):
    from verify_retained_pickup import compare

    steps = (.0005,.00025,.000125)
    grids = (512,1024,2048)
    def checked(step,grid):
        run = assess(packet/f"indexed-{step}-{grid}")
        if (run[1]["dt"],run[1]["grid"]) != (step,grid):
            raise ValueError("Run settings differ from the study manifest")
        return run

    temporal = [checked(dt,1024) for dt in steps]
    spatial = [checked(steps[0],512),temporal[0],checked(steps[0],2048)]
    runs = temporal + spatial[::2]
    if any(run[1]["inputs"] != runs[0][1]["inputs"] for run in runs):
        raise ValueError("Changed archive, acquisition, stop or route implementation")
    times = [compare(a,b) for a,b in zip(temporal,temporal[1:])]
    pressure = [compare(a,b) for a,b in zip(spatial,spatial[1:])]
    return dict(runs=[r[1] for r in runs],timesteps=times,pressureGrids=pressure,
                retainedAll=all(r[1]["retained"] for r in runs),
                temporalBoundPassed=times[-1]["withinTranslationBounds"],
                spatialBoundPassed=pressure[-1]["withinTranslationBounds"],
                hardwareValidated=False)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run",type=Path)
    parser.add_argument("--study",action="store_true",help="Compare three timesteps and pressure grids")
    args=parser.parse_args()
    if args.study:
        summary = assess_study(args.run)
        passed = all(summary[k] for k in ("retainedAll","temporalBoundPassed","spatialBoundPassed"))
    else:
        _, summary = assess(args.run)
        passed = summary["retained"]
    print(json.dumps(summary,indent=2,allow_nan=False))
    raise SystemExit(0 if passed else 1)
