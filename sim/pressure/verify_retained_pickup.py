"""Check a finite hold from every integrated state and compare shared time samples."""
import argparse
import json
import math
from pathlib import Path

import numpy as np


def evaluate(folder):
    result = json.loads((folder/"result.json").read_bytes())
    trace = [json.loads(line) for line in (folder/"trace.jsonl").read_bytes().splitlines()]
    dt, start = result["dt"], result["startTimeS"]
    if result["outcome"] != "HORIZON" or len(trace) != round(.8/dt):
        raise ValueError("Lift window incomplete")
    states, clearances, pivots, support, floors, energy = [], [], [], [], [], []
    for step, row in enumerate(trace, 1):
        s = row["state"]
        if abs(row["timeS"]-start-step*dt) > 1e-9 or s["timeS"] != row["timeS"]:
            raise ValueError("Non-contiguous simulation clock")
        states.append([s["chassis"]["x"],s["note"]["p"][0],s["zPivot"],s["noteZ"],
                       s["chassis"]["vx"],s["note"]["v"][0],s["vPivot"],s["noteVz"],
                       s["theta"],s["notePitch"],s["thetaRate"],s["notePitchRate"]])
        clearances.append(s["noteZ"]-.0254-.1524*abs(math.sin(s["notePitch"])))
        pivots.append(s["zPivot"]-.0254)
        support.append(len(row["normalN"]) == 2 and min(row["normalN"]) > 0)
        floors.append(row["floorForceN"][2])
        energy.append(abs(row["energyJ"]["residual"]-result["inheritedResidualJ"]))
        if max(abs(row[k]) for k in ("sourcePowerResidualW","portMidpointPowerDifferenceW",
                                      "pressureWorkResidualW")) > 1e-8:
            raise ValueError("Power residual exceeded")
    a = np.asarray(states)
    if not all(np.isfinite(v).all() for v in (a,clearances,pivots,floors,energy)):
        raise ValueError("Nonfinite measurements")
    airborne = next((j for j,h in enumerate(clearances) if h > .00001),None)
    if airborne is None: raise ValueError("Never cleared the floor")
    # Tick 50 starts the 0.30 s hold. Include the state at its boundary.
    hold = slice(round(.5/dt)-1,None)
    retained = (all(h > 0 for h in clearances[airborne:]) and all(f == 0 for f in floors[airborne:])
                and all(support[airborne:]) and min(clearances[hold]) >= .006
                and max(pivots) >= .0075 and pivots[-1] >= .0075 and max(energy) <= .002)
    return a, dict(outcome=result["outcome"],retained=retained,dt=dt,grid=result.get("pressureGrid",1024),
                   steps=len(a),firstClearanceAbove10umS=(airborne+1)*dt,
                   minHoldClearanceM=min(clearances[hold]), minHoldBeltContact=all(support[hold]),
                   finalRiseM=a[-1,3]-.0254,finalPivotRiseM=pivots[-1],maxEnergyDriftJ=max(energy),
                   initialMechanicalDifferenceJ=result["initialMechanicalDifferenceJ"])


def compare(left,right):
    a,ma = left
    b,mb = right
    stride = round(ma["dt"]/mb["dt"])
    if stride < 1 or not math.isclose(ma["dt"],stride*mb["dt"]) or len(a)*stride!=len(b):
        raise ValueError("Incompatible simulation clocks")
    errors = np.abs(a-b[stride-1::stride]).max(axis=0)
    return dict(states=len(a),positionM=max(errors[:4]),velocityMps=max(errors[4:8]),
                angleRad=max(errors[8:10]),angularVelocityRadps=max(errors[10:12]),
                withinTranslationBounds=bool(max(errors[:4])<=5e-5 and max(errors[4:8])<=.001))


def verify(packet):
    times=[evaluate(packet/f"shear-{dt}") for dt in (.0005,.00025,.000125)]
    grids=[evaluate(packet/"grid-512"),times[0],evaluate(packet/"grid-2048")]
    by_time=[compare(a,b) for a,b in zip(times,times[1:])]
    by_grid=[compare(a,b) for a,b in zip(grids,grids[1:])]
    return dict(runs=[x[1] for x in times+grids[::2]],timesteps=by_time,pressureGrids=by_grid,
                retainedAll=all(x[1]["retained"] for x in times+grids),
                temporalBoundPassed=by_time[-1]["withinTranslationBounds"],
                spatialBoundPassed=by_grid[-1]["withinTranslationBounds"],hardwareValidated=False)


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("packet",type=Path)
    parser.add_argument("output",type=Path)
    args=parser.parse_args()
    result=verify(args.packet)
    with args.output.open("x",encoding="utf-8") as stream:
        json.dump(result,stream,indent=2,allow_nan=False)
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if all(result[k] for k in ("retainedAll", "temporalBoundPassed", "spatialBoundPassed")) else 1)
