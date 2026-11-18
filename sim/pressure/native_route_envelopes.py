"""Bound a linearly interpolated planar NOTE trajectory in each native CAD frame.

Write one midpoint torus and three outward tube-radius increments per interval.
Inputs and output are private. All distances sent to Fusion are millimeters.
"""
import argparse
import json
import math
from pathlib import Path

GROUPS = ("fixed", "carriage", "cradle")


def pose(state):
    return dict(t=state["timeS"], x=1000*(state["note"]["p"][0]-state["chassis"]["x"]),
                y=1000*(state["note"]["p"][1]-state["chassis"]["y"]), z=1000*state["noteZ"],
                p=state["notePitch"], theta=state["theta"], pivot=1000*state["zPivot"])


def arc_bounds(a, b, radius, pivot_x):
    dx, dy, dz, dp, tilt, lift = (b[key]-a[key] for key in ("x","y","z","p","theta","pivot"))
    horizontal = math.hypot(dx, dy)
    fixed = math.hypot(horizontal, dz) + radius*abs(dp)
    sliding = math.hypot(horizontal, dz-lift)
    arm = max(math.hypot(q["x"]-pivot_x, q["z"]-q["pivot"]) for q in (a,b))
    return (fixed, sliding+radius*abs(dp), sliding+arm*abs(tilt)+radius*abs(dp-tilt))


def envelopes(rows, stride, radius, pivot_x, tube=25.4, tolerance=.002):
    for first in range(0, len(rows)-1, stride):
        last = min(first+stride, len(rows)-1)
        mid = (first+last)//2
        before, after = [0.]*3, [0.]*3
        for i in range(first,last):
            side = before if i < mid else after
            for k, bound in enumerate(arc_bounds(rows[i],rows[i+1],radius,pivot_x)):
                side[k] += bound
        padding = {group:max(before[i],after[i])+tolerance for i,group in enumerate(GROUPS)}
        # All registered rotations are about Y. Each torus extends exactly R+r
        # laterally even at high pitch. Endpoint extrema bound each linear segment.
        ys = [row["y"] for row in rows[first:last+1]]
        y_bounds = [min(ys)-radius-tube-tolerance, max(ys)+radius+tube+tolerance]
        yield dict(**rows[mid], first=first, last=last, yBoundsMm=y_bounds, begin=rows[first]["t"],
                   end=rows[last]["t"], padMm=padding)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run",type=Path)
    parser.add_argument("stop",type=Path)
    parser.add_argument("output",type=Path)
    parser.add_argument("--samples",type=int,default=40)
    args = parser.parse_args()
    if args.samples < 1:
        parser.error("samples must be positive")
    root = Path(__file__).resolve().parents[2]
    cad = json.loads((root/"cad/assembly/intake_components.json").read_bytes())
    result = json.loads((args.run/"result.json").read_bytes())
    if result["outcome"] != "HORIZON":
        parser.error("incomplete lift")
    initial = json.loads(args.stop.read_bytes())["final"]
    z0 = cad["floorCenterZ"]/1000
    initial.update(zPivot=z0, noteZ=z0, theta=0., notePitch=0.)
    rows = [pose(initial)]
    with (args.run/"trace.jsonl").open() as stream:
        rows.extend(pose(json.loads(line)["state"]) for line in stream)
    if len(rows)-1 != round(5.8/result["dt"]) or any(
            abs(b["t"]-a["t"]-result["dt"]) > 1e-9 for a,b in zip(rows,rows[1:])):
        raise ValueError("Incomplete or discontinuous trajectory")
    bounds = list(envelopes(rows,args.samples,cad["note"]["majorRadius"],cad["stageCenterX"],cad["note"]["tubeRadius"]))
    with args.output.open("x",encoding="utf-8") as out:
        json.dump(bounds,out,allow_nan=False)


if __name__ == "__main__":
    main()
