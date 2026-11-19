"""Bound module-to-moving-mechanism clearance from private, robot-registered CAD inventories."""
import argparse
import hashlib
import itertools
import json
import math
from pathlib import Path

import numpy as np

LAYOUT = Path(__file__).resolve().parents[1] / "drivetrain/maxswerve-layout.json"
PIVOT_MM = np.array([178., 0., 25.4])
MOUNT_TURNS = {"Front left":0,"Front right":3,"Rear left":1,"Rear right":2}


def corners(bounds):
    return np.array(list(itertools.product(*np.array(bounds).reshape(2,3).T)))


def verify(robot, supplier, layout):
    if len(supplier)!=72:
        raise ValueError("Unexpected supplier module inventory")
    modules = []
    for name,quarter_turns in MOUNT_TURNS.items():
        origin = np.array(layout["stations"][name]["wheelCenterMm"])
        c,s = round(math.cos(quarter_turns*math.pi/2)),round(math.sin(quarter_turns*math.pi/2))
        basis = np.array([[-s,0,c],[c,0,s],[0,1,0]])
        for part in supplier:
            pts = corners(part["bounds"])@basis.T+origin
            modules.append(np.r_[pts.min(axis=0),pts.max(axis=0)])
        # The pin-bound supplier roll and azimuth families fit inside this
        # conservative column. All stationary supplier boxes are also retained.
        x,y = origin[:2]
        modules.append([x-45,y-45,-2,x+45,y+45,98])
    return mechanism_clearance(robot, modules)


def mechanism_clearance(robot, fixed_bounds):
    moving = [r for r in robot["inventory"] if r["visible"] and r["role"]!="reserved-space"
              and r["motionGroup"] in ("carriage","cradle")]
    if len(moving)!=188:
        raise ValueError("Unexpected source body or module inventory")
    boxes = np.array([corners(row["boundsMm"]) for row in moving])
    pitches = np.array([row["motionGroup"]=="cradle" for row in moving])
    modules = np.asarray(fixed_bounds)
    if modules.ndim != 2 or modules.shape[1] != 6 or not np.isfinite(modules).all():
        raise ValueError("Expected finite Cartesian bounds for each stationary part")
    if not (modules[:,3:] > modules[:,:3]).all():
        raise ValueError("Empty stationary volume")
    reach = np.linalg.norm((boxes-PIVOT_MM)[:,:,[0,2]],axis=2).max(axis=1)
    pad = 2*math.sin(math.radians(.25))*reach*pitches
    margin = math.inf
    for lower_angle in range(60):
        angle = math.radians(lower_angle+.5)
        c,s = math.cos(angle),math.sin(angle)
        pitch = np.array([[c,0,-s],[0,1,0],[s,0,c]])
        at_mid = boxes.copy()
        at_mid[pitches] = (boxes[pitches]-PIVOT_MM)@pitch.T+PIVOT_MM
        # A half-degree arc to either interval endpoint moves any point by
        # at most the chord pad. Lift independently fills the entire Z range.
        lower = at_mid.min(axis=1)-pad[:,None]*[1,0,1]
        upper = at_mid.max(axis=1)+pad[:,None]*[1,0,1]+[0,0,450]
        pair_gaps = np.maximum(modules[:,None,:3]-upper,lower-modules[:,None,3:]).max(axis=2)
        margin = min(margin,float(np.min(pair_gaps)))
    if margin<=0:
        raise ValueError("Overlapping bounds require a native solid query")
    return {"mechanismBodies":len(moving),"pitchIntervals":60,"liftRangeMm":[0,450],
            "pitchRangeDeg":[0,60],"minimumAxisSeparationMm":margin}


if __name__=="__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("robot_inventory",type=Path)
    parser.add_argument("supplier_inventory",type=Path)
    args = parser.parse_args()
    layout = json.loads(LAYOUT.read_bytes())
    inventories = []
    for key,path in (("mechanismInventory",args.robot_inventory),("moduleInventory",args.supplier_inventory)):
        raw = path.read_bytes()
        if hashlib.sha256(raw).hexdigest()!=layout["articulation"]["sourceHashes"][key]:
            raise ValueError(f"{key} differs from the registered native study")
        inventories.append(json.loads(raw))
    print(json.dumps(verify(*inventories,layout),indent=2))
