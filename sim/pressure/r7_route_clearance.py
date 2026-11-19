"""Bound the R7 body, its current NOTE and unused floor stock on one D1 trip."""
import argparse
from collections import defaultdict
from pathlib import Path
import json

import numpy as np

from field_route_clearance import GAME_PIECES, audit, phase
from r7_dynamics import BUDGET, LAYOUT, verify_binding
from run_axial_shear_lift import digest, load
from speaker_trial_inputs import FRAME, MARKS, local_offset, placed_marks, read_attempt
from swerve_development_route import DEVELOPMENT_STARTS


def inventory(survey, manifest):
    budget = load(BUDGET)
    if digest(survey)!=budget["surveySha256"] or digest(manifest)!=budget["inventorySha256"]:
        raise ValueError("Unregistered mass survey or motion inventory")
    display = load(manifest)
    groups = {key:mesh["name"].split("-")[0] for mesh in display["meshes"] for key in mesh["sourceBodyKeys"]}
    motion = defaultdict(set)
    for item in display["bodyInventory"]:
        motion[item["parent"],item["component"],item["name"]].add(groups[item["key"]])
    rows = []
    for item in load(survey)["rows"]:
        parent = item["path"].split("+")[0].rsplit(":",1)[0]
        if (parent.startswith("MAXSwerve ") or "withdrawal reserve" in item["name"]
                or item["body"]=="Continuous covered bumper envelope"):
            continue
        cushion = "noodle" in item["name"] or "bumper wood" in item["name"]
        if not item["visible"] and not cushion:
            continue
        group, = motion.get((parent,item["name"],item["body"]), {"fixed"})
        box = np.array(item["bboxMm"])
        if cushion:
            box += [-1.5]*3 + [1.5]*3
        rows.append(dict(key=item["path"]+"/"+item["body"],motionGroup=group,boundsMm=box.tolist()))
    for name,station in load(LAYOUT)["stations"].items():
        lo,hi = np.array(station["installedBoundsMm"]).reshape(2,3)
        xy = np.array(station["wheelCenterMm"][:2])
        radius = np.linalg.norm(np.maximum(abs(lo[:2]-xy),abs(hi[:2]-xy)))
        rows.append(dict(key=name+" full module azimuth envelope",motionGroup="fixed",
                         boundsMm=[*(xy-radius),lo[2],*(xy+radius),hi[2]]))
    return rows


def registered_stock(parts):
    """Replace only CAD stock on the six roster marks, retaining every other occurrence."""
    occurrences = {mark: [] for mark in MARKS}
    obstacles = []
    for index, part in enumerate(parts):
        if part["path"][1] not in GAME_PIECES:
            obstacles.append(part)
            continue
        center = np.mean(part["bounds"], axis=0)
        matches = [m for m in MARKS if np.linalg.norm(center - FRAME.mark(m) - [0, 0, .0254]) <= .001]
        if matches:
            mark, = matches
            occurrences[mark].append(index)
        else:
            obstacles.append({**part, "path": ["retained field stock", str(index), *part["path"]]})
    if not all(occurrences.values()):
        raise ValueError("A roster placement has no independently registered source NOTE")
    return obstacles, occurrences


def screen(route, mark, field, survey, manifest, until=None):
    pickup, carry, shot = [route/(mark+"-"+s) for s in ("pickup","carry","shot")]
    start = load(pickup/"result.json")
    binding = verify_binding(start)
    if not binding or start["collectedMarks"] != list(MARKS[:MARKS.index(mark)+1]):
        raise ValueError("Expected the R7 route and fresh ordered supply")
    parts = load(field)
    if parts["leafCount"] != len(parts["parts"]) or parts["source"]!="42c91b3efb9521d059c13cf25e2d7dda6315bdd6b7eb1202847bfe84ffb43c1a":
        raise ValueError("Wrong field revision")
    marks = placed_marks(FRAME,start,DEVELOPMENT_STARTS)
    q = start["initialQ"]
    if not np.allclose(FRAME.to_field(q[-7:-4])[:2],marks[mark][:2],rtol=0,atol=1e-10):
        raise ValueError("Active NOTE did not start at its registered mark")
    if mark=="middle":
        setup = read_attempt(DEVELOPMENT_STARTS,start["attempt"])
        pose = [*local_offset(setup,"robot"),np.deg2rad(setup["robot_heading_deg"]),0,0]
        if start["start"]!=0 or not np.allclose(q[:5],pose,rtol=0,atol=1e-10) or any(start["initialV"]):
            raise ValueError("Unexpected initial robot state")
    cad = inventory(survey,manifest)
    waiting = [p[:2] for k,p in marks.items() if k not in start["collectedMarks"]]
    rows = [np.array([start["start"],*q[:5],*q[-7:-4]])]
    for folder in (pickup,carry,shot):
        rc = load(folder/"result.json")
        if verify_binding(rc)!=binding or rc["pieceId"]!=start["pieceId"]:
            raise ValueError("Different piece or mass case in trip")
        rows.extend(phase(folder))
    obstacles, occurrences = registered_stock(parts["parts"])
    rows = np.array(rows)
    if until is not None:
        if not rows[0,0] < until <= rows[-1,0]:
            raise ValueError("Observation outside this trip")
        rows = rows[:np.searchsorted(rows[:,0], until) + 1]
    result = audit(rows,FRAME,cad,obstacles,waiting,summarize_pairs=True)
    return dict(piece=start["pieceId"],fieldSha256=digest(field),massBinding=binding,envelopes=len(cad),
                stockOccurrenceIndices=occurrences,
                traces=[digest(folder/"trace.jsonl") for folder in (pickup,carry,shot)],**result)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ("route","field","survey","manifest"):
        parser.add_argument(name,type=Path)
    parser.add_argument("mark",choices=MARKS)
    parser.add_argument("--until", type=float, help="Include the full step containing this observation")
    args=parser.parse_args()
    print(json.dumps(screen(args.route,args.mark,args.field,args.survey,args.manifest,args.until),indent=2))
