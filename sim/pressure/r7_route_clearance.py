"""Bound the R7 body, its current NOTE and unused floor stock on one D1 trip."""
import argparse
from collections import defaultdict
from pathlib import Path
import json

import numpy as np

from field_route_clearance import GAME_PIECES, PIVOT, SURFACES, audit, phase
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


def installed_field(path):
    parts = load(path)
    if (parts["leafCount"] != len(parts["parts"]) or
            parts["source"] != "42c91b3efb9521d059c13cf25e2d7dda6315bdd6b7eb1202847bfe84ffb43c1a"):
        raise ValueError("Wrong installed field revision")
    return parts["parts"]


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


def stationary_stock(parts, marks):
    """Bound the flat floor NOTES against installed parts and one another before pickup."""
    boxes = [(part["path"], box) for part in parts if part["path"][1] not in SURFACES
             for box in part.get("tightBounds", [part["bounds"]])]
    bounds = np.asarray([box for _, box in boxes])
    # Include the full free NOTE height and the nominal compressed floor position.
    extents = np.array([.1778, .1778, .0254])
    lower = {name: np.r_[pos[:2], .0254] - extents - [0., 0., .0002] for name, pos in marks.items()}
    upper = {name: np.r_[pos[:2], .0254] + extents for name, pos in marks.items()}
    clearances = {}
    for name in marks:
        gaps = np.maximum(bounds[:, 0] - upper[name], lower[name] - bounds[:, 1]).max(axis=1)
        closest = int(np.argmin(gaps))
        clearances[name] = dict(gapM=float(gaps[closest]), fieldPath=boxes[closest][0])
    pair_gaps = [float(np.maximum(lower[a]-upper[b], lower[b]-upper[a]).max())
                 for i, a in enumerate(marks) for b in list(marks)[i+1:]]
    return dict(structure=clearances, minimumStockPairGapM=min(pair_gaps),
                clear=min(min(pair_gaps), *(row["gapM"] for row in clearances.values())) > 0)


def western_extent(rows, cad, waiting=()):
    """Bound every robot and supply pose along the field X axis."""
    reach = 0.
    for item in cad:
        box = np.asarray(item["boundsMm"]).reshape(2,3)/1000
        extent = np.abs(box).max(axis=0)
        if item["motionGroup"] == "cradle":
            # The cradle pitches about the local Y axis. Lift is vertical.
            arm = np.abs(box-PIVOT).max(axis=0)
            extent[0] = abs(PIVOT[0]) + np.hypot(arm[0], arm[2])
        reach = max(reach, float(np.hypot(extent[0], extent[1])))
    centers = np.c_[rows[:,1:3], np.zeros(len(rows))]
    chassis = (centers-[.658,0,0]) @ FRAME.rotation.T + FRAME.origin_mark
    xmin = float(chassis[:,0].min()-reach)
    if rows.shape[1] == 9:
        piece = (rows[:,6:9]-[.658,0,0]) @ FRAME.rotation.T + FRAME.origin_mark
        xmin = min(xmin, float(piece[:,0].min()-.1778))
    for xy in waiting:
        xmin = min(xmin, float(xy[0]-.1778))
    return xmin-3e-6


def parked(hold, flight, field, survey, manifest):
    from post_release_hold import verify_hold
    record = load(hold/"result.json")
    verify_hold(hold, record)
    shot = load(flight/"result.json")
    if (record["firstReleaseTrace"] != shot["traceSha256"] or record["start"] != shot["end"]
            or record["nativeSha256"] != shot["nativeSha256"] or record["collectedMarks"] != list(MARKS)
            or not verify_binding(record) or verify_binding(record) != verify_binding(shot)):
        raise ValueError("Final hold and final release differ")
    trace = flight/"trace.jsonl"
    if digest(trace) != record["firstReleaseTrace"]:
        raise ValueError("Final release trace changed")
    with trace.open("rb") as stream:
        for line in stream:
            last_line = line
    last = json.loads(last_line)
    if (record["initialRobotQ"] != last["q"][:-7] or record["initialRobotV"] != last["v"][:-6]
            or record["initialRotors"] != last["externalRotors"] or record["initialShearM"] != last["shearM"]
            or record["initialBusV"] != last["busV"] or record["initialSourceJ"] != last["sourceJ"]):
        raise ValueError("Parked robot does not continue the recorded final state")
    obstacles, _ = registered_stock(installed_field(field))
    rows = np.vstack(([record["start"], *record["initialRobotQ"][:5]], phase(hold)))
    cad = inventory(survey, manifest)
    result = audit(rows, FRAME, cad, obstacles, [], summarize_pairs=True)
    return dict(**result, trace=record["trace"], release=record["firstReleaseTrace"], fieldSha256=digest(field),
                westernExtentM=western_extent(rows, cad))


def screen(route, mark, field, survey, manifest, until=None):
    pickup, carry, shot = [route/(mark+"-"+s) for s in ("pickup","carry","shot")]
    start = load(pickup/"result.json")
    binding = verify_binding(start)
    if not binding or start["collectedMarks"] != list(MARKS[:MARKS.index(mark)+1]):
        raise ValueError("Expected the R7 route and fresh ordered supply")
    parts = installed_field(field)
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
    obstacles, occurrences = registered_stock(parts)
    rows = np.array(rows)
    if until is not None:
        if not rows[0,0] < until <= rows[-1,0]:
            raise ValueError("Observation outside this trip")
        rows = rows[:np.searchsorted(rows[:,0], until) + 1]
    result = audit(rows,FRAME,cad,obstacles,waiting,summarize_pairs=True)
    return dict(piece=start["pieceId"],fieldSha256=digest(field),massBinding=binding,envelopes=len(cad),
                stockOccurrenceIndices=occurrences, westernExtentM=western_extent(rows, cad, waiting),
                traces=[digest(folder/"trace.jsonl") for folder in (pickup,carry,shot)],**result)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ("route","field","survey","manifest"):
        parser.add_argument(name,type=Path)
    parser.add_argument("mark",choices=MARKS)
    parser.add_argument("--until", type=float, help="Include the full step containing this observation")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--stock-placement", action="store_true", help="Check stationary supply before the first drive")
    mode.add_argument("--final-hold", type=Path, help="Check the parked robot after the final release")
    args=parser.parse_args()
    if args.stock_placement:
        if args.mark != MARKS[0]:
            parser.error("Stock placement requires the first mark")
        source = load(args.route/(args.mark+"-pickup")/"result.json")
        marks = placed_marks(FRAME, source, DEVELOPMENT_STARTS)
        obstacles, indices = registered_stock(installed_field(args.field))
        result = dict(**stationary_stock(obstacles, marks), sourceStock=indices,
                      fieldSha256=digest(args.field), rosterSha256=source["rosterSha256"])
    elif args.final_hold:
        if args.mark != MARKS[-1]:
            parser.error("Final hold requires the last mark")
        result = parked(args.final_hold, args.route/(args.mark+"-shot"), args.field, args.survey, args.manifest)
    else:
        result = screen(args.route,args.mark,args.field,args.survey,args.manifest,args.until)
    print(json.dumps(result,indent=2,allow_nan=False))
