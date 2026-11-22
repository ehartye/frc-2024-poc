"""Budget the installed R7 solids and purchased assemblies at the native floor pose."""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import re

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
# Screening ranges for the physical material, in kg/m³. Envelopes need separate treatment.
DENSITY = {
    "aluminum": (2650, 2700, 2800),
    "steel": (7700, 7850, 8050),
    "foam": (20, 32, 55),
    "plywood": (450, 600, 750),
    "elastomer": (900, 1100, 1300),
    "UHMW": (920, 940, 970),
    "enveloped chain": (1100, 2500, 4700),
    "enveloped bearing": (2200, 4000, 6500),
    "enveloped pulley": (650, 1450, 2700),
}
KG = {
    "NEO": (.43, .4536, .49),
    "MAXPlanetary": (.20, .32, .50),
    "MAXSwerve with motors": (1.65, 1.720, 1.85),
}
CASES = ("light", "nominal", "heavy")


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def classify(name):
    if "bumper wood" in name:
        return "plywood"
    if "noodle" in name:
        return "foam"
    if " neo " in name:
        return "NEO"
    if "maxplanetary" in name:
        return "MAXPlanetary"
    if "slide pad" in name:
        return "UHMW"
    if "chain run" in name or "chain straight-run" in name:
        return "enveloped chain"
    if "bearing representation" in name or "bearing envelope" in name:
        return "enveloped bearing"
    if any(token in name for token in ("pulley", "sprocket")):
        return "enveloped pulley"
    if any(token in name for token in ("belt", "tread", "insulating-pad")):
        return "elastomer"
    if re.search(r"bolt|locknut|screw|barrel nut|shaft|stub axle|collar envelope|[- ](head|nut)$", name):
        return "steel"
    return "aluminum"


def budget(survey, manifest, *, independent_bounds=False):
    assembly = json.loads(survey.read_bytes())
    display = json.loads(manifest.read_bytes())
    identity = json.loads((ROOT / "docs/detailed-design/archive/draft-001/media/swerve-r7.json").read_bytes())
    if assembly["archive"] != display["sourceSha256"] or assembly["archive"] != identity["sourceSha256"]:
        raise ValueError("The material survey and display must both belong to R7")
    electrical = {r["id"]: r["massKg"] for r in json.loads(
        (ROOT / "cad/electrical/manifest.json").read_bytes())["components"] if r["massKg"] is not None}
    group_by_key = {k: mesh["name"].split("-")[0] for mesh in display["meshes"] for k in mesh["sourceBodyKeys"]}
    inventory = defaultdict(list)
    for row in display["bodyInventory"]:
        key = row["parent"], row["component"], row["name"]
        inventory[key].append(group_by_key[row["key"]])

    parcels = []
    counts = Counter()

    def add(group, role, masses, center, radius2, size):
        # zz is the native moment about the robot origin divided by native mass.
        # Estimate the other two central moments from installed dimensions.
        x, y, z = size
        yaw = radius2-center[:2]@center[:2]
        # Split the measured horizontal variance between axes by footprint aspect.
        fraction_x = x*x/(x*x+y*y)
        central = np.array([yaw*(1-fraction_x)+z*z/12, yaw*fraction_x+z*z/12, yaw])
        if min(central) < -1e-7 or not np.isfinite([*center, *central, *masses]).all():
            raise ValueError("Invalid native center or moment")
        parcels.append((group, role, np.asarray(masses), center, np.maximum(central, 0)))
        counts[role] += 1

    for row in assembly["rows"]:
        parent = row["path"].split("+")[0].rsplit(":", 1)[0]
        if parent.startswith("MAXSwerve "):
            continue  # One purchased module at each registered corner below.
        name = row["name"].lower()
        key = parent, row["name"], row["body"]
        if not row["visible"] and ("noodle" in name or "bumper wood" in name):
            group = "fixed"
        elif row["visible"] and inventory.get(key):
            group = inventory[key].pop()
        elif parent.startswith(("frame fastening bridge", "baseplate-", "stack-post-", "radio-power-deck-tab", "breaker-")):
            group = "fixed"  # Fixed mounting hardware omitted from the public mesh selection.
        elif not row["visible"] or "withdrawal reserve" in name:
            continue
        else:
            raise ValueError(f"Unmatched installed solid {key}")
        if row["body"] == "Continuous covered bumper envelope":
            continue  # Covering is a separate mass allowance. Wood and foam are budgeted individually.
        role = classify(name)
        if row["name"] in electrical:
            role = "catalog electronics"
            masses = np.array([.95, 1., 1.05])*electrical[row["name"]]
        elif role in KG:
            masses = KG[role]
        else:
            masses = np.array(DENSITY[role])*row["volumeMm3"]*1e-9
        box = np.array(row["bboxMm"]).reshape(2, 3)/1000
        add(group, role, masses, np.array(row["centerMm"])/1000, row["polarRadiusSquaredM2"], box[1]-box[0])

    remaining = {key: parts for key, parts in inventory.items() if parts and "visual only" not in key[0]}
    if remaining or counts["foam"] != 8 or counts["plywood"] != 4 or counts["NEO"] != 8:
        raise ValueError(f"Incomplete R7 inventory {remaining}")
    layout = json.loads((ROOT / "cad/drivetrain/maxswerve-layout.json").read_bytes())
    for station in layout["stations"].values():
        box = np.array(station["installedBoundsMm"]).reshape(2, 3)/1000
        center = box.mean(axis=0)
        size = box[1]-box[0]
        add("fixed", "MAXSwerve with motors", KG["MAXSwerve with motors"], center,
            center[:2]@center[:2]+size[:2]@size[:2]/12, size)
    # Distributed allowances, with explicit reference poses and radii of gyration.
    add("fixed", "bumper covers", (.25, .50, .80), np.array([.01, 0., .12]), .22, (.84, .84, .14))
    add("fixed", "wire, terminals and retention", (.6, 1.2, 2.2), np.array([-.10, 0., .25]), .09, (.6, .6, .3))
    add("fixed", "welds and installation hardware", (.15, .45, 1.), np.array([0., 0., .17]), .09, (.66, .66, .28))

    def summarize(selected, index):
        masses = np.array([p[2][index] for p in selected])
        centers = np.array([p[3] for p in selected])
        center = np.average(centers, axis=0, weights=masses)
        delta2 = (centers-center)**2
        moments = (masses[:, None]*(np.array([p[4] for p in selected])+delta2.sum(axis=1)[:, None]-delta2)).sum(axis=0)
        delta = centers-center
        cross = -(delta.T*masses)@delta
        return dict(massKg=float(masses.sum()), centerM=center.tolist(), diagonalAtCenterKgM2=moments.tolist(),
                    productsAtCenterKgM2=[float(cross[i, j]) for i,j in ((0,1), (0,2), (1,2))])

    if independent_bounds:
        # For a trial CG coordinate t, sum m*(x-t) is linear in each mass.
        # Its sign at the extremizing corner brackets the fractional optimum.
        centers = np.array([p[3] for p in parcels])
        low, high = np.array([p[2] for p in parcels])[:, [0, 2]].T
        bounds = []
        for axis in centers.T:
            limits = []
            for maximize in (False, True):
                lo, hi = min(axis), max(axis)
                for _ in range(60):
                    trial = (lo+hi)/2
                    mass = np.where((axis > trial)==maximize, high, low)
                    if mass@(axis-trial) > 0:
                        lo = trial
                    else:
                        hi = trial
                limits.append(float((lo+hi)/2))
            bounds.append(limits)
        return dict(schema="r7-independent-mass-bounds/1", archive=assembly["archive"], surveySha256=sha(survey),
                    centerIntervalsM=bounds, massIntervalKg=[float(low.sum()), float(high.sum())],
                    inertiaDiagonalAtCenterIntervalKgM2=np.stack([
                        summarize(parcels, i)["diagonalAtCenterKgM2"] for i in (0,2)], axis=1).tolist())
    return dict(schema="r7-floor-mass-budget/1", archive=assembly["archive"], surveySha256=sha(survey),
                inventorySha256=sha(manifest), densityKgM3=DENSITY, componentKg=KG,
                roles={role: dict(count=counts[role], massKg=sum(
                    (p[2] for p in parcels if p[1]==role), np.zeros(3)).tolist()) for role in sorted(counts)},
                cases={case: {group: summarize([p for p in parcels if group=="robot" or p[0]==group], i)
                              for group in ("robot", "fixed", "carriage", "cradle")} for i, case in enumerate(CASES)})


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("survey", "manifest", "output"):
        parser.add_argument(name, type=Path)
    parser.add_argument("--independent-bounds", action="store_true")
    args = parser.parse_args()
    data = budget(args.survey, args.manifest, independent_bounds=args.independent_bounds)
    args.output.write_text(json.dumps(data, indent=2, allow_nan=False)+"\n", encoding="utf8", newline="\r\n")
