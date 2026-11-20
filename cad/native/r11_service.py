"""Extend the isolated R10 study with unpowered receiving and mast service tooling."""

import sys
from pathlib import Path
from adsk import core, fusion
from physical_bumper import LIB, design, native, sha
from physical_mount import hexagonal
from stage_climber import box, cut, pt, rod
from passive_service_fit import common, dimensions, interference, moved, overlap, registered_solid

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "service-readiness/bench"))
from layout import (
    WIDTH,
    DEPTH,
    THICKNESS,
    CLINCH_HOLES,
    BENCH_HOLES,
    BATTERY_BAYS,
    CASE_SWEEP,
    CROSSOVER,
)

PARENT = "06d0f92aa7c3ccd4f23f489b8d8041805e07cf6ac70a3272aeeba3397e890a21"
PLATE = "R11 isolated controller receiving plate"
GAGE = "R11 disconnected battery dressing gauges"
KEY = "R11 unloaded mast parking spacer"


def union(a, b):
    if not fusion.TemporaryBRepManager.get().booleanOperation(
        a, b, fusion.BooleanTypes.UnionBooleanType
    ):
        raise ValueError("Unable to join service tool")
    return a


def zc(x, y, z0, z1, diameter):
    return rod((x, y, z0), (x, y, z1), diameter)


def base_parts(d, name, items, offset=(0, 0, 0)):
    mat = core.Matrix3D.create()
    mat.translation = core.Vector3D.create(*(v / 10 for v in offset))
    occ = d.rootComponent.occurrences.addNewComponent(mat)
    comp = occ.component
    comp.name = name
    base = comp.features.baseFeatures.add()
    base.name = name + " geometry"
    base.startEdit()
    try:
        for title, shape, material, color in items:
            comp.bRepBodies.add(shape, base).name = title
    finally:
        base.finishEdit()
    if len(items) != comp.bRepBodies.count:
        raise ValueError(name + " inventory")
    for body, (title, _, material, color) in zip(comp.bRepBodies, items):
        body.name = title
        body.material = material
        body.appearance = color
    return occ


def drill(comp, body, points, radius, label):
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = label + " centers"
    for x, y in points:
        curve = sk.sketchCurves.sketchCircles.addByCenterRadius(pt(x, y, 0), radius / 10)
        curve.isFixed = True
    coll = core.ObjectCollection.create()
    for p in sk.profiles:
        coll.add(p)
    if coll.count != len(points):
        raise ValueError(label + " profile count")
    ex = comp.features.extrudeFeatures
    req = ex.createInput(coll, fusion.FeatureOperations.CutFeatureOperation)
    req.setOneSideExtent(
        fusion.DistanceExtentDefinition.create(core.ValueInput.createByString("2 mm")),
        fusion.ExtentDirections.NegativeExtentDirection,
    )
    req.participantBodies = [body]
    ex.add(req).name = label


def build(source, archive):
    if sha(source) != PARENT:
        raise ValueError("Expected released R10 fixture")
    doc = native(source)
    try:
        d = design(doc)
        lib = core.Application.get().materialLibraries.itemByName(LIB).materials
        al = lib.itemByName("Aluminum 6061")
        ss = lib.itemByName("Stainless Steel")
        polymer = lib.itemByName("ABS Plastic") or lib.itemByName("Nylon 6/6")
        steelcolor = d.appearances.itemByName("R7 structure")
        refcolor = d.appearances.itemByName("R7 electrical") or steelcolor
        gripcolor = d.appearances.itemByName("R7 mechanism") or steelcolor
        items = [
            (
                "R11 plain receiving plate",
                box(-WIDTH / 2, -DEPTH / 2, -THICKNESS, WIDTH / 2, DEPTH / 2, 0),
                al,
                steelcolor,
            )
        ]
        for i, (x, y) in enumerate(CLINCH_HOLES, 1):
            shank = union(zc(x, y, -4, -2, 7.87), zc(x, y, -2, -0.62, 5.38))
            cut(shank, zc(x, y, -4.1, -0.5, 3.3))
            bolt = union(zc(x, y, -5.5, 2.5, 4), zc(x, y, 2.5, 6.5, 7))
            cut(bolt, hexagonal(x, y, 4.5, 6.6, 3))
            items.extend(
                [
                    (f"PEM CLS-M4-2 bench nut {i}", shank, ss, steelcolor),
                    (f"Accu SSCF-M4-8-A2 bench screw {i}", bolt, ss, steelcolor),
                ]
            )
        # Removable gauge pads represent the flange height at each fastener.
        # Exact purchased bracket/controller geometry stays in the private review.
        for i, (x, y) in enumerate(CLINCH_HOLES, 1):
            washer = cut(zc(x, y, 0, 2.5, 9.7), zc(x, y, -0.1, 2.6, 5.0))
            items.append((f"Flange-height gauge pad {i}", washer, polymer, refcolor))
        # The bench assembly sits outside the vehicle coordinate envelope.
        bench = base_parts(d, PLATE, items, (640, 0, 120))
        plate = bench.component.bRepBodies.itemByName("R11 plain receiving plate")
        drill(bench.component, plate, CLINCH_HOLES, 2.705, "Twelve captive nut holes")
        drill(bench.component, plate, BENCH_HOLES, 3.3, "Four fixture bench attachment holes")
        items = [
            (f"Dressing maximum {i}", box(*v), polymer, refcolor)
            for i, v in enumerate(BATTERY_BAYS, 1)
        ]
        items.append(("Dressing crossover maximum", box(*CROSSOVER), polymer, refcolor))
        base_parts(d, GAGE, items)
        # An open-bottom receiving gauge. Each blind pocket checks the selected
        # envelope including height. This tool stays off the battery in service.
        shell = box(-294, -282, 377.115, -106, -198, 437.115)
        for x, y, z, X, Y, Z in BATTERY_BAYS:
            cut(shell, box(x, y, z - 0.2, X, Y, Z))
        # Open the crossover for side-by-side preassembled leads. The loose
        # sill recreates the raised floor once the disconnected mockup is in place.
        x, y, z, X, Y, Z = CROSSOVER
        cut(shell, box(x - 0.1, y, 376.915, X + 0.1, Y, Z))
        sill = box(x + 0.15, y + 0.15, 377.115, X - 0.15, Y - 0.15, z)
        base_parts(
            d,
            "R11 twin battery pocket receiving gauge",
            [
                ("Detached twin pocket gauge", shell, polymer, refcolor),
                ("Removable crossover height sill", sill, polymer, gripcolor),
                (
                    "Bare SB50 housing receiving block 49 x 38 x 17",
                    box(-279.5, -259, 377.115, -230.5, -221, 394.115),
                    polymer,
                    refcolor,
                ),
            ],
            (840, 30, -257.115),
        )
        key = union(box(22.5, -59.3, 600, 27.5, -10, 619.6), box(16, -72, 602, 27.5, -59.3, 615))
        union(key, box(23.6, -59.3, 595.5, 27.4, -56.3, 600))
        base_parts(d, KEY, [("Mast spacer with grasp tab and seating stop", key, al, gripcolor)])
        if not d.exportManager.execute(
            d.exportManager.createFusionArchiveExportOptions(str(archive))
        ):
            raise RuntimeError("R11 export failed")
    finally:
        doc.close(False)


def inspect(d, whole):
    parts = {
        b.name: registered_solid(o, b) for o in d.rootComponent.allOccurrences for b in o.bRepBodies
    }
    keyname = "Mast spacer with grasp tab and seating stop"
    fixed = "Outer guide with double shear lock bore"
    top = "Top bracket"
    key = parts[keyname]
    # The insertion translation has a constant cross-section for each of the
    # three rectangular portions. Their swept union includes intermediate poses.
    sweep = union(box(22.5, -149.3, 600, 27.5, -10, 619.6), box(16, -162, 602, 27.5, -59.3, 615))
    union(sweep, box(23.6, -149.3, 595.5, 27.4, -56.3, 600))
    limits = dimensions(key)
    gauges = {"mast insertion": sweep}
    for i, (x, y, z, X, Y, Z) in enumerate(BATTERY_BAYS, 1):
        # Cover the full published case-height tolerance with the union of
        # the packed 58 mm-high bays. Its bottom can fall inside a taller case.
        gauges[f"dress {i} park"] = box(x, y, z - 3, X, Y, Z)
        gauges[f"dress {i} removal"] = box(x, y, z - 3, X, Y, CASE_SWEEP[-1])
    x, y, z, X, Y, Z = CROSSOVER
    gauges["dress crossover park"] = box(x, y, z - 3, X, Y, Z)
    gauges["dress crossover removal"] = box(x, y, z - 3, X, Y, CASE_SWEEP[-1])
    # Compare the manufactured bench plate against the existing, occupied deck
    # at its center in both orthogonal orientations. These are trial placements.
    plate = parts["R11 plain receiving plate"]
    long_x = moved(plate, -800, 0, 87.115)
    matrix = core.Matrix3D.create()
    import math

    matrix.setToRotation(math.pi / 2, core.Vector3D.create(0, 0, 1), pt(-160, 0, 205.115))
    mgr = fusion.TemporaryBRepManager.get()
    long_y = mgr.copy(long_x)
    if not mgr.transform(long_y, matrix):
        raise ValueError("Bench plate rotation")
    trials = {"long X": long_x, "long Y": long_y}
    trial_hits = {k: {} for k in trials}
    trial_bounds = {k: dimensions(v) for k, v in trials.items()}
    gauge_bounds = {k: dimensions(v) for k, v in gauges.items()}
    hits = {name: {} for name in gauges}
    tested = {name: [] for name in gauges}
    # Keep the parked fit and removal phases distinct. Compare the registered
    # R10 replacement strap as well as the inherited R9 strap in parked coordinates.
    for occ in whole.rootComponent.allOccurrences:
        for b in occ.bRepBodies:
            bounds = dimensions(b)
            possible = [n for n, limits in gauge_bounds.items() if overlap(limits, bounds)]
            plate_trials = [n for n, limits in trial_bounds.items() if overlap(limits, bounds)]
            if not possible and not plate_trials:
                continue
            article = registered_solid(occ, b)
            for n in plate_trials:
                volume = common(trials[n], article)
                if volume > 0.01:
                    trial_hits[n][b.name] = volume
            for n in possible:
                if n.startswith("dress") and b.name == "R7 144 battery":
                    continue
                tested[n].append(b.name)
                v = common(gauges[n], article)
                if v > 0.01:
                    hits[n][b.name] = v
    for n, g in gauges.items():
        exclude = (
            [keyname]
            if n == "mast insertion"
            else [
                "R7 144 battery",
                "Dressing crossover maximum",
                *[f"Dressing maximum {i}" for i in (1, 2)],
            ]
        )
        for who, v in interference(g, parts, exclude).items():
            hits[n]["fixture " + who] = v
    datum = parts["R7 203 battery-top-strap"]
    controls = [
        common(moved(gauges["dress 1 park"], x=8), datum),
        common(moved(gauges["dress 2 park"], x=-8), datum),
        common(moved(sweep, x=-1), parts["Saddle spine"]),
    ]
    lower = common(moved(key, z=-0.02), parts[fixed]) / 0.02
    upper = common(moved(key, z=0.42), parts[top]) / 0.02
    plate = parts["R11 plain receiving plate"]
    nut = parts["PEM CLS-M4-2 bench nut 1"]
    shell = parts["Detached twin pocket gauge"]
    sill = parts["Removable crossover height sill"]
    tool_offset = (840, 30, -257.115)
    go = {}
    for name in ("Dressing maximum 1", "Dressing maximum 2", "Dressing crossover maximum"):
        probe = moved(parts[name], *tool_offset)
        go[name] = round(common(probe, shell) + common(probe, sill), 4)
    bare = parts["Bare SB50 housing receiving block 49 x 38 x 17"]
    go["bare housing against pocket"] = common(bare, shell) + common(bare, sill)
    high = moved(parts["Dressing maximum 1"], 840, 30, -256.115)
    sag = moved(parts["Dressing crossover maximum"], 840, 30, -258.115)
    hole_fit = [common(zc(640 + x, y, 117, 121, 5.40), plate) for x, y in CLINCH_HOLES]
    hx, hy = CLINCH_HOLES[0]
    bad_hole = common(zc(640 + hx + 0.5, hy, 117, 121, 5.40), plate)
    result = dict(
        bodyCount=len(parts),
        wholeRobotCenteredPlateTrialOverlapMm3=trial_hits,
        receivingGaugeParkOverlapMm3=go,
        highDressingControlMm3=common(high, shell),
        lowCrossoverControlMm3=common(sag, sill),
        clinchEntryGaugeMm3=hole_fit,
        clinchOffsetControlMm3=bad_hole,
        parkAndRemovalHits=hits,
        r9BroadPhaseCandidates=tested,
        deliberateOverlapMm3=controls,
        spacerBoundsMm=limits,
        spacerStockVolumeMm3=round(key.volume * 1000, 4),
        spacerLowerProjectedSeatMm2=round(lower, 3),
        spacerUpperProjectedSeatMm2=round(upper, 3),
        nativePlateStockVolumeMm3=round(plate.volume * 1000, 3),
        nativePlateBoundsMm=dimensions(plate),
        nativeBenchNutBoundsMm=dimensions(nut),
        clearanceToTopBracketMm=round(dimensions(parts[top])[2] - limits[5], 3),
    )
    return result


def verify(r):
    if any(len(hits) < 1 for hits in r["wholeRobotCenteredPlateTrialOverlapMm3"].values()):
        raise ValueError("Centered stock no longer reproduces the deck obstruction")
    if any(r["receivingGaugeParkOverlapMm3"].values()) or any(r["clinchEntryGaugeMm3"]):
        raise ValueError("Receiving tool or captive nut hole obstructed")
    if (
        min(r["highDressingControlMm3"], r["lowCrossoverControlMm3"], r["clinchOffsetControlMm3"])
        <= 0.01
    ):
        raise ValueError("Invalid receiving tool negative control")
    if any(r["parkAndRemovalHits"].values()) or min(r["deliberateOverlapMm3"]) <= 0.01:
        raise ValueError("Unexpected fit result")
    if min(r["spacerLowerProjectedSeatMm2"], r["spacerUpperProjectedSeatMm2"]) < 100:
        raise ValueError("Missing parking support bearing face")
    if (
        abs(r["nativePlateStockVolumeMm3"] - 108614.614) > 0.02
        or r["clearanceToTopBracketMm"] != 0.4
    ):
        raise ValueError("R11 plate or spacer geometry mismatch")
