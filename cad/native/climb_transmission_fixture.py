"""Unpowered screw, carrier and two-way thrust support for the isolated mast study."""

import math
from adsk import core, fusion
from stage_climber import box, cut, pt, rod
from physical_bumper import install_part
from physical_mount import hexagonal

SCREW_X, SCREW_Y = 38.0, -39.0
FIXED = "Bench screw and thrust cartridge"
MOVING = "Bench bronze nut and carrier"


def tube(z0, z1, outer, inner):
    x, y = SCREW_X, SCREW_Y
    return cut(rod((x, y, z0), (x, y, z1), outer), rod((x, y, z0 - 0.1), (x, y, z1 + 0.1), inner))


def merge(a, b):
    if not fusion.TemporaryBRepManager.get().booleanOperation(
        a, b, fusion.BooleanTypes.UnionBooleanType
    ):
        raise ValueError("Climber fixture join")
    return a


def socket(screw, x, y, z, across_flats, depth):
    tool = hexagonal(0, 0, -0.1, depth, across_flats)
    tx = core.Matrix3D.create()
    tx.setToRotation(math.pi / 2, core.Vector3D.create(0, 1, 0), pt(0, 0, 0))
    tx.translation = core.Vector3D.create(x / 10, y / 10, z / 10)
    if not fusion.TemporaryBRepManager.get().transform(tool, tx):
        raise ValueError("Place transverse hex recess")
    return cut(screw, tool)


def countersink(comp, target, y, z):
    mgr = fusion.TemporaryBRepManager.get()
    # A 90-degree mouth opens into the 4.3 mm host clearance bore.
    tool = mgr.createCylinderOrCone(pt(27.99, y, z), 0.451, pt(30.35, y, z), 0.215)
    base = comp.features.baseFeatures.add()
    base.name = "Carrier fixing countersink tool"
    base.startEdit()
    try:
        comp.bRepBodies.add(tool, base)
    finally:
        base.finishEdit()
    tools = core.ObjectCollection.create()
    tools.add(base.bodies.item(0))
    combine = comp.features.combineFeatures
    req = combine.createInput(target, tools)
    req.operation = fusion.FeatureOperations.CutFeatureOperation
    combine.add(req).name = "Countersink carrier fixing to the guide face"


def sketch_cut(comp, body, plane, profile_points, radius, travel, title):
    sk = comp.sketches.add(plane)
    sk.name = title + " profile"
    for point in profile_points:
        sk.sketchCurves.sketchCircles.addByCenterRadius(
            sk.modelToSketchSpace(pt(*point)), radius / 10
        )
    profiles = core.ObjectCollection.create()
    for profile in sk.profiles:
        profiles.add(profile)
    req = comp.features.extrudeFeatures.createInput(
        profiles, fusion.FeatureOperations.CutFeatureOperation
    )
    req.setDistanceExtent(False, core.ValueInput.createByString(f"{travel} mm"))
    req.participantBodies = [body]
    comp.features.extrudeFeatures.add(req).name = title


def install(d, chassis, lib):
    root = d.rootComponent
    fixed = next(o for o in root.allOccurrences if o.component.name == "R8 fixed climb mast")
    slide = next(o for o in root.allOccurrences if o.component.name == "R8 sliding saddle")
    for occ, names in [
        (
            fixed,
            [
                "Thrust bearing support envelope",
                "Screw envelope",
                "Drive gearbox envelope",
                "Drive motor envelope",
            ],
        ),
        (slide, ["Nut carrier plate", "Travelling nut envelope"]),
    ]:
        for n in names:
            body = occ.component.bRepBodies.itemByName(n)
            if not body:
                raise ValueError("Unexpected inherited climber " + n)
            occ.component.features.removeFeatures.add(body).name = "Supersede " + n
    fcomp = fixed.component
    beam = fcomp.bRepBodies.itemByName("Crossmember with four rail mounting holes")
    sketch_cut(
        fcomp,
        beam,
        fcomp.xYConstructionPlane,
        [(SCREW_X, SCREW_Y, 0)],
        9.05,
        159.1,
        "Cartridge barrel bore through beam walls",
    )
    scomp = slide.component
    inner = scomp.bRepBodies.itemByName("Inner sliding tube with lock bore")
    sk = scomp.sketches.add(scomp.xYConstructionPlane)
    sk.sketchCurves.sketchLines.addTwoPointRectangle(pt(27, -53, 0), pt(49, -7, 0))
    req = scomp.features.extrudeFeatures.createInput(
        sk.profiles.item(0), fusion.FeatureOperations.CutFeatureOperation
    )
    req.setDistanceExtent(False, core.ValueInput.createByString("197 mm"))
    req.participantBodies = [inner]
    scomp.features.extrudeFeatures.add(req).name = "Open lower guide for shouldered carrier"
    # Two flush screws pass through the near tube wall into the tapped tenon.
    index = [(-17.0, 204.0), (-26.0, 212.0)]
    sketch_cut(
        scomp,
        inner,
        scomp.yZConstructionPlane,
        [(0, y, z) for y, z in index],
        2.15,
        30.55,
        "Two carrier retaining screw bores",
    )
    metal = lib.itemByName("Steel, Alloy")
    al = lib.itemByName("Aluminum 6061")
    inox = lib.itemByName("Stainless Steel")
    bronze = lib.itemByName("Bronze") or d.materials.addByCopy(
        lib.itemByName("Copper"), "SAE 660 bronze envelope - receiving mass pending"
    )
    color = d.appearances.itemByName("R7 structure")
    stationary = []
    moving = []

    def add(to, n, s, mat):
        to.append((n, s, mat, color))

    def spindle(z0, z1, diam):
        return rod((SCREW_X, SCREW_Y, z0), (SCREW_X, SCREW_Y, z1), diam)

    shaft = merge(spindle(106.5, 162.0, 6.0), spindle(162.0, 595.0, 9.525))
    add(stationary, "Thomson SRA3708S machined screw envelope", shaft, inox)
    # F6-14M housing abutment bore at most 8.0; shaft washers bear on diameter 12.
    housing = merge(tube(120.65, 164.0, 18.0, 6.8), tube(117.0, 120.65, 24.0, 6.8))
    cut(housing, spindle(116.9, 122.0, 14.05), spindle(155.0, 164.1, 14.05))
    # Insert the radial bushing before either thrust assembly.
    cut(housing, spindle(121.9, 140.0, 8.0))
    add(stationary, "Machined flanged M18 x 1 thrust housing", housing, metal)
    polymer = lib.itemByName("Nylon 6/6") or lib.itemByName("ABS Plastic")
    add(
        stationary,
        "igus GSM-0608-08 installed sleeve envelope",
        tube(132.0, 140.0, 8.0, 6.02),
        polymer,
    )
    add(
        stationary,
        "Cartridge round slotted lockring M18 x 1",
        cut(
            tube(159.0, 164.0, 21.5, 17.0),
            box(37.25, -51, 161, 38.75, -46.5, 165),
            box(37.25, -31.5, 161, 38.75, -27, 165),
        ),
        metal,
    )
    for z in (117.0, 155.0):
        add(stationary, f"EZO F6-14M thrust bearing envelope {z}", tube(z, z + 5, 14.0, 6.0), metal)
    add(stationary, "Upper hardened shaft abutment", tube(160.0, 162.0, 12.0, 6.05), metal)
    add(stationary, "Lower hardened shaft abutment", tube(115.0, 117.0, 12.0, 6.05), metal)
    nut = cut(hexagonal(38, -39, 109.0, 115.0, 10.0), spindle(108.9, 116, 5.1))
    add(stationary, "Accu HNN-M6-1-C8-Z retaining nut envelope", nut, metal)
    # Lower carrier occupies the cavity below the shortened moving tube.
    # This leaves the original full-width guide intact around the nut flange.
    carriage = merge(
        box(28.05, -51.9, 180.35, 47.95, -8.1, 197.0), box(30.6, -49.4, 197.0, 45.4, -10.6, 218.0)
    )
    cut(
        carriage,
        spindle(180.0, 193.1, 14.45),
        spindle(193.05, 218.1, 10.4),
        box(37.65, -39, 180.0, 38.35, -8.0, 196.9),
    )

    def across(x0, x1, y, z, diam):
        return rod((x0, y, z), (x1, y, z), diam)

    cut(
        carriage,
        across(27.9, 32.05, -24, 187.0, 7.3),
        across(32.0, 38.5, -24, 187.0, 4.5),
        across(38.5, 48, -24, 187.0, 3.3),
    )
    for y, z in index:
        cut(carriage, across(30.4, 31.4, y, z, 4.3), across(31.4, 45.6, y, z, 3.3))
    add(moving, "Split tapped nut carrier with guide tenon", carriage, al)
    nut = merge(tube(174.0, 180.35, 19.05, 9.6), tube(180.35, 193.05, 15.875, 9.6))
    add(moving, "Thomson BN3708S SAE 660 nut envelope", nut, bronze)
    # A flush head rotates clear of the stationary guide. The cylindrical
    # shank represents major thread diameter only in its threaded partner.
    screw = merge(across(28.05, 32.05, -24, 187.0, 7), across(32.05, 46.05, -24, 187.0, 4))
    socket(screw, 28.05, -24, 187.0, 3.0, 2.0)
    add(moving, "Accu SSCF-M4-14-A4 clamp screw envelope", screw, inox)
    for y, z in index:
        countersink(scomp, inner, y, z)
        mgr = fusion.TemporaryBRepManager.get()
        head = mgr.createCylinderOrCone(pt(28, y, z), 0.448, pt(30.48, y, z), 0.2)
        screw = merge(head, across(30.48, 44.0, y, z, 4.0))
        socket(screw, 28.0, y, z, 2.5, 1.4)
        add(moving, f"Accu SSK-M4-16-A2 carrier screw envelope {y}", screw, inox)
    install_part(d, fixed, FIXED, stationary)
    install_part(d, slide, MOVING, moving)
