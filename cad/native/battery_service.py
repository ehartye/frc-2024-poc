"""Passive ES17-12 restraint on the released deck and tray interfaces."""

import adsk.core as core
import adsk.fusion as fusion
from stage_climber import box, cut, pt, rod
from physical_bumper import install_part

BATTERY_GROUP = "Battery restraint rework"
BASE_Z = 208.115
STRAP_Z = 375.993 + 2.125


def cylinder(x, y, z, h, diam):
    return rod((x, y, z), (x, y, z + h), diam)


def aligned_face(comp, offset):
    planes = comp.constructionPlanes
    req = planes.createInput()
    req.setByOffset(comp.xYConstructionPlane, core.ValueInput.createByReal(offset / 10))
    return planes.add(req)


def stop_profiles():
    # End stops and four side lands avoid the original tray bolts and strap feet.
    return [(-296, -258, -293, -222), (-107, -258, -104, -222)] + [
        (x - 12, lo, x + 12, hi)
        for x in (-255, -145)
        for lo, hi in ((-283.5, -280.5), (-199.5, -196.5))
    ]


def install(d, chassis, lib):
    from physical_mount import hexagonal

    comp = chassis.component
    tray = comp.bRepBodies.itemByName("R7 166 battery-tray")
    if not tray:
        raise ValueError("Expected original tray")
    sketch = comp.sketches.add(aligned_face(comp, BASE_Z))
    sketch.name = "Six integral battery stop locations"
    for x, y, X, Y in stop_profiles():
        sketch.sketchCurves.sketchLines.addTwoPointRectangle(pt(x, y, 0), pt(X, Y, 0))
    profiles = core.ObjectCollection.create()
    for profile in sketch.profiles:
        profiles.add(profile)
    if profiles.count != 6:
        raise ValueError("Incomplete stop profiles")
    maker = comp.features.extrudeFeatures
    req = maker.createInput(profiles, fusion.FeatureOperations.JoinFeatureOperation)
    req.setDistanceExtent(False, core.ValueInput.createByString("15 mm"))
    req.participantBodies = [tray]
    maker.add(req).name = "Integral stops on machined replacement tray"
    # The earlier base plate body carries the exact original mounting holes.
    # Replacing its manufacturing stock with one billet preserves that interface.
    old = comp.bRepBodies.itemByName("R7 203 battery-top-strap")
    targets = core.ObjectCollection.create()
    targets.add(old)
    move = comp.features.moveFeatures
    request = move.createInput2(targets)
    v = core.ValueInput.createByReal
    request.defineAsTranslateXYZ(v(0), v(0), v((STRAP_Z - 375.993) / 10), False)
    move.add(request).name = "Upper strap clearance for largest specified battery"
    for i in [204, 209, 210, 211, 216, 217, 218]:
        key = next(b for b in comp.bRepBodies if b.name.startswith(f"R7 {i} "))
        comp.features.removeFeatures.add(key).name = f"Replace battery pad or fastening article {i}"
    mgr = fusion.TemporaryBRepManager.get()
    steel = lib.itemByName("Steel, Alloy")
    al = lib.itemByName("Aluminum 6061")
    color = d.appearances.itemByName("R7 structure")
    rubber = d.materials.addByCopy(
        lib.itemByName("Rubber, Nitrile"), "Battery compression pad - lot and mass pending"
    )

    def join(a, b):
        if not mgr.booleanOperation(a, b, fusion.BooleanTypes.UnionBooleanType):
            raise ValueError("Battery screw union")
        return a

    def rectangle_ring(x, y, z, h, od, inside):
        return cut(cylinder(x, y, z, h, od), cylinder(x, y, z - 0.1, h + 0.2, inside))

    items = []
    # The single bonded dielectric pad stays on the bar. The two shim packs
    # sit at the fastener stations outside the case, retained by the screws.
    items.append(
        (
            "Insulating top pad installed at 3.125 mm",
            box(-215, -277, 374.993, -185, -203, STRAP_Z),
            rubber,
            color,
        )
    )
    for y in (-298, -182):
        items.append(
            (
                f"Upper strap spacer 2 mm {y}",
                rectangle_ring(-200, y, 375.993, 2.0, 12, 5.5),
                al,
                color,
            )
        )
        items.append(
            (
                f"Upper strap shim 0.125 mm {y}",
                rectangle_ring(-200, y, 377.993, 0.125, 12, 5.5),
                steel,
                color,
            )
        )
        seat = STRAP_Z + 3
        screw = join(
            cylinder(-200, y, seat - 19.05, 19.05, 4.826), cylinder(-200, y, seat, 4.826, 7.9248)
        )
        cut(screw, hexagonal(-200, y, seat + 2.4, seat + 5.0, 3.96875))
        items.append((f"Aspen SO007-1032X34 strap screw {y}", screw, steel, color))
        nut = cut(hexagonal(-200, y, 366.643, 372.993, 9.525), cylinder(-200, y, 366.0, 7.3, 4.1))
        items.append((f"Airfasco MS21044N3 strap locknut envelope {y}", nut, steel, color))
    install_part(d, chassis, BATTERY_GROUP, items)
