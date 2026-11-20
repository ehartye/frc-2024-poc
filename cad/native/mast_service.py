"""Independent, removable positive lock for supported, unloaded mast service."""

from adsk import core, fusion
from physical_bumper import install_part
from stage_climber import pt, rod, cut

AXIS_Y = -19.0
PIN_Z = 305.0
LOCK_GROUP = "Unloaded mast service lock"


def install(d, chassis, lib):
    root = d.rootComponent
    rails = [
        ("R8 fixed climb mast", "Outer guide with double shear lock bore"),
        ("R8 sliding saddle", "Inner sliding tube with lock bore"),
    ]
    for name, article in rails:
        comp = next(o.component for o in root.allOccurrences if o.component.name == name)
        part = comp.bRepBodies.itemByName(article)
        sketch = comp.sketches.add(comp.yZConstructionPlane)
        sketch.name = "Unloaded service position at z305"
        center = sketch.modelToSketchSpace(pt(0, AXIS_Y, PIN_Z))
        sketch.sketchCurves.sketchCircles.addByCenterRadius(center, 0.5)
        req = comp.features.extrudeFeatures.createInput(
            sketch.profiles.item(0), fusion.FeatureOperations.CutFeatureOperation
        )
        req.setDistanceExtent(False, core.ValueInput.createByString("65 mm"))
        req.participantBodies = [part]
        comp.features.extrudeFeatures.add(req).name = "Ream aligned service pin bore to 10 H11"
    mgr = fusion.TemporaryBRepManager.get()

    def axial(x0, x1, d):
        return rod((x0, AXIS_Y, PIN_Z), (x1, AXIS_Y, PIN_Z), d)

    def join(a, b):
        if not mgr.booleanOperation(a, b, fusion.BooleanTypes.UnionBooleanType):
            raise ValueError("Positive service pin profile")
        return a

    color = d.appearances.itemByName("R7 structure")
    metal = lib.itemByName("Stainless Steel")
    collar = cut(axial(52, 53.5, 18), axial(51.9, 53.6, 10.0))
    head = join(axial(13.9, 53.5, 9.96), axial(53.5, 80.5, 14))
    # External supplier-sized envelope of engaged detents. Internal detent
    # and release mechanism remain those of the purchased pin.
    for sign in (-1, 1):
        join(head, mgr.createSphere(pt(21.9, AXIS_Y + sign * 4.6, PIN_Z), 0.14))
    fixed = next(o for o in root.allOccurrences if o.component.name == rails[0][0])
    install_part(
        d,
        fixed,
        "Fixed guide service collar",
        [("Flush collar on service side", collar, lib.itemByName("Aluminum 6061"), color)],
    )
    install_part(
        d,
        fixed,
        LOCK_GROUP,
        [("Ganter GN 113.4-10-30 installed service pin envelope", head, metal, color)],
    )
