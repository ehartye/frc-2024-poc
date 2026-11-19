"""Install the boxed front bumper support in the R6 native robot. Dimensions are mm."""
from pathlib import Path
import runpy

import adsk.core
import adsk.fusion


SHIFT = 34.925
TUBE_FACE = 394.462
REVISION = "Boxed front support"


def register_front(design):
    """Place the crossmember and both front cushions with component move features."""
    root = design.rootComponent
    mount = root.occurrences.itemByName("Team 1780 - removable bumper mounts:1")
    rail = mount.component.occurrences.itemByName("Front bumper frame rail:1").createForAssemblyContext(mount)
    targets = [(rail,TUBE_FACE),*[(root.occurrences.itemByName(f"fixed bumper end noodle 1 {i}:1"),477.012)
                                 for i in (0,1)]]
    manager = adsk.fusion.TemporaryBRepManager.get()
    def extent(occ):
        solid = manager.copy(occ.component.bRepBodies.item(0))
        if not manager.transform(solid,occ.transform2):
            raise ValueError("Could not register the front body")
        return 10*solid.preciseBoundingBox.maxPoint.x
    for occ,target in targets:
        delta = target-extent(occ)
        if abs(delta)<.001:
            continue
        if abs(delta-SHIFT)>.001:
            raise ValueError(f"Unexpected original datum on {occ.name}")
        local = adsk.core.Vector3D.create(SHIFT,0,0)
        inverse = occ.transform2
        inverse.invert()
        local.transformBy(inverse)
        bodies = adsk.core.ObjectCollection.create()
        bodies.add(occ.component.bRepBodies.item(0))
        moves = occ.component.features.moveFeatures
        spec = moves.createInput2(bodies)
        distances = [adsk.core.ValueInput.createByString(f"{d:.6f} mm") for d in local.asArray()]
        if not spec.defineAsTranslateXYZ(*distances,False):
            raise ValueError("Cannot define front body placement")
        moves.add(spec).name = "Register front body to the boxed frame"
    if not design.computeAll() or any(abs(extent(o)-x)>.001 for o,x in targets):
        raise ValueError("Front body placement failed recompute")


def install(design):
    root = design.rootComponent
    mount_occ = root.occurrences.itemByName("Team 1780 - removable bumper mounts:1")
    if mount_occ is None:
        raise ValueError("Choose the robot with the R6 removable bumper assembly")
    mount = mount_occ.component
    if mount.attributes.itemByName("bumper", "revision") or mount.occurrences.count != 24:
        raise ValueError("Expected the R6 removable bumper assembly")
    frame = runpy.run_path(str(Path(__file__).with_name("build_swerve_frame.py")))
    point, extrude = frame["point"], frame["extrude"]
    ops = adsk.fusion.FeatureOperations
    value = adsk.core.ValueInput.createByString

    def named(owner, name):
        found = [o for o in owner.occurrences if o.component.name == name]
        if len(found) != 1:
            raise ValueError(f"Expected one {name}")
        return found[0].createForAssemblyContext(mount_occ) if owner == mount else found[0]

    def offset_end(occ, x, amount, label):
        # Select the planar face in assembly coordinates. The feature itself
        # belongs to the native component, including when it is rotated.
        matches = []
        for f in occ.bRepBodies.item(0).faces:
            bb = f.boundingBox
            if (adsk.core.Plane.cast(f.geometry) and
                    max(abs(10*p.x-x) for p in (bb.minPoint, bb.maxPoint)) < .025):
                matches.append(f.nativeObject or f)
        if len(matches) != 1:
            raise ValueError(f"Expected a unique end face on {occ.name} at {x}")
        offsets = occ.component.features.offsetFacesFeatures
        offsets.add(offsets.createInput(matches, value(f"{amount} mm"))).name = label

    def advance(occ, distance, *, root_occurrence=False):
        grounded = root_occurrence and occ.isGrounded
        if grounded:
            occ.isGrounded = False
        pose = occ.transform2
        translation = pose.translation
        translation.x += distance/10
        pose.translation = translation
        occ.transform2 = pose
        if design.snapshots.hasPendingSnapshot:
            design.snapshots.add()
        if grounded:
            occ.isGrounded = True

    rail_occ = named(mount, "Front bumper frame rail")
    rail = rail_occ.component
    old = rail.bRepBodies.item(0)
    if abs(rail_occ.transform2.translation.x*10 - (TUBE_FACE-SHIFT)) > .001:
        raise ValueError("Unexpected front rail datum")
    rail.features.removeFeatures.add(old).name = "Retire the stepped front plate"
    # Component axes point across the robot, upward, and forward, respectively.
    section = rail.sketches.add(rail.yZConstructionPlane)
    section.name = "Front crossmember tube section"
    for wall in (0., 3.175):
        a = section.modelToSketchSpace(point(0,74.6+wall,-38.1+wall))
        b = section.modelToSketchSpace(point(0,125.4-wall,-wall))
        section.sketchCurves.sketchLines.addTwoPointRectangle(a,b)
    ring = next(p for p in section.profiles if p.profileLoops.count == 2)
    extrude(rail,ring,"711.2 mm",ops.NewBodyFeatureOperation,"Cut front box tube",symmetric=True)
    for u in (-235,235):
        lug = rail.sketches.add(rail.xYConstructionPlane)
        lug.sketchCurves.sketchLines.addTwoPointRectangle(point(u-15,125.4,0),point(u+15,160,0))
        extrude(rail,lug.profiles.item(0),"-9.525 mm",ops.JoinFeatureOperation,"Weld front screw landing")
    holes = rail.sketches.add(rail.xYConstructionPlane)
    for u in (-235,235):
        holes.sketchCurves.sketchCircles.addByCenterRadius(point(u,145,0),.33)
    profiles = adsk.core.ObjectCollection.create()
    for p in holes.profiles:
        profiles.add(p)
    extrude(rail,profiles,"-9.525 mm",ops.CutFeatureOperation,"Drill two front M6 clearances")
    rail.bRepBodies.item(0).name = "Box tube with welded screw landings"

    for side in ("Left","Right"):
        offset_end(named(mount,side+" bumper frame rail"),350.012,6.35,"Extend rail to crossmember")
        offset_end(named(mount,side+" drilled bumper wood"),378.587,SHIFT,"Lengthen side plywood")
    front_hardware = [o.createForAssemblyContext(mount_occ) for o in mount.occurrences if o.component.name.startswith("M6 ")
                      and o.transform2.translation.x*10 > 345]
    if len(front_hardware) != 4:
        raise ValueError("Expected four front screw and nut occurrences")
    for occ in [named(mount,"Front drilled bumper wood"), *front_hardware]:
        advance(occ,SHIFT)
    for sign in (-1,1):
        for level in (0,1):
            foam = named(root,f"fixed bumper side noodle {sign} {level}")
            offset_end(foam,378.587,SHIFT,"Lengthen the side cushion")
    cover = named(root,"Continuous bumper native display v2")
    offset_end(cover,443.587,SHIFT,"Extend the outer cover face")
    offset_end(cover,359.537,-SHIFT,"Advance the inner cover face")
    advance(named(root,"1780 WHITE team marking x1 - visual only"),SHIFT,root_occurrence=True)
    for sign in (-1,1):
        advance(named(root,f"1780 WHITE team marking y{sign} - visual only"),SHIFT/2,root_occurrence=True)
    if not design.computeAll():
        raise ValueError("Boxed front structure failed to recompute")
    register_front(design)
    mount.attributes.add("bumper","revision",REVISION)
    return mount


def run(context):
    """Choose the saved R6 assembly and export the revision as a new archive."""
    app = adsk.core.Application.get()
    source = app.userInterface.createFileDialog()
    source.title = "Open the robot with the stepped front bumper plate"
    source.filter = "Fusion archive (*.f3d)"
    if source.showOpen()!=adsk.core.DialogResults.DialogOK:
        return
    dest = app.userInterface.createFileDialog()
    dest.title = "Save the robot with the boxed front support"
    dest.initialFilename = "Team 1780 - MAXSwerve Front Crossmember.f3d"
    dest.filter = source.filter
    if dest.showSave()!=adsk.core.DialogResults.DialogOK:
        return
    target = Path(dest.filename).with_suffix(".f3d")
    if target.exists():
        raise ValueError("Choose a new archive filename")
    previous = app.activeDocument
    options = app.importManager.createFusionArchiveImportOptions(source.filename)
    doc = app.importManager.importToNewDocument(options)
    try:
        doc.name = "Team 1780 - Front Crossmember - Temporary Build"
        design = adsk.fusion.Design.cast(doc.products.itemByProductType("DesignProductType"))
        install(design)
        if not design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(str(target))):
            raise ValueError("Boxed front archive export failed")
    finally:
        doc.close(False)
        if previous and previous.isValid:
            previous.activate()
