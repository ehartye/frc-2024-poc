"""Author the original three-rail, open-intake frame subassembly in Fusion. Units are mm."""
import adsk.core
import adsk.fusion


def point(x, y, z):
    return adsk.core.Point3D.create(x/10, y/10, z/10)


def placement(x, y, z, *, rail=None):
    vector = adsk.core.Vector3D.create
    basis = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
    if rail == "side":
        basis = ((0, 1, 0), (0, 0, 1), (1, 0, 0))
    elif rail == "rear":
        basis = ((-1, 0, 0), (0, 0, 1), (0, 1, 0))
    transform = adsk.core.Matrix3D.create()
    transform.setWithCoordinateSystem(point(x,y,z), *(vector(*v) for v in basis))
    return transform


def section_rectangle(sketch, width, height, width_expression, height_expression):
    curves = sketch.sketchCurves.sketchLines.addCenterPointRectangle(
        point(0,0,0), point(width/2,height/2,0))
    orientation = adsk.fusion.DimensionOrientations
    dims = sketch.sketchDimensions
    for horizontal, expression in ((True,width_expression), (False,height_expression)):
        for line in curves:
            if line.isConstruction:
                continue
            a, b = line.startSketchPoint, line.endSketchPoint
            if (abs(a.geometry.y-b.geometry.y) < 1e-8) != horizontal:
                continue
            direction = orientation.HorizontalDimensionOrientation if horizontal else orientation.VerticalDimensionOrientation
            dim = dims.addDistanceDimension(a,b,direction,point(width/2+8,height/2+8,0))
            dim.parameter.expression = expression
            break


def extrude(component, profiles, extent, operation, name, *, symmetric=False):
    distance = adsk.core.ValueInput.createByString(extent)
    extrudes = component.features.extrudeFeatures
    spec = extrudes.createInput(profiles, operation)
    if symmetric:
        spec.setSymmetricExtent(distance, True)
    else:
        spec.setDistanceExtent(False,distance)
    if operation == adsk.fusion.FeatureOperations.CutFeatureOperation:
        spec.participantBodies = list(component.bRepBodies)
    feature = extrudes.add(spec)
    feature.name = name
    return feature


def named_circle(sketch, center, diameter, expression):
    circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(center, diameter/20)
    label = adsk.core.Point3D.create(center.x+.4,center.y+.4,0)
    sketch.sketchDimensions.addDiameterDimension(circle,label).parameter.expression = expression
    return circle


def make_rail(component):
    sketch = component.sketches.add(component.xYConstructionPlane)
    sketch.name = "Rectangular tube section"
    section_rectangle(sketch,38.1,50.8,"frameRailWidth","frameRailHeight")
    section_rectangle(sketch,31.75,44.45,"frameRailWidth - 2 * frameWall","frameRailHeight - 2 * frameWall")
    ring = next(p for p in sketch.profiles if p.profileLoops.count == 2)
    ops = adsk.fusion.FeatureOperations
    extrude(component,ring,"frameRailLength",ops.NewBodyFeatureOperation,"Cut rectangular tube to length",symmetric=True)
    holes = component.sketches.add(component.xZConstructionPlane)
    holes.name = "Outer pair of housing bores at each end"
    for station in (-228.25,228.25):
        for offset in (-12.7,12.7):
            center = holes.modelToSketchSpace(point(offset,0,station))
            circle = named_circle(holes,center,5.1,"frameBoltClearance")
            circle.centerSketchPoint.isFixed = True
    profiles = adsk.core.ObjectCollection.create()
    for profile in holes.profiles:
        profiles.add(profile)
    extrude(component,profiles,"frameRailHeight + 5 mm",ops.CutFeatureOperation,"Drill both tube walls",symmetric=True)
    component.bRepBodies.item(0).name = "Tab-mounted rectangular rail"


def sleeve_placement(x, y, rear):
    # The outer holes sit 3.175 mm from the inner wall. Face the milled
    # side of the sleeve toward that wall at every bolt station.
    radial = x+329.85 if rear else y-(329.85 if y>0 else -329.85)
    axis = (1 if radial>0 else -1,0) if rear else (0,1 if radial>0 else -1)
    v = adsk.core.Vector3D.create
    mat = adsk.core.Matrix3D.create()
    mat.setWithCoordinateSystem(point(x,y,95.25),v(*axis,0),v(-axis[1],axis[0],0),v(0,0,1))
    return mat


def face_sleeve(component):
    milling = component.sketches.add(component.xYConstructionPlane)
    milling.name = "Sleeve wall clearance"
    milling.sketchCurves.sketchLines.addTwoPointRectangle(point(3.05,-6,0),point(7,6,0))
    extrude(component,milling.profiles.item(0),"frameRailHeight",adsk.fusion.FeatureOperations.CutFeatureOperation,
            "Mill flat 3.05 mm from bolt axis",symmetric=True)


def make_sleeve(component):
    sketch = component.sketches.add(component.xYConstructionPlane)
    sketch.name = "Crush sleeve section"
    for diameter, expr in ((9.5,"frameSleeveOD"),(5.1,"frameBoltClearance")):
        named_circle(sketch,point(0,0,0),diameter,expr)
    ring = next(p for p in sketch.profiles if p.profileLoops.count == 2)
    extrude(component,ring,"frameRailHeight - 2 * frameWall",adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
            "Face to the clear distance between tube walls",symmetric=True)
    face_sleeve(component)
    component.bRepBodies.item(0).name = "Through-bolt crush sleeve"


def build(design, root=None):
    params = dict(frameRailWidth=38.1,frameRailHeight=50.8,frameWall=3.175,frameRailLength=480.,
                  frameBoltClearance=5.1,frameSleeveOD=9.5)
    for key, number in params.items():
        design.userParameters.add(key,adsk.core.ValueInput.createByString(f"{number} mm"),"mm","Swerve rail stock or drilling dimension")
    root = root or design.rootComponent
    occurrences = root.occurrences
    rail = None
    sleeve = None
    for x,y,kind in ((0,329.85,"side"),(0,-329.85,"side"),(-329.85,0,"rear")):
        mount = placement(x,y,95.25,rail=kind)
        if rail is None:
            rail = occurrences.addNewComponent(mount).component
            rail.name = "Swerve frame - interchangeable drilled rail"
            make_rail(rail)
        else:
            occurrences.addExistingComponent(rail,mount)
        for station in (-228.25,228.25):
            for offset in (-12.7,12.7):
                cx,cy = (x-offset,station) if kind=="rear" else (station,y+offset)
                loc = sleeve_placement(cx,cy,kind=="rear")
                if sleeve is None:
                    sleeve = occurrences.addNewComponent(loc).component
                    sleeve.name = "Swerve frame - bolt crush sleeve"
                    make_sleeve(sleeve)
                else:
                    occurrences.addExistingComponent(sleeve,loc)
    if design.snapshots.hasPendingSnapshot:
        design.snapshots.add()
    assert design.computeAll()
    return root

def run(context):
    """Fusion Scripts entry point. Export a new native frame and verify the reopened archive."""
    from pathlib import Path
    import runpy

    app = adsk.core.Application.get()
    dialog = app.userInterface.createFileDialog()
    dialog.title = "Save the Team 1780 frame and deck supports"
    dialog.filter = "Fusion archive (*.f3d)"
    dialog.initialFilename = "Team 1780 - Swerve Frame and Deck Supports.f3d"
    if dialog.showSave() != adsk.core.DialogResults.DialogOK:
        return
    path = Path(dialog.filename).with_suffix(".f3d")
    if path.exists():
        raise ValueError("Choose a new archive filename")
    here = Path(__file__).resolve().parent
    previous = app.activeDocument
    draft = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    try:
        draft.name = "Team 1780 - Frame and Deck Supports - Export"
        design = adsk.fusion.Design.cast(draft.products.itemByProductType("DesignProductType"))
        design.designType = adsk.fusion.DesignTypes.ParametricDesignType
        root = build(design)
        runpy.run_path(str(here/"swerve_frame_fasteners.py"))["install"](root,globals())
        runpy.run_path(str(here/"swerve_deck_support.py"))["install"](design,globals())
        if not design.computeAll() or not design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(str(path))):
            raise RuntimeError("Frame export failed")
    finally:
        draft.close(False)
        if previous and previous.isValid:
            previous.activate()
    opened = app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(path)))
    try:
        opened.name = "Team 1780 - Frame and Deck Supports - Reopen Check"
        design = adsk.fusion.Design.cast(opened.products.itemByProductType("DesignProductType"))
        result = runpy.run_path(str(here/"verify_swerve_frame.py"))["inspect"](design.rootComponent)
        app.log(f"Verified {sum(result['articleCounts'].values())} frame articles in {path.name}")
    finally:
        opened.close(False)
        if previous and previous.isValid:
            previous.activate()
