"""Build the two stepped mast webs and four welded tube extensions in Fusion."""
from pathlib import Path
import runpy

import adsk.core
import adsk.fusion


ASSEMBLY = "Team 1780 - lift bridge supports"
WEB = "Mast support - stepped web"
TUBE = "Mast support - bridge extension"
BRIDGES = (101.6, 279.4)
LEGACY_PREFIXES = tuple("frame fastening " + kind + "-" for kind in
                        ("adapter", "sleeve", "bolt", "head", "nut"))


def install(design):
    root = design.rootComponent
    if root.occurrences.itemByName(ASSEMBLY + ":1"):
        raise ValueError("The lift bridge supports already exist")
    if not any(o.component.name == "Team 1780 - swerve frame and electronics supports"
               for o in root.occurrences):
        raise ValueError("The supported MAXSwerve base is required")
    frame = runpy.run_path(str(Path(__file__).with_name("build_swerve_frame.py")))
    point, extrude = frame["point"], frame["extrude"]
    for key, value in (("mastWebThickness", 8), ("mastExtensionLength", 12)):
        design.userParameters.add(key, adsk.core.ValueInput.createByString(f"{value} mm"),
                                  "mm", "Swerve mast support stock")
    assembly = root.occurrences.addNewComponent(adsk.core.Matrix3D.create()).component
    assembly.name = ASSEMBLY

    def at(x, y, z):
        # The sketch lies in robot X-Z. Symmetric extrusion runs along robot Y.
        vector = adsk.core.Vector3D.create
        transform = adsk.core.Matrix3D.create()
        transform.setWithCoordinateSystem(point(x,y,z), vector(1,0,0),
                                          vector(0,0,1), vector(0,-1,0))
        return transform

    parts = assembly.occurrences
    web = parts.addNewComponent(at(75, -329, 120.65)).component
    web.name = WEB
    sketch = web.sketches.add(web.xYConstructionPlane)
    sketch.name = "Raised front edge clears the module plate"
    outline = [(0,0), (115,0), (115,4.6), (218,4.6), (218,66.865), (0,66.865)]
    for i, start in enumerate(outline):
        end = outline[(i+1) % len(outline)]
        sketch.sketchCurves.sketchLines.addByTwoPoints(point(*start,0),point(*end,0))
    new_body = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
    extrude(web,sketch.profiles.item(0),"mastWebThickness",new_body,
            "Cut stepped web from plate",symmetric=True)
    parts.addExistingComponent(web,at(75,329,120.65))
    tube = parts.addNewComponent(at(BRIDGES[0],-319,162.115)).component
    tube.name = TUBE
    section = tube.sketches.add(tube.xYConstructionPlane)
    section.name = "Match the retained bridge tube wall"
    frame["section_rectangle"](section,25.4,50.8,"25.4 mm","50.8 mm")
    frame["section_rectangle"](section,19.05,44.45,"19.05 mm","44.45 mm")
    ring = next(profile for profile in section.profiles if profile.profileLoops.count == 2)
    extrude(tube,ring,"mastExtensionLength",new_body,"Saw bridge extension",symmetric=True)
    for x,y in ((BRIDGES[0],319),(BRIDGES[1],-319),(BRIDGES[1],319)):
        parts.addExistingComponent(tube,at(x,y,162.115))
    if design.snapshots.hasPendingSnapshot:
        design.snapshots.add()
    return assembly


def retire_old_mounts(design):
    root = design.rootComponent
    old = [o for o in root.occurrences if o.component.name.startswith(LEGACY_PREFIXES)]
    expected = {f"frame fastening {kind}-{bridge}-{side}"
                for kind in ("adapter","sleeve") for bridge in (0,1) for side in (-1,1)}
    expected.update(f"frame fastening {kind}-{bridge}-{side}-{hole}"
                    for kind in ("bolt","head","nut") for bridge in (0,1)
                    for side in (-1,1) for hole in ("tube","rail0","rail1"))
    if len(old)!=44 or {o.component.name for o in old}!=expected:
        raise ValueError("Original bridge adapter inventory differs from the supported chassis")
    start = design.timeline.count
    for occurrence in old:
        name = occurrence.component.name
        root.features.removeFeatures.add(occurrence).name = "Retire " + name
    design.timeline.timelineGroups.add(start,design.timeline.count-1).name = "Retire old bridge adapters"
    return len(old)


def run(context):
    """Build a private derivative from a local supported-chassis archive in Fusion."""
    app = adsk.core.Application.get()
    source = app.userInterface.createFileDialog()
    source.title = "Open the supported MAXSwerve chassis archive"
    source.filter = "Fusion archive (*.f3d)"
    if source.showOpen()!=adsk.core.DialogResults.DialogOK:
        return
    destination = app.userInterface.createFileDialog()
    destination.title = "Save the robot with lift bridge supports"
    destination.filter = source.filter
    destination.initialFilename = "Team 1780 - MAXSwerve Lift Supports.f3d"
    if destination.showSave()!=adsk.core.DialogResults.DialogOK:
        return
    output = Path(destination.filename).with_suffix(".f3d")
    if output.exists():
        raise ValueError("Choose a new archive filename")
    previous = app.activeDocument
    options = app.importManager.createFusionArchiveImportOptions(source.filename)
    draft = app.importManager.importToNewDocument(options)
    try:
        draft.name = "Team 1780 - Lift Bridge Export"
        design = adsk.fusion.Design.cast(draft.products.itemByProductType("DesignProductType"))
        install(design)
        retire_old_mounts(design)
        if not design.computeAll():
            raise ValueError("Could not recompute the supported robot")
        export = design.exportManager.createFusionArchiveExportOptions(str(output))
        if not design.exportManager.execute(export):
            raise ValueError("Fusion archive export failed")
    finally:
        draft.close(False)
        if previous and previous.isValid:
            previous.activate()
