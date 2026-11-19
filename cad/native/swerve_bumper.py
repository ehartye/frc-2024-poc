"""Native bumper rails, drilled plywood and eight removable M6 fasteners (mm)."""
import math
from pathlib import Path
import runpy

import adsk.core
import adsk.fusion


ASSEMBLY = "Team 1780 - removable bumper mounts"
ORIGINAL = "original rigid front backing support proposal"
# Outward normal, inner wood face, bolt head seat depth below wood, wood half length.
FACES = {"Front":((1,0),359.537,9.525,355.6), "Rear":((-1,0),359.537,10.637,355.6),
         "Left":((0,1),355.6,6.7,378.587), "Right":((0,-1),355.6,6.7,378.587)}
STATIONS = {"Front":(-235,235),"Rear":(-175,175),"Left":(-326,175),"Right":(-175,326)}
WOOD_NAMES = {"Front":"fixed bumper end backing 1", "Rear":"fixed bumper end backing -1",
              "Left":"fixed bumper side backing 1", "Right":"fixed bumper side backing -1"}
BOLT_HEIGHT = 145.
WOOD = 19.05


def placement(face, u=0., height=0., depth=0.):
    (nx,ny),radius,_,_ = FACES[face]
    transform = adsk.core.Matrix3D.create()
    point,vector = adsk.core.Point3D.create,adsk.core.Vector3D.create
    transform.setWithCoordinateSystem(point(((radius+depth)*nx-u*ny)/10,
                                            ((radius+depth)*ny+u*nx)/10,height/10),
                                      vector(-ny,nx,0),vector(0,0,1),vector(nx,ny,0))
    return transform


def install(design):
    root = design.rootComponent
    if root.occurrences.itemByName(ASSEMBLY+":1") or not root.occurrences.itemByName("Team 1780 - lift bridge supports:1"):
        raise ValueError("Expected the R5 supported chassis")
    replace = {ORIGINAL,*WOOD_NAMES.values()}
    old = [o for o in root.occurrences if o.component.name in replace]
    if len(old)!=5 or {o.component.name for o in old}!=replace:
        raise ValueError("Expected the original front plate and four wood backings")
    # A different source panel (including drilled or relieved wood) needs its
    # own machining plan. Check full solid volumes before substituting panels.
    for o in old:
        name = o.component.name
        if name==ORIGINAL:
            expected = 3.175*711.2*127
        else:
            half = FACES[next(f for f,n in WOOD_NAMES.items() if n==name)][3]
            expected = 2*half*127*WOOD
        if o.bRepBodies.count!=1 or abs(o.bRepBodies.item(0).volume*1000-expected)>.1:
            raise ValueError(f"Unexpected retained material in {name}")
    timeline_start = design.timeline.count
    for o in old:
        name=o.component.name
        root.features.removeFeatures.add(o).name="Replace "+name
    design.timeline.timelineGroups.add(timeline_start,design.timeline.count-1).name="Replace original bumper blanks"
    mount = root.occurrences.addNewComponent(adsk.core.Matrix3D.create()).component
    mount.name = ASSEMBLY
    frame = runpy.run_path(str(Path(__file__).with_name("build_swerve_frame.py")))
    point,extrude = frame["point"],frame["extrude"]
    ops = adsk.fusion.FeatureOperations
    new,join,cut = ops.NewBodyFeatureOperation,ops.JoinFeatureOperation,ops.CutFeatureOperation

    def sketch(part,z,title):
        plane = part.xYConstructionPlane
        if abs(z)>1e-8:
            planes = part.constructionPlanes
            offset = planes.createInput()
            offset.setByOffset(plane,adsk.core.ValueInput.createByString(f"{z} mm"))
            plane = planes.add(offset)
        drawing = part.sketches.add(plane)
        drawing.name = title
        return drawing

    def feature(part,drawing,dz,operation,title):
        profiles = adsk.core.ObjectCollection.create()
        for profile in drawing.profiles:
            profiles.add(profile)
        return extrude(part,profiles,f"{dz} mm",operation,title)

    def rectangle(part,u0,u1,h0,h1,z0,z1,operation,title):
        drawing = sketch(part,z0,title+" profile")
        drawing.sketchCurves.sketchLines.addTwoPointRectangle(point(u0,h0,0),point(u1,h1,0))
        feature(part,drawing,z1-z0,operation,title)

    def drill(part,stations,height,r,z0,z1,operation,title):
        drawing = sketch(part,z0,title+" profile")
        for u in stations:
            drawing.sketchCurves.sketchCircles.addByCenterRadius(point(u,height,0),r/10)
        feature(part,drawing,z1-z0,operation,title)

    def part_at(name,pose):
        o = mount.occurrences.addNewComponent(pose)
        o.component.name = name
        return o

    screw = nut = None
    for face,(_,radius,thick,half) in FACES.items():
        stations = STATIONS[face]
        rail = part_at(face+" bumper frame rail",placement(face)).component
        if face=="Front":
            rectangle(rail,-half,half,58.5,185.5,-3.175,0,new,"Continuous center skin")
            for lo,hi in ((-half,-210),(210,half)):
                rectangle(rail,lo,hi,58.5,185.5,-thick,-3.175,join,"Full height outer reinforcement")
            rectangle(rail,-210,210,160.1,185.5,-thick,-3.175,join,"Upper reinforcement above NOTE")
        else:
            # Local U increases toward -X on the left side and +X on the right.
            limits = {"Left":(-350.012,348.9),"Right":(-348.9,350.012),"Rear":(-355.25,355.25)}[face]
            gap = 1.112 if face=="Rear" else .35
            rectangle(rail,*limits,74.6,112.7,-thick,-gap,new,"Lower frame backing bar")
            for u in stations:
                rectangle(rail,u-15,u+15,112.7,160,-thick,0,join,"Full depth screw landing")
        drill(rail,stations,BOLT_HEIGHT,3.3,-thick,0,cut,"Drill M6 clearance")

        wood_occ = part_at(face+" drilled bumper wood",placement(face))
        wood = wood_occ.component
        rectangle(wood,-half,half,58.5,185.5,0,WOOD,new,"Original plywood blank")
        drill(wood,stations,BOLT_HEIGHT,4.6,0,WOOD,cut,"Barrel nut clearance")
        for u in stations:
            rectangle(wood,u-7.6,u+7.6,BOLT_HEIGHT-7.6,BOLT_HEIGHT+7.6,
                      WOOD-1.5,WOOD,cut,"Square nut anti-rotation pocket")
        wood_occ.isLightBulbOn = False
        for u in stations:
            bolt_pose = placement(face,u,BOLT_HEIGHT,-thick)
            nut_pose = placement(face,u,BOLT_HEIGHT,WOOD)
            if screw is None:
                screw = part_at("M6 x 25 bumper socket screw",bolt_pose).component
                drill(screw,[0],0,3,0,25,new,"Nominal M6 shaft")
                drill(screw,[0],0,5,-6,0,join,"Socket head")
                socket = sketch(screw,-6,"Five mm hex socket")
                rad = 5/math.sqrt(3)
                polygon = [point(rad*math.cos(i*math.pi/3),rad*math.sin(i*math.pi/3),0) for i in range(6)]
                for i in range(6):
                    socket.sketchCurves.sketchLines.addByTwoPoints(polygon[i],polygon[(i+1)%6])
                feature(screw,socket,3.2,cut,"Broach hex recess")
                nut = part_at("M6 captive square flange barrel nut",nut_pose).component
                rectangle(nut,-7.5,7.5,-7.5,7.5,-1.5,0,new,"Anti-rotation flange")
                drill(nut,[0],0,4.5,-12,-1.5,join,"Integral threaded barrel")
                drill(nut,[0],0,3,-12,0,cut,"M6 nominal thread envelope")
            else:
                mount.occurrences.addExistingComponent(screw,bolt_pose)
                mount.occurrences.addExistingComponent(nut,nut_pose)
    if design.snapshots.hasPendingSnapshot:
        design.snapshots.add()
    if not design.computeAll():
        raise ValueError("Bumper recompute failed")
    return mount


def run(context):
    """Export a new private bumper revision from the saved lift-support chassis."""
    app = adsk.core.Application.get()
    source = app.userInterface.createFileDialog()
    source.title = "Open the MAXSwerve chassis with lift bridge supports"
    source.filter = "Fusion archive (*.f3d)"
    if source.showOpen()!=adsk.core.DialogResults.DialogOK:
        return
    dest = app.userInterface.createFileDialog()
    dest.title = "Save the robot with removable bumper mounts"
    dest.filter = source.filter
    dest.initialFilename = "Team 1780 - Removable Bumpers.f3d"
    if dest.showSave()!=adsk.core.DialogResults.DialogOK:
        return
    archive = Path(dest.filename).with_suffix(".f3d")
    if archive.exists():
        raise ValueError("Choose a new archive filename")
    previous = app.activeDocument
    options = app.importManager.createFusionArchiveImportOptions(source.filename)
    doc = app.importManager.importToNewDocument(options)
    try:
        doc.name = "Team 1780 - Bumper Mount Export"
        design = adsk.fusion.Design.cast(doc.products.itemByProductType("DesignProductType"))
        install(design)
        if not design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(str(archive))):
            raise ValueError("Fusion archive export failed")
    finally:
        doc.close(False)
        if previous and previous.isValid:
            previous.activate()
