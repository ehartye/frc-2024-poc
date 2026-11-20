"""Build the R8 Stage study on a fresh import of the verified student R7 in Fusion.

Lengths in this recipe are millimetres. All generated solids are original study
geometry. The screw transmission, nut and return spring have envelope geometry.
"""
from collections import defaultdict
from hashlib import sha256
import json
from pathlib import Path
import struct
import sys

import adsk.core as core
import adsk.fusion as fusion

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts/showcase"))
from export_swerve import dimensions, registered_solid, tessellate

BASE = ROOT / "cad/student/r7-maxswerve.f3d"
ARCHIVE = ROOT / "cad/student/r8-stage-climber.f3d"
RECEIPT = ROOT / "cad/student/r8-reopened.json"
BASE_HASH = "6e2b65f3493dfd0a8c7a27bbf26445dec29213e93806ff9b0c55df5548b00266"
NAMES = ("R8 fixed climb mast", "R8 sliding saddle", "R8 transverse lock")
ROLE = ("structure", "mechanism", "reference")


def pt(x, y, z):
    return core.Point3D.create(x/10, y/10, z/10)


def box(x0, y0, z0, x1, y1, z1):
    center = pt((x0+x1)/2, (y0+y1)/2, (z0+z1)/2)
    shape = core.OrientedBoundingBox3D.create(center, core.Vector3D.create(1,0,0),
        core.Vector3D.create(0,1,0), (x1-x0)/10, (y1-y0)/10, (z1-z0)/10)
    return fusion.TemporaryBRepManager.get().createBox(shape)


def rod(a, b, diameter):
    return fusion.TemporaryBRepManager.get().createCylinderOrCone(pt(*a), diameter/20, pt(*b), diameter/20)


def cut(solid, *tools):
    for tool in tools:
        if not fusion.TemporaryBRepManager.get().booleanOperation(solid, tool, fusion.BooleanTypes.DifferenceBooleanType):
            raise ValueError("Failed to cut a structural bore")
    return solid


def geometry():
    # Saddle throat passes the whole chain cross section along Y. The robot hangs
    # from the underside of one continuous roof. Neither cheek hooks a second link.
    beam = cut(box(13,-348.4,120.65,63,348.4,159), box(16,-349,123.65,60,349,156))
    fixed = []
    for y in (-329,329):
        for x in (23,53):
            beam = cut(beam, rod((x,y,120),(x,y,160),8.5))
            bolt = rod((x,y,62),(x,y,159),8)
            head = rod((x,y,158.8),(x,y,165),14)
            if not fusion.TemporaryBRepManager.get().booleanOperation(bolt,head,fusion.BooleanTypes.UnionBooleanType):
                raise ValueError("Failed to join fastener envelope")
            fixed.append((f"Rail mounting fastener envelope {x} {y}",bolt))
            for name,z0,z1,od in (("Rail crush sleeve",73.025,117.475,13),
                                  ("Crossmember crush sleeve",123.65,156,13),("Captive nut envelope",62,69.85,14)):
                sleeve=cut(rod((x,y,z0),(x,y,z1),od),rod((x,y,z0-1),(x,y,z1+1),8.5))
                fixed.append((f"{name} {x} {y}",sleeve))
    bore = lambda: rod((20,-19,246),(54,-19,246),10.5)
    fixed.extend([
        ("Crossmember with four rail mounting holes", beam),
        ("Outer guide with double shear lock bore", cut(box(24,-56,159,52,-4,600),
             box(27,-53,158,49,-7,601), bore())),
        ("Thrust bearing support envelope", cut(box(27,-53,159,49,-7,169),rod((38,-39,158),(38,-39,170),10.5))),
        ("Screw envelope",rod((38,-39,159),(38,-39,592),10)),
        ("Drive gearbox envelope",box(25,-4,160,51,48,210)),
        ("Drive motor envelope",rod((38,48,186),(38,110,186),36)),
        ("Spring return housing envelope", cut(box(52,-28,236,130,-10,256),
             rod((51,-19,246),(131,-19,246),11))),
    ])
    sliding = [
        ("Inner sliding tube with lock bore",cut(box(28,-52,170,48,-8,620),box(30.5,-49.5,169,45.5,-10.5,621),bore())),
        ("Nut carrier plate",cut(box(30.5,-49.5,170,45.5,-10.5,175),rod((38,-39,169),(38,-39,176),10.5))),
        ("Travelling nut envelope",cut(box(31,-49,175,45,-30,192),rod((38,-39,170),(38,-39,193),10.5))),
        ("Top bracket",box(16,-49,620,48,-11,630)),
        ("Saddle spine",box(16,-47,480,22,-13,620)),
        ("Chain bearing roof",box(-28,-47,480,16,-13,493)),
        ("Retaining lip",box(-28,-47,453,-19,-13,480)),
    ]
    lock = [("Spring engaged through pin",rod((22,-19,246),(87,-19,246),10))]
    return fixed, sliding, lock


def document(archive):
    app = core.Application.get()
    return app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(archive)))


def joint(root, name, child, parent, axis=None, max_mm=0):
    anchor = child.component.originConstructionPoint.createForAssemblyContext(child)
    geom = fusion.JointGeometry.createByPoint(anchor) if axis is not None else None
    request = root.asBuiltJoints.createInput(child,parent,geom)
    if axis is not None:
        request.setAsSliderJointMotion(axis)
    else:
        request.setAsRigidJointMotion()
    result = root.asBuiltJoints.add(request)
    result.name = name
    if axis is not None:
        limits = result.jointMotion.slideLimits
        limits.minimumValue,limits.maximumValue = sorted((0.,max_mm/10))
        limits.isMinimumValueEnabled = limits.isMaximumValueEnabled = True
    return result


def side_rails(comp):
    return [body for body in comp.bRepBodies if body.name.endswith("Tab-mounted rectangular rail")
            and abs(dimensions(body)[0]+240)<.01]


def drill_mounts(chassis):
    comp=chassis.component
    rails=side_rails(comp)
    if len(rails)!=2:
        raise ValueError("Expected both R7 side rails")
    sketch=comp.sketches.add(comp.xYConstructionPlane)
    sketch.name="R8 crossmember through-bolt centers"
    for y in (-329,329):
        for x in (23,53):
            c=sketch.sketchCurves.sketchCircles.addByCenterRadius(pt(x,y,0),.425)
            c.isFixed=True
    profiles=core.ObjectCollection.create()
    for profile in sketch.profiles: profiles.add(profile)
    if profiles.count!=4: raise ValueError("Four bolt bores required")
    tool=comp.features.extrudeFeatures
    req=tool.createInput(profiles,fusion.FeatureOperations.CutFeatureOperation)
    req.setDistanceExtent(False,core.ValueInput.createByString("125 mm"))
    req.participantBodies=rails
    tool.add(req).name="Drill four 8.5 mm mounting holes through the R7 rails"


def build():
    if sha256(BASE.read_bytes()).hexdigest() != BASE_HASH:
        raise ValueError("R7 student archive identity changed")
    if ARCHIVE.exists():
        raise FileExistsError(ARCHIVE)
    app = core.Application.get()
    prior = app.activeDocument
    doc = document(BASE)
    try:
        doc.name = "R8 Stage study on R7"
        d = fusion.Design.cast(doc.products.itemByProductType("DesignProductType"))
        root = d.rootComponent
        chassis = next(o for o in root.occurrences if o.component.name == "R7 chassis")
        if root.asBuiltJoints.count != 14 or root.allOccurrences.count != 15:
            raise ValueError("Unexpected R7 base")
        drill_mounts(chassis)
        added = []
        for name, role, items in zip(NAMES, ROLE, geometry()):
            occ = root.occurrences.addNewComponent(core.Matrix3D.create())
            comp = occ.component
            comp.name = name
            base = comp.features.baseFeatures.add()
            base.name = "Stage climb study solids in robot coordinates"
            base.startEdit()
            try:
                for label, solid in items:
                    comp.bRepBodies.add(solid,base).name = label
            finally:
                base.finishEdit()
            for body,(label,_) in zip(comp.bRepBodies,items):
                body.name = label
                body.appearance = d.appearances.itemByName("R7 "+role) or d.appearances.itemByName("R7 structure")
            added.append(occ)
        fixed, slide, lock = added
        joint(root,"Climb crossmember mount",fixed,chassis)
        joint(root,"Saddle extension",slide,fixed,fusion.JointDirections.ZAxisJointDirection,380)
        joint(root,"Lock pin withdrawal",lock,fixed,fusion.JointDirections.XAxisJointDirection,52)
        if not d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(str(ARCHIVE))):
            raise ValueError("R8 export failed")
    finally:
        doc.close(False)
        if prior and prior.isValid:
            prior.activate()


def verify(mesh_dir):
    app = core.Application.get()
    prior = app.activeDocument
    doc = document(ARCHIVE)
    try:
        d = fusion.Design.cast(doc.products.itemByProductType("DesignProductType"))
        root = d.rootComponent
        occs = {o.component.name:o for o in root.occurrences}
        if len(occs) != 18 or root.asBuiltJoints.count != 17:
            raise ValueError("Unexpected reopened R8 assembly")
        slide,lock = [root.asBuiltJoints.itemByName(n).jointMotion for n in ("Saddle extension","Lock pin withdrawal")]
        roof = occs[NAMES[1]].bRepBodies.itemByName("Chain bearing roof")
        pin = occs[NAMES[2]].bRepBodies.itemByName("Spring engaged through pin")
        poses = []
        for label, travel, withdrawal in [("home locked",0,0),("unlocked",0,52),("above chain",380,52),
                                        ("pull down",170,52),("seated stop",0,52),("passive hold",0,0)]:
            lock.slideValue = withdrawal/10
            slide.slideValue = travel/10
            app.activeViewport.refresh()
            top = dimensions(registered_solid(occs[NAMES[1]],roof))
            axle = dimensions(registered_solid(occs[NAMES[2]],pin))
            if abs(top[2]-480-travel)>.02 or abs(axle[0]-22-withdrawal)>.02:
                raise ValueError(f"Reopened joint mismatch: {label} {top} {axle}")
            poses.append(dict(pose=label,roofUnderMm=top[2],pinStartXMm=axle[0],travelMm=travel,withdrawalMm=withdrawal))
        mount_checks=[]
        for rail in side_rails(occs['R7 chassis'].component):
            y=329 if dimensions(rail)[1]>0 else -329
            for x in (23,53):
                for z in (71.4,119.1):
                    # Compare the through bore and adjacent solid in both walls.
                    inside=fusion.PointContainment.PointInsidePointContainment
                    if rail.pointContainment(pt(x+4.1,y,z))==inside or rail.pointContainment(pt(x+4.4,y,z))!=inside:
                        raise ValueError("Mounting bore or surrounding rail wall missing")
                    mount_checks.append([x,y,z])
        if len(mount_checks)!=8: raise ValueError("Incomplete mounting-wall observations")
        mesh_dir.mkdir(parents=True,exist_ok=True)
        groups = defaultdict(bytearray)
        inventory = []
        colors = {}
        for name,occ in occs.items():
            for body in occ.bRepBodies:
                solid = registered_solid(occ,body)
                appearance = body.appearance.name
                group = (name if name in NAMES else "R7 robot") + " - " + appearance
                for prop_id in ("opaque_albedo","surface_albedo"):
                    prop = core.ColorProperty.cast(body.appearance.appearanceProperties.itemById(prop_id))
                    if prop:
                        c = prop.value
                        colors[group] = [c.red/255,c.green/255,c.blue/255,1]
                        break
                groups[group].extend(tessellate(solid))
                if name in NAMES:
                    inventory.append(dict(component=name,body=body.name,boundsMm=dimensions(solid),volumeMm3=round(solid.volume*1000,3)))
        meshes = []
        for i,(name,triangles) in enumerate(groups.items()):
            if len(triangles)%50 or not triangles:
                raise ValueError("Invalid tessellation")
            raw = bytes(80)+struct.pack("<I",len(triangles)//50)+triangles
            file = f"stage-{i}.stl"
            (mesh_dir/file).write_bytes(raw)
            meshes.append(dict(name=name,file=file,triangles=len(triangles)//50,sha256=sha256(raw).hexdigest(),
                               rgba=colors.get(name,[.47,.49,.5,1]),sourceBodyKeys=[str(i)]))
        digest = sha256(ARCHIVE.read_bytes()).hexdigest()
        manifest=dict(schema="registered-student-export/1",sourceFormat="F3D",sourceSha256=digest,units="mm",
            frame="robot-floor-XYZ",meshes=meshes,bodyInventory=[dict(key=str(i)) for i in range(len(meshes))])
        (mesh_dir/"manifest.json").write_text(json.dumps(manifest,indent=2))
        receipt = dict(source="cad/student/r8-stage-climber.f3d",sha256=digest,parentSha256=BASE_HASH,
            components={n:o.bRepBodies.count for n,o in occs.items()},joints=[j.name for j in root.asBuiltJoints],
            poses=poses,railWallChecksMm=mount_checks,climberBodies=inventory,displayGroups=meshes)
        RECEIPT.write_text(json.dumps(receipt,indent=2,allow_nan=False)+"\n",encoding="utf8",newline="\n")
    finally:
        doc.close(False)
        if prior and prior.isValid:
            prior.activate()
