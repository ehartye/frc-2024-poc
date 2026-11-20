"""Extract project-authored solids from the reopened R7 robot in Fusion."""
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import struct

import adsk.core
import adsk.fusion

from public_parts import MARKINGS, public_occurrence

PALETTE = {"structure": (73,106,129), "mechanism": (0,127,120), "game-piece": (230,180,80),
           "electrical": (130,99,169), "reference": (138,145,153), "bumper": (215,25,32),
           "number": (255,255,255)}


def attribute(component, group, name, default=None):
    value = component.attributes.itemByName(group, name)
    return value.value if value else default


def registered_solid(occurrence, body):
    manager = adsk.fusion.TemporaryBRepManager.get()
    solid = manager.copy(body.nativeObject or body)
    if not manager.transform(solid, occurrence.transform2):
        raise ValueError("Failed to place native solid in robot coordinates")
    return solid


def dimensions(solid):
    box = solid.preciseBoundingBox
    return [round(10*x,5) for p in (box.minPoint,box.maxPoint) for x in p.asArray()]


def tessellate(solid):
    calculator = solid.meshManager.createMeshCalculator()
    calculator.surfaceTolerance = .02  # Fusion cm
    mesh = calculator.calculate()
    points = [v*10 for v in mesh.nodeCoordinatesAsDouble]
    indices = mesh.nodeIndices
    out = bytearray()
    for i in range(0,len(indices),3):
        a,b,c = (3*k for k in indices[i:i+3])
        out.extend(struct.pack("<12fH",0,0,0,*points[a:a+3],*points[b:b+3],*points[c:c+3],0))
    return out


def extract(archive, expected, output):
    if hashlib.sha256(archive.read_bytes()).hexdigest() != expected:
        raise ValueError("Selected F3D differs from its engineering record")
    app = adsk.core.Application.get()
    previous = app.activeDocument
    options = app.importManager.createFusionArchiveImportOptions(str(archive))
    doc = app.importManager.importToNewDocument(options)
    try:
        doc.name = "Team 1780 - Temporary showcase export"
        design = adsk.fusion.Design.cast(doc.products.itemByProductType("DesignProductType"))
        root = design.rootComponent
        top = {o.name:o.component for o in root.occurrences}
        groups = defaultdict(bytearray)
        keys = defaultdict(list)
        inventory, wheels = [], {}
        supplier_solids = 0
        for occ in root.allOccurrences:
            path = occ.fullPathName.split("+")
            owner = top[path[0]]
            if owner.name.startswith("MAXSwerve"):
                for body in occ.bRepBodies:
                    supplier_solids += int(body.isSolid)
                    if len(path)==1 and owner.name.endswith(" wheel") and body.isSolid:
                        station = owner.name.removeprefix("MAXSwerve ").removesuffix(" wheel")
                        wheels.setdefault(station,[]).append(dimensions(registered_solid(occ,body)))
                continue
            if not occ.isVisible:
                continue
            role = attribute(owner,"frcDisplay","role","structure")
            if owner.name in MARKINGS:
                role = "number"
            if owner.name == "Continuous bumper native display v2":
                role = "bumper"
            motion = attribute(owner,"frcSource","motionGroup","fixed")
            if role == "reserved-space":
                continue
            for body in occ.bRepBodies:
                if not body.isVisible or not body.isSolid:
                    continue
                row = dict(parent=owner.name,component=occ.component.name,depth=len(path),name=body.name,
                           sourceRootIndex=attribute(owner,"frcSource","sourceRootIndex"))
                if not public_occurrence(row):
                    continue
                solid = registered_solid(occ,body)
                faces = tessellate(solid)
                tag = motion+"-"+role
                row["key"] = str(len(inventory))
                keys[tag].append(row["key"])
                inventory.append(row)
                groups[tag].extend(faces)
        if supplier_solids != 1092 or not groups:
            raise ValueError(f"Unexpected display or supplier inventory ({supplier_solids})")
        output.mkdir(exist_ok=False)
        meshes=[]
        for tag,triangles in groups.items():
            count = len(triangles)//50
            raw = bytes(80)+struct.pack("<I",count)+triangles
            path = output/(tag+".stl")
            path.write_bytes(raw)
            role = tag.split("-",1)[1]
            meshes.append(dict(name=tag,file=path.name,sourceBodyKeys=keys[tag],triangles=count,
                               sha256=hashlib.sha256(raw).hexdigest(),rgba=[c/255 for c in PALETTE[role]]+[1]))
        manifest=dict(schema="fusion-showcase-meshes/3",sourceFormat="F3D",sourceSha256=expected,units="mm",
                      frame="robot-floor-XYZ",bodyInventory=inventory,meshes=meshes,excludedSupplierSolids=supplier_solids)
        (output/"manifest.json").write_text(json.dumps(manifest))
        (output/"wheel-bounds.json").write_text(json.dumps(wheels))
    finally:
        doc.close(False)
        if previous and previous.isValid:
            previous.activate()
