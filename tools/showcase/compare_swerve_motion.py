"""Check both directions of solid difference for the registered moving Fusion bodies."""
import hashlib
import adsk.core
import adsk.fusion
from export_swerve import attribute, registered_solid, dimensions
from public_parts import public_occurrence


def snapshot(archive, expected):
    if hashlib.sha256(archive.read_bytes()).hexdigest() != expected:
        raise ValueError("Motion-check archive differs from receipt")
    app=adsk.core.Application.get()
    previous=app.activeDocument
    doc=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(archive)))
    try:
        doc.name="Team 1780 - Temporary moving-solid registration"
        root=adsk.fusion.Design.cast(doc.products.itemByProductType("DesignProductType")).rootComponent
        owners={o.name:o.component for o in root.occurrences}
        result={}
        for occ in root.allOccurrences:
            path=occ.fullPathName.split("+")
            owner=owners[path[0]]
            group=attribute(owner,"frcSource","motionGroup")
            if not occ.isVisible or group not in ("cradle","carriage") or attribute(owner,"frcDisplay","role")=="reserved-space":
                continue
            for body in occ.bRepBodies:
                row=dict(parent=owner.name,depth=len(path),component=occ.component.name,name=body.name,
                         sourceRootIndex=attribute(owner,"frcSource","sourceRootIndex"))
                if body.isVisible and body.isSolid and public_occurrence(row):
                    key=(occ.fullPathName,body.name,group)
                    if key in result:
                        raise ValueError("Ambiguous moving part")
                    result[key]=registered_solid(occ,body)
        return result
    finally:
        doc.close(False)
        if previous and previous.isValid:
            previous.activate()


def compare(display, plant, display_sha, plant_sha):
    a,b=snapshot(display,display_sha),snapshot(plant,plant_sha)
    if a.keys()!=b.keys() or len(a)!=188:
        raise ValueError("Different moving parts or motion-group assignments")
    manager=adsk.fusion.TemporaryBRepManager.get()
    overlap_type=adsk.fusion.BooleanTypes.IntersectionBooleanType
    worst=0.
    for key,x in a.items():
        y=b[key]
        if dimensions(x)!=dimensions(y):
            raise ValueError("Different world-space bounds for "+str(key))
        # The symmetric-difference volume is |A| + |B| - 2 |A intersect B|.
        common=manager.copy(x)
        if not manager.booleanOperation(common,manager.copy(y),overlap_type):
            raise ValueError("Native intersection failed for "+str(key))
        delta=abs((x.volume+y.volume-2*common.volume)*1000)
        worst=max(worst,delta)
        if delta > .001:
            raise ValueError(f"Moving articles differ by {delta} mm3: {key}")
    return dict(displayNative=display_sha,plantNative=plant_sha,registeredBodyCount=len(a),
                maximumSymmetricDifferenceMm3=worst,volumeToleranceMm3=.001)
