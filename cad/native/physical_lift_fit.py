"""Inspect the parked rebuilt upper lift transmission against the native robot."""
import math

import adsk.core as core
import adsk.fusion as fusion

from physical_bumper import dimensions, registered_solid, sha
from physical_lift import ASSEMBLY
from physical_front import FRONT, REAR

SUPPLIER_SHA = '948f553b2a921c76f479abb416424c5fd45ad72a5630798df5c1159ff8e7ad90'


def inspect(d, supplier):
    if sha(supplier)!=SUPPLIER_SHA:
        raise ValueError('Unexpected REV gearbox STEP')
    manager = fusion.TemporaryBRepManager.get()
    def overlap(a,b):
        result=manager.copy(a)
        if not manager.booleanOperation(result,b,fusion.BooleanTypes.IntersectionBooleanType):
            raise ValueError('Lift intersection failed')
        return result.volume*1000 if result.isValid and result.lumps.count else 0.
    def near(a,b):
        return all(min(a[i+3],b[i+3])-max(a[i],b[i])>.00001 for i in range(3))
    entries=[]
    for occ in d.rootComponent.allOccurrences:
        for b in occ.bRepBodies:
            shape=registered_solid(occ,b)
            entries.append((b.name,occ.component.name,shape,dimensions(shape)))
    lifts=[r for r in entries if r[1]==ASSEMBLY]
    if len(lifts)!=28:
        raise ValueError(f'Upper lift inventory: {len(lifts)}')
    hits=[]
    for i,(name,group,shape,bb) in enumerate(lifts):
        for other,group2,solid,other_bb in entries:
            if name==other or (group2==ASSEMBLY and name>other) or not near(bb,other_bb):
                continue
            v=overlap(shape,solid)
            if v>.01:
                hits.append(dict(first=name,second=other,mm3=round(v,4)))
    projection=[]
    for name,group,_,bb in entries:
        if group.startswith('R9 bumper') or group.startswith('R9 removable') or (
           name.startswith('R7 ') and int(name.split()[1]) in {*range(487,503),593,594,*range(598,612)}):
            continue
        out=max(-REAR-bb[0],bb[3]-FRONT,-355.6-bb[1],bb[4]-355.6)
        if out>.1:
            projection.append(dict(body=name,overhangMm=round(out,3)))
    new_bounds=[min(r[3][i] for r in lifts) for i in range(3)]+[max(r[3][i] for r in lifts) for i in range(3,6)]
    # Compare the exact supplier assembly against the installed articles in a
    # separate import. Published work retains only its source, transform and measurements.
    app=core.Application.get()
    prior=app.activeDocument
    doc=app.importManager.importToNewDocument(app.importManager.createSTEPImportOptions(str(supplier)))
    try:
        source=fusion.Design.cast(doc.products.itemByProductType('DesignProductType')).rootComponent
        records=[manager.copy(b) for b in source.bRepBodies]
        records.extend(registered_solid(o,b) for o in source.allOccurrences for b in o.bRepBodies)
        bounds=[dimensions(b) for b in records]
        ext=[min(v[i] for v in bounds) for i in range(3)]+[max(v[i] for v in bounds) for i in range(3,6)]
        if len(records)!=96 or max(abs(a-b) for a,b in zip(ext,[-31.75,-13,-25.4,31.75,50.5,25.4]))>.03:
            raise ValueError(f'Unexpected STEP units or orientation: {len(records)} {ext}')
        c=math.sqrt(.5)
        m=core.Matrix3D.create()
        m.setWithCoordinateSystem(core.Point3D.create((60-18.75*c)/10,0,(898.5-18.75*c)/10),
            core.Vector3D.create(-c,0,c),core.Vector3D.create(c,0,c),core.Vector3D.create(0,1,0))
        exact_hits={}
        for item in records:
            if not manager.transform(item,m):
                raise ValueError('STEP installation failed')
            bb=dimensions(item)
            for name,g,solid,other_bb in entries:
                if name.startswith('REV-21-2120 ') or not near(bb,other_bb): continue
                v=overlap(item,solid)
                if v>.01: exact_hits[name]=exact_hits.get(name,0.)+v
        return dict(nativeBodies=len(lifts),boundsMm=new_bounds,parkedIntersections=hits,
            perimeterScreen=sorted(projection,key=lambda v:-v['overhangMm']),
            supplier=dict(sku='REV-21-2120',sha256=SUPPLIER_SHA,rawBoundsMm=ext,
                solids=len(records),robotFromSourceCm=m.asArray(),intersectionMm3=exact_hits),
            frameRectangleMm=[-REAR,-355.6,FRONT,355.6],
            scope='Parked pose. Thread minor/major overlap is reported. Wire exits, real preload and qualification require separate checks.')
    finally:
        doc.close(False)
        if prior and prior.isValid: prior.activate()
