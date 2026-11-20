"""Restore the source-registered, project-made electrical deck and breaker supports."""
import json

import adsk.core as core
import adsk.fusion as fusion
from physical_bumper import ROOT, R7, design, dimensions, install_part, native, registered_solid, sha
from stage_climber import cut,rod

SUPPORT='R9 electrical supports'
SPEC=ROOT/'cad/assembly-readiness/electrical-source.json'
WASHER='JTD FW0010-375031-SS'


def extract(source,folder):
    if sha(source)!=R7:raise ValueError('Electrical source archive mismatch')
    expected=json.loads(SPEC.read_bytes())
    folder.mkdir(parents=True,exist_ok=True)
    mgr=fusion.TemporaryBRepManager.get()
    prior=core.Application.get().activeDocument
    doc=native(source)
    try:
        found={}
        for occ in design(doc).rootComponent.allOccurrences:
            name=occ.component.name
            if name not in expected:continue
            if occ.bRepBodies.count!=1 or name in found:raise ValueError(name)
            solid=registered_solid(occ,occ.bRepBodies.item(0))
            if abs(solid.volume-expected[name]['volumeCm3'])>.003 or max(abs(a-b) for a,b in zip(dimensions(solid),expected[name]['boundsMm']))>.03:
                raise ValueError('Changed support in registered archive '+name)
            file=folder/(name+'.smt')
            if not mgr.exportToFile([solid],str(file)):raise ValueError(name)
            found[name]=sha(file)
        if set(found)!=set(expected):raise ValueError('Incomplete electrical support transfer')
        (folder/'parts.json').write_text(json.dumps(found,indent=2))
    finally:
        doc.close(False)
        if prior and prior.isValid:prior.activate()


def install(d,chassis,lib,folder):
    mgr=fusion.TemporaryBRepManager.get()
    expected=json.loads(SPEC.read_bytes())
    items=[]
    al=lib.itemByName('Aluminum 6061')
    steel=lib.itemByName('Steel, Alloy')
    color=d.appearances.itemByName('R7 structure')
    registry=json.loads((folder/'parts.json').read_bytes())
    if set(registry)!=set(expected):raise ValueError('Electrical support inventory changed')
    for name,digest in registry.items():
        p=folder/(name+'.smt')
        if sha(p)!=digest:raise ValueError(name)
        b=mgr.createFromFile(str(p)).item(0)
        if abs(b.volume-expected[name]['volumeCm3'])>.003:raise ValueError(name)
        if name.startswith('breaker-'):
            head_lift=.031*2.54 if name.startswith('breaker-mount-bolt') and name.endswith(('head','shaft')) else 0.
            matrix=core.Matrix3D.create()
            matrix.translation=core.Vector3D.create(0,1.2,head_lift)
            if not mgr.transform(b,matrix):raise ValueError(name)
        items.append((name,b,steel if '-bolt-' in name else al,color))
    for x,y in ((-308.575,-188.97),(-251.425,-161.03)):
        y+=12.
        z=258.83148403491677
        a=cut(rod((x,y,z),(x,y,z+.031*25.4),.375*25.4),rod((x,y,z-.1),(x,y,z+1),.196*25.4))
        items.append((f'{WASHER} at {x}',a,lib.itemByName('Stainless Steel') or steel,color))
    install_part(d,chassis,SUPPORT,items)


def inspect(d,step):
    expected=json.loads((ROOT/'cad/electrical/breaker-mount-evidence.json').read_bytes())
    if sha(step)!=expected['stepSha256']:raise ValueError('Unexpected Eaton source')
    app=core.Application.get()
    prior=app.activeDocument
    mgr=fusion.TemporaryBRepManager.get()
    def common(a,b):
        t=mgr.copy(a)
        if not mgr.booleanOperation(t,b,fusion.BooleanTypes.IntersectionBooleanType):raise ValueError('Breaker intersection')
        return t.volume*1000 if t.isValid and t.lumps.count else 0.
    def seat(a,b,dz):
        m=core.Matrix3D.create()
        m.translation=core.Vector3D.create(0,0,dz/10)
        t=mgr.copy(a)
        mgr.transform(t,m)
        return (common(t,b)-common(a,b))/abs(dz)
    parts={}
    for o in d.rootComponent.allOccurrences:
        for b in o.bRepBodies:
            s=registered_solid(o,b)
            parts[b.name]=(s,dimensions(s))
    doc=app.importManager.importToNewDocument(app.importManager.createSTEPImportOptions(str(step)))
    try:
        root=design(doc).rootComponent
        solids=[mgr.copy(b) for b in root.bRepBodies]+[registered_solid(o,b) for o in root.allOccurrences for b in o.bRepBodies]
        offset=parts['R7 149 main-breaker'][1][1]+199.13
        offset=round(offset)
        if offset not in (0,12):raise ValueError('Unexpected breaker shift')
        m=core.Matrix3D.create()
        m.setWithCoordinateSystem(core.Point3D.create(-28,-17.5+offset/10,(expected['zShiftMm']+40)/10),core.Vector3D.create(1,0,0),
            core.Vector3D.create(0,0,1),core.Vector3D.create(0,-1,0))
        def box_union(bodies):
            boxes=[dimensions(s) for s in bodies]
            return [min(b[i] for b in boxes) for i in range(3)]+[max(b[i] for b in boxes) for i in range(3,6)]
        source_bounds=box_union(solids)
        solid_count=sum(int(s.isSolid) for s in solids)
        for s in solids:
            if not mgr.transform(s,m):raise ValueError('CB185 placement')
        pedestal=parts['breaker-pedestal'][0]
        landed=sum(seat(s,pedestal,-.01) for s in solids)
        contacts=[]
        for x,y in expected['worldHoleCentersXY']:
            y+=offset
            washer=parts[f'{WASHER} at {x}'][0]
            collisions=sum(common(washer,s) for s in solids)
            area=sum(seat(washer,s,-.01) for s in solids)
            contacts.append(dict(atMm=[x,y],washerIntersectionMm3=collisions,washerLandMm2=area))
        return dict(sku='CB185-120',supplierStepSha256=sha(step),
            sourceSolidCount=solid_count,sourceBodyCount=len(solids),sourceBoundsMm=source_bounds,installedBoundsMm=box_union(solids),
            volumeProbeStatus='rejected for open shells' if solid_count<len(solids) else 'solid probe',
            volumeProbePedestalAreaMm2=landed,volumeProbeWashers=contacts,
            limit='Terminal lug, insulation boot, hand access, reset actuation and harness are outside the source model.')
    finally:
        doc.close(False)
        if prior and prior.isValid:prior.activate()
