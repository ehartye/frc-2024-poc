"""Restore the twelve physical bumper cores to a copy of the registered R8 assembly."""
import hashlib
import json
from pathlib import Path
import sys

import adsk.core as core
import adsk.fusion as fusion

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts/showcase'))
from export_swerve import dimensions, registered_solid

R8='26a5335e66c05dec8918b767dd27a4cb62421c1d88450102e6c66d1a4914c612'
FOAM='PE closed-cell foam, AndyMark am-5580 (nominal 2 lb per ft3)'
SLIDE='TIVAR 1000 natural UHMW-PE (mass model)'
R7='0db7b7df1ad6ecfae4f7185c6a5c36279711d8db643bfb5ac6a9031034c50517'
CORE='R9 removable bumper cores'
COVER='R9 bumper cover envelope'
LIB='Fusion Material Library'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def native(path):
    app=core.Application.get()
    return app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(path)))


def design(doc):
    return fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))


def make_box(lo,hi):
    p=core.Point3D.create(*[(a+b)/20 for a,b in zip(lo,hi)])
    vec=core.Vector3D.create
    obb=core.OrientedBoundingBox3D.create(p,vec(1,0,0),vec(0,1,0),*[(b-a)/10 for a,b in zip(lo,hi)])
    return fusion.TemporaryBRepManager.get().createBox(obb)


def shell():
    from physical_front import SHIFT, REAR_SHIFT
    # A rectangular outer and return skin. Corner folds, seams and pre-tension
    # are excluded. A mass from this simplified skin does not weigh the cloth.
    outer=make_box([-443.587+REAR_SHIFT,-439.65,58],[478.512+SHIFT,439.65,186])
    inner=make_box([-443.087+REAR_SHIFT,-439.15,58.5],[478.012+SHIFT,439.15,185.5])
    open_center=make_box([-359.537+REAR_SHIFT,-355.6,56],[394.462+SHIFT,355.6,188])
    manager=fusion.TemporaryBRepManager.get()
    for body in (inner,open_center):
        if not manager.booleanOperation(outer,body,fusion.BooleanTypes.DifferenceBooleanType):
            raise ValueError('Fabric skin cut failed')
    return outer


def cover_hardware(material, appearance):
    """T50 506 staple envelopes; the leg tips and wire section are simplified."""
    from physical_front import SHIFT, REAR_SHIFT, SIDE_LENGTH
    items = []
    manager = fusion.TemporaryBRepManager.get()
    for side, normal, mid, length in (
        ('Front', 0, 403.987+SHIFT, 711.2), ('Rear', 0, -369.062+REAR_SHIFT, 711.2),
        ('Left', 1, 365.125, SIDE_LENGTH), ('Right', 1, -365.125, SIDE_LENGTH),
    ):
        for i in range(9):
            along = (i/8-.5)*(length-60)+(17.4625+(SHIFT+REAR_SHIFT)/2 if normal==1 else 0)
            def box(u0, u1, z0, z1):
                limits = [(mid-.5, mid+.5), (along+u0, along+u1)]
                if normal==1:
                    limits.reverse()
                return make_box([limits[0][0],limits[1][0],z0],
                                [limits[0][1],limits[1][1],z1])
            for edge, outer, sign in [('upper',186,1), ('lower',58,-1)]:
                def element(u0, u1, a, b):
                    lo,hi=sorted([outer+sign*a,outer+sign*b])
                    return box(u0,u1,lo,hi)
                shape = element(-4.7625,4.7625,0,.6)
                for lo,hi in [(-4.7625,-4.1625),(4.1625,4.7625)]:
                    if not manager.booleanOperation(shape, element(lo,hi,-9.525,0),
                                                    fusion.BooleanTypes.UnionBooleanType):
                        raise ValueError('Staple union failed')
                items.append((f'{side} {edge} cover staple {i+1}',shape,material,appearance))
    return items


def install_part(d, chassis, label, items):
    root=d.rootComponent
    occ=root.occurrences.addNewComponent(core.Matrix3D.create())
    comp=occ.component
    comp.name=label
    base=comp.features.baseFeatures.add()
    base.name='Registered bumper geometry'
    base.startEdit()
    try:
        for name,shape,_,_ in items:
            comp.bRepBodies.add(shape,base).name=name
    finally:
        base.finishEdit()
    if comp.bRepBodies.count!=len(items):
        raise ValueError('Bumper body count mismatch')
    for body,(name,_,material,appearance) in zip(comp.bRepBodies,items):
        body.name=name
        body.material=material
        body.appearance=appearance
    req=root.asBuiltJoints.createInput(occ,chassis,None)
    req.setAsRigidJointMotion()
    root.asBuiltJoints.add(req).name=label+' installed'
    return occ


def extract_cores(source, folder):
    """Export project-owned backing and foam only, in the robot coordinate system."""
    if sha(source) != R7:
        raise ValueError('Unexpected source archive')
    folder.mkdir(parents=True, exist_ok=True)
    app = core.Application.get()
    prior = app.activeDocument
    doc = native(source)
    try:
        manager = fusion.TemporaryBRepManager.get()
        expected = json.loads((ROOT/'cad/assembly-readiness/bumper-core-volumes.json').read_bytes())
        catalog = []
        for occ in design(doc).rootComponent.allOccurrences:
            label = occ.component.name
            if label not in expected:
                continue
            if occ.bRepBodies.count != 1:
                raise ValueError(label)
            solid = registered_solid(occ, occ.bRepBodies.item(0))
            if abs(expected[label]-solid.volume) > .001:
                raise ValueError('Changed source solid '+label)
            file = folder/(label+'.smt')
            if not manager.exportToFile([solid], str(file)):
                raise ValueError('Transfer failed '+label)
            catalog.append(dict(part=label, volumeCm3=solid.volume, sha256=sha(file)))
        if sorted(row['part'] for row in catalog) != sorted(expected):
            raise ValueError('Incomplete physical core set')
        (folder/'parts.json').write_text(json.dumps(catalog, indent=2)+'\n', encoding='utf8')
    finally:
        doc.close(False)
        if prior and prior.isValid:
            prior.activate()


def build(folder,output,material_map,electrical_folder,plan_output):
    source=ROOT/'cad/student/r8-stage-climber.f3d'
    if sha(source)!=R8 or output.exists(): raise ValueError('Input identity or destination')
    app=core.Application.get()
    prior=app.activeDocument
    doc=native(source)
    try:
        d=design(doc)
        top=d.rootComponent
        chassis=next(o for o in top.occurrences if o.component.name=='R7 chassis')
        old=chassis.component.bRepBodies.itemByName('R7 503 Continuous covered bumper envelope')
        if not old or abs(old.volume-35692.483)>.02: raise ValueError('Unexpected bumper source')
        comp=chassis.component
        comp.features.removeFeatures.add(old).name='Remove full-volume visualization bumper'
        lib=app.materialLibraries.itemByName(LIB).materials
        def polymer(base_name, name, density):
            material = d.materials.addByCopy(lib.itemByName(base_name), name)
            for key in ('structural_Density', 'thermal_Density'):
                prop = material.materialProperties.itemById(key)
                if prop:
                    prop.value = density
            return material

        foam = polymer('Polyethylene, Low Density', FOAM, 32.03692675)
        slide = polymer('UHMW, White', SLIDE, 930)
        plywood=lib.itemByName('Plywood, Finish')
        manager=fusion.TemporaryBRepManager.get()
        items=[]
        catalog=json.loads((folder/'parts.json').read_bytes())
        for row in catalog:
            path=folder/(row['part']+'.smt')
            if sha(path)!=row['sha256']: raise ValueError('Transferred part hash mismatch')
            shape=manager.createFromFile(str(path)).item(0)
            if abs(shape.volume-row['volumeCm3'])>.001: raise ValueError('Transfer volume mismatch')
            from physical_front import reshape_core
            shape = reshape_core(row['part'],shape)
            items.append((row['part'],shape,foam if 'noodle' in row['part'] else plywood,d.appearances.itemByName('R7 structure')))
        if len(items)!=12: raise ValueError('Physical core set incomplete')
        install_part(d,chassis,CORE,items)
        skin = [('Sewn nylon skin, nominal envelope',shell(),lib.itemByName('Nylon 6/6'),
                 d.appearances.itemByName('R7 bumper'))]
        install_part(d,chassis,COVER,skin+cover_hardware(lib.itemByName('Steel'),
                     d.appearances.itemByName('R7 structure')))
        from physical_electrical import install as install_electrical
        install_electrical(d,chassis,lib,electrical_folder)
        from physical_harness import install as install_harness
        install_harness(d,chassis,lib)
        from physical_mount import install as install_mount
        install_mount(d,chassis,lib)
        from physical_front import install as install_front
        install_front(d,chassis,lib)
        from physical_lift import install as install_lift
        install_lift(d,chassis,lib)
        selected=json.loads(material_map.read_bytes())
        count=0
        for occ in top.occurrences:
            for body in occ.component.bRepBodies:
                key=occ.component.name+' / '+body.name
                if key not in selected: continue
                old_color=body.appearance
                body.material=slide if selected[key]==SLIDE else lib.itemByName(selected[key])
                body.appearance=old_color
                count+=1
        if count!=len(selected): raise ValueError('Material assignment missed one or more bodies')
        sys.path.insert(0,str(ROOT/'cad/assembly-readiness'))
        from article_ids import identify
        planning=[]
        for occ in top.allOccurrences:
            for body in occ.component.bRepBodies:
                identity,owner,_=identify(occ.component.name,body.name)
                body.attributes.add('assemblyEvidence','identity',identity)
                body.attributes.add('assemblyEvidence','owner',owner or 'display-only')
                planning.append(dict(occurrence=occ.fullPathName,body=body.name,identity=identity,instance=owner,
                    selection=(body.material or occ.component.material).name,materialId=(body.material or occ.component.material).id,designVolumeCm3=body.volume))
        plan_output.write_text(json.dumps(planning,indent=1)+'\n',encoding='utf8')
        if not d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(str(output))):
            raise ValueError('R9 archive export failed')
    finally:
        doc.close(False)
        if prior and prior.isValid: prior.activate()


def inspect(path,catalog,expected_map,out):
    app=core.Application.get()
    prior=app.activeDocument
    doc=native(path)
    try:
        d=design(doc)
        root=d.rootComponent
        mapping=json.loads(expected_map.read_bytes())
        seen=set()
        rows=[]
        for occ in root.allOccurrences:
            for body in occ.bRepBodies:
                key=occ.component.name+' / '+body.name
                material=body.material.name if body.material else occ.component.material.name
                if key in mapping:
                    if material!=mapping[key]: raise ValueError('Reopened physical material mismatch '+key)
                    seen.add(key)
                original=body.nativeObject or body
                props=original.getPhysicalProperties(fusion.CalculationAccuracy.VeryHighCalculationAccuracy)
                shape=registered_solid(occ,body)
                accurate_volume=shape.volume
                physical_material=original.material or occ.component.material
                density_prop=physical_material.materialProperties.itemById('structural_Density')
                density=density_prop.value if density_prop else props.density*1e6
                corrected_mass=accurate_volume*density/1e6
                if abs(props.volume-accurate_volume)<max(.00002,accurate_volume*.00001) and abs(props.mass-corrected_mass)<max(.000002,corrected_mass*.00001):
                    mass=props.mass
                    method='very-high native body'
                else:
                    mass=corrected_mass
                    method='registered solid volume times physical material density'

                attrs=(body.nativeObject or body).attributes
                identity=attrs.itemByName('assemblyEvidence','identity').value
                owner=attrs.itemByName('assemblyEvidence','owner').value
                rows.append(dict(identity=identity,instance=None if owner=='display-only' else owner,occurrence=occ.fullPathName,component=occ.component.name,body=body.name,material=material,
                    materialId=(body.material or occ.component.material).id,
                    massKg=mass,volumeCm3=accurate_volume,nativeVeryHighKg=props.mass,nativeVeryHighVolumeCm3=props.volume,
                    lowProxyVolumeCm3=body.physicalProperties.volume,densityKgM3=density,massMethod=method,boundsMm=dimensions(shape)))
        if seen!=set(mapping) or any('Continuous covered bumper envelope' in r['body'] for r in rows):
            raise ValueError('Incomplete replacement')
        parts={r['body']:r for r in rows if r['component']==CORE}
        for source in json.loads((catalog/'parts.json').read_bytes()):
            r=parts[source['part']]
            from physical_front import added_volume_cm3
            if abs(source['volumeCm3']+added_volume_cm3(source['part'])-r['volumeCm3'])>.002: raise ValueError(r['body'])
            if 'noodle' in r['body'] and abs(r['massKg']/r['volumeCm3']*1e6-32.03693)>.02: raise ValueError('Foam density mismatch')
        result=dict(archiveSha256=sha(path),parentSha256=R8,totalNativeKg=root.physicalProperties.mass,
            components=root.allOccurrences.count,joints=root.asBuiltJoints.count,rows=rows,registeredBodyKg=sum(r['massKg'] for r in rows))
        out.write_text(json.dumps(result,indent=1)+'\n',encoding='utf8',newline='\n')
    finally:
        doc.close(False)
        if prior and prior.isValid: prior.activate()
