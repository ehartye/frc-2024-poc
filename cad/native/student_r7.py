"""Create and verify the releasable R7 Fusion assembly in an authorized Fusion session."""
import hashlib
import json
import math
from pathlib import Path
import sys

import adsk.core as core
import adsk.fusion as fusion


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/"scripts/showcase"))
from export_swerve import PALETTE, attribute, dimensions, registered_solid
from public_parts import public_occurrence
from swerve_modules import install_module

BUDGET = json.loads((ROOT/"cad/drivetrain/r7-floor-mass.json").read_bytes())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def extract(archive, manifest, folder):
    if sha(archive) != BUDGET["archive"] or sha(manifest) != BUDGET["inventorySha256"]:
        raise ValueError("Unregistered R7 archive or display inventory")
    catalog = json.loads(manifest.read_bytes())
    expected = catalog["bodyInventory"]
    if len(expected) != 612 or catalog["excludedSupplierSolids"] != 1092:
        raise ValueError("Unexpected R7 source inventory")
    role = {k:m["name"] for m in catalog["meshes"] for k in m["sourceBodyKeys"]}
    app = core.Application.get()
    prior = app.activeDocument
    folder.mkdir(exist_ok=False)
    doc = app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(archive)))
    try:
        d = fusion.Design.cast(doc.products.itemByProductType("DesignProductType"))
        top = {o.name:o.component for o in d.rootComponent.occurrences}
        rows = []
        supplier = 0
        for occ in d.rootComponent.allOccurrences:
            path = occ.fullPathName.split("+")
            owner = top[path[0]]
            if owner.name.startswith("MAXSwerve"):
                supplier += sum(int(body.isSolid) for body in occ.bRepBodies)
                continue
            if not occ.isVisible or attribute(owner,"frcDisplay","role") == "reserved-space":
                continue
            for body in occ.bRepBodies:
                row = dict(parent=owner.name, component=occ.component.name, depth=len(path), name=body.name,
                           sourceRootIndex=attribute(owner,"frcSource","sourceRootIndex"),key=str(len(rows)))
                if not body.isSolid or not body.isVisible or not public_occurrence(row):
                    continue
                if row != expected[len(rows)]:
                    raise ValueError("Selected occurrence differs from reviewed display inventory")
                solid = registered_solid(occ, body)
                target = folder/(row["key"]+".smt")
                if not fusion.TemporaryBRepManager.get().exportToFile([solid],str(target)):
                    raise ValueError("Could not transfer project body "+row["key"])
                rows.append(dict(**row, group=role[row["key"]], sha256=sha(target), boundsMm=dimensions(solid),
                                 volumeMm3=solid.volume*1000))
        if len(rows) != 612 or supplier != 1092:
            raise ValueError("Incomplete R7 body accounting")
        (folder/"catalog.json").write_text(json.dumps(dict(archive=sha(archive), inventory=sha(manifest),
                                                excludedSupplierSolids=supplier, rows=rows)))
    finally:
        doc.close(False)
        if prior and prior.isValid:
            prior.activate()


def transform(xyz=(0,0,0), heading=0):
    m = core.Matrix3D.create()
    m.setToRotation(heading, core.Vector3D.create(0,0,1), core.Point3D.create(0,0,0))
    m.translation = core.Vector3D.create(*[v/10 for v in xyz])
    return m


def proxies(station):
    model = {"nodes":[{},{},{}]}
    kinds = {"rail":"box", "cheek":"box", "motor":"cylinder", "ring":"ring", "hub":"cylinder"}
    install_module(model,kinds,station,0,1,2)
    # Students can edit these original solid envelopes. The metal hub and tread share the roll joint.
    model["nodes"][2].setdefault("children",[]).append(len(model["nodes"]))
    model["nodes"].append(dict(name=station+" tire",mesh="tire",translation=[0,0,0],scale=[.0762,.0256,.0762],
                               rotation=[math.sqrt(.5),0,0,math.sqrt(.5)]))
    for i,group in enumerate(("housing","carrier","wheel")):
        for k in model["nodes"][i]["children"]:
            yield group, model["nodes"][k]


def proxy_solid(node):
    manager = fusion.TemporaryBRepManager.get()
    x, z, ny = [v*100 for v in node["translation"]]
    center = [x,-ny,z]
    sx,sz,sy = [v*100 for v in node["scale"]]
    p, v = core.Point3D.create, core.Vector3D.create
    if node["mesh"] == "box":
        return manager.createBox(core.OrientedBoundingBox3D.create(p(*center),v(1,0,0),v(0,1,0),sx,sy,sz))
    direction = (0,1,0) if "rotation" in node else (0,0,1)
    def cylinder(radius):
        a,b = [p(*[c + sign*sz*n/2 for c,n in zip(center,direction)]) for sign in (-1,1)]
        return manager.createCylinderOrCone(a,radius,b,radius)
    solid = cylinder(sx/2)
    if node["mesh"] in ("ring","tire"):
        bore = .40*sx if node["mesh"] == "ring" else 2.10
        if not manager.booleanOperation(solid,cylinder(bore),fusion.BooleanTypes.DifferenceBooleanType):
            raise ValueError("Failed to cut proxy bore")
    return solid


def build(folder, output):
    catalog = json.loads((folder/"catalog.json").read_bytes())
    if catalog["inventory"] != BUDGET["inventorySha256"] or catalog["archive"] != BUDGET["archive"]:
        raise ValueError("Unregistered source extraction")
    app = core.Application.get()
    previous = app.activeDocument
    doc = app.documents.add(core.DocumentTypes.FusionDesignDocumentType)
    manager = fusion.TemporaryBRepManager.get()
    layout = json.loads((ROOT/"cad/drivetrain/maxswerve-layout.json").read_bytes())
    try:
        doc.name = "R7 MAXSwerve student assembly"
        d = fusion.Design.cast(doc.products.itemByProductType("DesignProductType"))
        d.designIntent = fusion.DesignIntentTypes.HybridDesignIntentType
        d.designType = fusion.DesignTypes.ParametricDesignType
        root = d.rootComponent
        names = {"fixed":"R7 chassis", "carriage":"R7 carriage", "cradle":"R7 cradle"}
        poses = {"fixed":transform(), "carriage":transform(), "cradle":transform((178,0,25.4))}
        source = {key:[] for key in names}
        for row in catalog["rows"]:
            group,role = row["group"].split("-",1)
            source[group].append((row,role))
        for station, reg in layout["stations"].items():
            ax,ay,_ = layout["articulation"]["nativeJointTest"]["positiveAxes"][station]["roll"]
            for suffix,node in proxies(station):
                group = station+" "+suffix
                names[group] = group
                poses[group] = transform(reg["wheelCenterMm"], math.atan2(ax,-ay))
                source.setdefault(group,[]).append((node, "tire" if node["mesh"]=="tire" else "module"))
        colors = dict(**PALETTE,module=(105,112,119),tire=(45,46,47))
        colors_used = {}
        groups = {}
        for group, name in names.items():
            occ = root.occurrences.addNewComponent(poses[group])
            occ.component.name = name
            groups[group]=occ
            comp = occ.component
            base = comp.features.baseFeatures.add()
            base.name = "Registered authored solids" if group in ("fixed","carriage","cradle") else "Original module proxy solids"
            labels = []
            base.startEdit()
            try:
                inverse = poses[group].copy()
                inverse.invert()
                for item,role in source[group]:
                    if "key" in item:
                        path = folder/(item["key"]+".smt")
                        if sha(path) != item["sha256"]:
                            raise ValueError("Changed transferred solid")
                        imported = manager.createFromFile(str(path))
                        if imported.count != 1:
                            raise ValueError("Unexpected transferred body count")
                        solid = imported.item(0)
                        if not manager.transform(solid,inverse):
                            raise ValueError("Could not restore local coordinates")
                        label = "R7 "+item["key"]+" "+item["name"]
                    else:
                        solid = proxy_solid(item)
                        label = item["name"]
                    created = comp.bRepBodies.add(solid,base)
                    created.name = label
                    if role not in colors_used:
                        library=app.materialLibraries.itemByName("Fusion Appearance Library")
                        ap=d.appearances.addByCopy(library.appearances.itemByName("Plastic - Matte (Gray)"),"R7 "+role)
                        for prop_id in ("opaque_albedo","surface_albedo"):
                            prop=core.ColorProperty.cast(ap.appearanceProperties.itemById(prop_id))
                            if prop: prop.value=core.Color.create(*colors[role],255)
                        colors_used[role]=ap
                    created.appearance=colors_used[role]
                    labels.append((label,role))
            finally:
                base.finishEdit()
            if comp.bRepBodies.count != len(labels):
                raise ValueError("Base feature changed body count")
            for body,(label,role) in zip(comp.bRepBodies,labels):
                body.name=label
                body.appearance=colors_used[role]
        groups["fixed"].isGrounded = True
        def joint(title,parent,child,kind,axis=None):
            moving = groups[child]
            pt = moving.component.originConstructionPoint.createForAssemblyContext(moving)
            geom = fusion.JointGeometry.createByPoint(pt) if kind != "rigid" else None
            first,second = (groups[parent],moving) if kind == "revolute" else (moving,groups[parent])
            req=root.asBuiltJoints.createInput(first,second,geom)
            if kind=="rigid":
                req.setAsRigidJointMotion()
            elif kind=="slider":
                req.setAsSliderJointMotion(axis)
            else:
                req.setAsRevoluteJointMotion(axis)
            result=root.asBuiltJoints.add(req)
            result.name=title
            return result
        axes = fusion.JointDirections
        lift=joint("Lift", "fixed","carriage","slider",axes.ZAxisJointDirection)
        pitch=joint("Pitch", "carriage","cradle","revolute",axes.YAxisJointDirection)
        for limit,maximum in ((lift.jointMotion.slideLimits,44.94),(pitch.jointMotion.rotationLimits,math.radians(60))):
            limit.minimumValue,limit.maximumValue = 0.,maximum
            limit.isMinimumValueEnabled = limit.isMaximumValueEnabled = True
        for station in layout["stations"]:
            joint(station+" mount","fixed",station+" housing","rigid")
            joint(station+" azimuth",station+" housing",station+" carrier","revolute",axes.ZAxisJointDirection)
            joint(station+" roll",station+" carrier",station+" wheel","revolute",axes.YAxisJointDirection)
        if root.asBuiltJoints.count != 14 or root.occurrences.count != 15:
            raise ValueError("Incomplete R7 component or joint graph")
        output.parent.mkdir(parents=True,exist_ok=True)
        if not d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(str(output))):
            raise ValueError("Student archive export failed")
    finally:
        doc.close(False)
        if previous and previous.isValid:
            previous.activate()


def relative(child, parent):
    # Fusion matrices are row-major, with translations in native centimetres.
    a = parent.transform2.copy()
    a.invert()
    left, right = a.asArray(), child.transform2.asArray()
    return [sum(left[i*4+k]*right[k*4+j] for k in range(4)) for i in range(4) for j in range(4)]


def verify(file, catalog_path, receipt, project_source):
    app = core.Application.get()
    previous = app.activeDocument
    doc = app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(file)))
    try:
        root = fusion.Design.cast(doc.products.itemByProductType("DesignProductType")).rootComponent
        occurrences = {o.component.name:o for o in root.occurrences}
        if len(occurrences) != 15 or root.allOccurrences.count != 15 or root.bRepBodies.count != 0:
            raise ValueError("Unexpected student assembly tree")
        catalog=json.loads(catalog_path.read_bytes())
        rows={v["key"]:v for v in catalog["rows"]}
        seen=set()
        components={}
        max_bounds=max_volume=0.
        for name,occ in occurrences.items():
            features=occ.component.features
            if features.baseFeatures.count != 1:
                raise ValueError("Expected one direct-edit group per registered component")
            components[name]=dict(bodies=occ.bRepBodies.count,editableFeatures=0,
                baseFeatures=features.baseFeatures.count,
                representation="Registered authored BRep solids" if name.startswith("R7 ") else "Original dimensioned module proxy")
            for body in occ.bRepBodies:
                if not body.isSolid:
                    raise ValueError("Unexpected surface body")
                if not name.startswith("R7 "):
                    continue
                label=body.name.split(" ",2)
                if len(label)!=3 or label[0]!="R7":
                    raise ValueError("Unidentified authored body "+body.name)
                key=label[1]
                row=rows[key]
                if key in seen or label[-1]!=row["name"] or not body.isVisible:
                    raise ValueError("Missing or substituted authored body "+key)
                seen.add(key)
                solid=registered_solid(occ,body)
                max_bounds=max(max_bounds,max(abs(a-b) for a,b in zip(dimensions(solid),row["boundsMm"])))
                max_volume=max(max_volume,abs(solid.volume*1000-row["volumeMm3"]))
        if seen!=set(rows) or max_bounds>.01 or max_volume>.05:
            raise ValueError(f"R7 home pose changed: {len(seen)} bodies, {max_bounds} mm, {max_volume} mm3")
        layout=json.loads((ROOT/"cad/drivetrain/maxswerve-layout.json").read_bytes())
        axes=layout["articulation"]["nativeJointTest"]["positiveAxes"]
        joints={}
        home={name:occ.transform2.asArray() for name,occ in occurrences.items()}
        for j in root.asBuiltJoints:
            a,b=j.occurrenceOne,j.occurrenceTwo
            item=dict(occurrenceOne=a.component.name,occurrenceTwo=b.component.name)
            if j.name.endswith(" mount"):
                if not fusion.RigidJointMotion.cast(j.jointMotion):
                    raise ValueError("Rigid housing mount required")
                item["type"]="rigid"
            else:
                slide=j.name=="Lift"
                motion=(fusion.SliderJointMotion if slide else fusion.RevoluteJointMotion).cast(j.jointMotion)
                if not motion: raise ValueError("Unexpected joint motion "+j.name)
                key="slideValue" if slide else "rotationValue"
                samples=[]
                for value in ((0,12,0) if slide else (0,math.radians(25),math.radians(42 if j.name=="Pitch" else -18),0)):
                    setattr(motion,key,value)
                    app.activeViewport.refresh()
                    attained=getattr(motion,key)
                    if abs(attained-value)>1e-6:
                        raise ValueError("Could not attain "+j.name)
                    if not slide and abs(value)>1e-5:
                        moving = b  # Revolute joints are ordered fixed/parent first.
                        home_pose=home[moving.component.name]
                        actual=moving.transform2.asArray()
                        # World rotation from home to current. The axial skew vector
                        # equals 2 sin(angle) times the signed world-space joint axis.
                        rotation=[[sum(actual[4*i+k]*home_pose[4*j+k] for k in range(3))
                                   for j in range(3)] for i in range(3)]
                        direction=[(rotation[2][1]-rotation[1][2])/(2*math.sin(value)),
                                   (rotation[0][2]-rotation[2][0])/(2*math.sin(value)),
                                   (rotation[1][0]-rotation[0][1])/(2*math.sin(value))]
                        if j.name=="Pitch":
                            expected=[0,-1,0]
                        else:
                            station,kind=j.name.rsplit(" ",1)
                            expected=axes[station]["steer" if kind=="azimuth" else "roll"]
                        if math.dist(direction,expected)>.00001:
                            raise ValueError(f"{j.name} actual positive axis {direction} differs from {expected}")
                        item["positiveWorldAxis"]=direction
                    samples.append(dict(position=attained*10 if slide else math.degrees(attained),
                                        relativeTransform=relative(a,b)))
                gap=max(abs(x-y) for x,y in zip(samples[0]["relativeTransform"],samples[1]["relativeTransform"]))
                if gap<.01 or max(abs(x-y) for x,y in zip(samples[0]["relativeTransform"],samples[-1]["relativeTransform"]))>.001:
                    raise ValueError("Immobile or non-returning joint "+j.name)
                item.update(type="slider" if slide else "revolute",positionUnit="mm" if slide else "deg",samples=samples)
                for n,occ in occurrences.items():
                    if max(abs(x-y) for x,y in zip(home[n],occ.transform2.asArray()))>.001:
                        raise ValueError("Assembly did not return home after "+j.name)
            joints[j.name]=item
        # Exercise each wheel with the carrier already steered, then return both axes.
        combined={}
        for station in layout["stations"]:
            steer=root.asBuiltJoints.itemByName(station+" azimuth").jointMotion
            roll=root.asBuiltJoints.itemByName(station+" roll").jointMotion
            tire=occurrences[station+" wheel"]
            housing=occurrences[station+" housing"]
            steer.rotationValue=math.radians(31)
            middle=relative(tire,housing)
            roll.rotationValue=math.radians(-24)
            final=relative(tire,housing)
            if max(abs(a-b) for a,b in zip(middle,final))<.1:
                raise ValueError("Wheel did not roll inside its turned carrier")
            combined[station]=dict(steerDeg=math.degrees(steer.rotationValue),rollDeg=math.degrees(roll.rotationValue),
                                   wheelRelativeToHousing=final)
            roll.rotationValue=steer.rotationValue=0.
        if len(joints)!=14 or sum(v["type"]!="rigid" for v in joints.values())!=10:
            raise ValueError("Incomplete reopened joint observations")
        data=dict(source=project_source,sha256=sha(file),opened=True,components=components,joints=joints,
                  privateSourceSha256=BUDGET["archive"],sourceManifestSha256=catalog["inventory"],
                  authoredBodies=len(seen),combinedSteerAndRoll=combined,excludedSupplierSolids=1092,maximumBoundsDifferenceMm=max_bounds,
                  maximumAbsoluteVolumeDifferenceMm3=max_volume,transformTranslationUnit="cm")
        receipt.write_text(json.dumps(data,indent=2,allow_nan=False)+"\n",encoding="utf8",newline="\n")
    finally:
        doc.close(False)
        if previous and previous.isValid: previous.activate()
