"""Check attained NOTE envelopes against registered native BReps in Autodesk Fusion.

Call check() from the bridge with a verified Design and private input/output paths.
The entire check uses temporary solids. Fusion geometry uses centimeters.
"""
import hashlib
import json
import math
from pathlib import Path

import adsk.core
import adsk.fusion


def torus_at(manager, x, y, z, pitch, padding):
    core = adsk.core
    origin = core.Point3D.create(0., 0., 0.)
    tube = 25.4 + padding
    ring = manager.createTorus(origin, core.Vector3D.create(0., 0., 1.), 15.24, tube/10)
    transform = core.Matrix3D.create()
    transform.setToRotation(-pitch, core.Vector3D.create(0., 1., 0.), origin)
    transform.translation = core.Vector3D.create(x/10, y/10, z/10)
    if not manager.transform(ring, transform):
        raise ValueError("Failed to place the native NOTE")
    box = ring.preciseBoundingBox
    measured = [10*value for value in box.minPoint.asArray()+box.maxPoint.asArray()]
    extent = [152.4*abs(math.cos(pitch))+tube, 152.4+tube, 152.4*abs(math.sin(pitch))+tube]
    expected = [v+sign*e for sign in (-1,1) for v,e in zip((x,y,z),extent)]
    if max(abs(a-b) for a,b in zip(measured,expected)) > .002:
        raise ValueError("Native torus placement exceeds tolerance")
    return ring, measured


def tilted_torus(manager,frame):
    core=adsk.core
    center=frame["center"]
    normal=core.Vector3D.create(*frame["normal"])
    if abs(normal.length-1.)>1e-8:raise ValueError("Nonunit sweep normal")
    tube=25.4+frame["padMm"]
    ring=manager.createTorus(core.Point3D.create(0,0,0),core.Vector3D.create(0,0,1),15.24,tube/10)
    ref=core.Vector3D.create(0,1,0) if abs(normal.y)<.9 else core.Vector3D.create(1,0,0)
    x=ref.crossProduct(normal);x.normalize();y=normal.crossProduct(x)
    transform=core.Matrix3D.create()
    transform.setWithCoordinateSystem(core.Point3D.create(*(v/10 for v in center)),x,y,normal)
    if not manager.transform(ring,transform):raise ValueError("Native placement failed")
    bb=ring.preciseBoundingBox
    measured=[v*10 for v in bb.minPoint.asArray()+bb.maxPoint.asArray()]
    extent=[tube+152.4*math.sqrt(max(0.,1-n*n)) for n in frame["normal"]]
    expected=[c+sign*e for sign in (-1,1) for c,e in zip(center,extent)]
    if max(abs(a-b) for a,b in zip(measured,expected))>.002:
        raise ValueError("Native three-dimensional torus placement exceeds tolerance")
    return ring,measured


def check(design, registration, poses, output, archive_sha256):
    data = json.loads(Path(registration).read_bytes())
    if data["sourceSha256"] != archive_sha256:
        raise ValueError("Native registration revision mismatch")
    inventory = data["inventory"]
    bodies = []
    index = 0
    for i, occurrence in enumerate(design.rootComponent.allOccurrences):
        for j, solid in enumerate(occurrence.bRepBodies):
            item = inventory[index]
            index += 1
            if item["key"] != f"body-{i+1}-{j+1}" or item["name"] != solid.name:
                raise ValueError("Native body registration mismatch")
            if not item["exclusion"] and item["role"] != "uncalibrated-compliant-tread":
                bodies.append((item,solid))
    if index != len(inventory):
        raise ValueError("Native body count mismatch")
    manager = adsk.fusion.TemporaryBRepManager.get()
    pose_bytes = Path(poses).read_bytes()
    packet_digest = hashlib.sha256(pose_bytes).hexdigest()
    with Path(output).open("x",encoding="utf-8") as stream:
        for pose in json.loads(pose_bytes):
            if "frames" in pose:
                rings={key:tilted_torus(manager,frame) for key,frame in pose["frames"].items()}
            else:
                x,y,z,p,theta,pivot = (pose[k] for k in ("x","y","z","p","theta","pivot"))
                pad = pose["padMm"]
                dx,dz = x-178.,z-pivot
                c,s = math.cos(theta),math.sin(theta)
                rings = {
                    "fixed": torus_at(manager,x,y,z,p,pad["fixed"]),
                    "carriage": torus_at(manager,x,y,25.4+dz,p,pad["carriage"]),
                    "cradle": torus_at(manager,178+c*dx+s*dz,y,25.4-s*dx+c*dz,p-theta,pad["cradle"])
                }
            hits = []
            checks = 0
            for item,solid in bodies:
                if pose["parentIds"] is not None and item["key"] not in pose["parentIds"]:
                    continue
                ring,box = rings[item["motionGroup"]]
                other = item["boundsMm"]
                if other[1] > pose["yBoundsMm"][1]+.002 or other[4] < pose["yBoundsMm"][0]-.002:
                    continue
                if any(box[k+3]+.002 < other[k] or other[k+3]+.002 < box[k] for k in range(3)):
                    continue
                common = manager.copy(ring)
                if not manager.booleanOperation(common,solid,adsk.fusion.BooleanTypes.IntersectionBooleanType):
                    hits.append(dict(id=item["key"], unresolved="intersection"))
                    continue
                checks += 1
                try:
                    volume = common.volume*1000 if common.isValid and common.faces.count else 0.
                except RuntimeError:
                    hits.append(dict(id=item["key"], unresolved="volume"))
                    continue
                if volume > .0001:
                    hits.append(dict(id=item["key"],volumeMm3=volume))
            stream.write(json.dumps(dict(**pose, checks=checks, hits=hits, archiveSha256=archive_sha256,
                                         posePacketSha256=packet_digest))+"\n")
            stream.flush()

import adsk.core,adsk.fusion,hashlib,json
def snapshot(doc):
    design=adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'));root=design.rootComponent
    occ=[];bodies=[];components=[]
    for o in root.allOccurrences:
        occ.append([o.fullPathName,o.transform2.asArray()])
        components.append([o.fullPathName,o.component.revisionId])
        for b in o.bRepBodies:
            bb=b.boundingBox
            bodies.append([o.fullPathName,b.name,b.revisionId,b.isVisible,bb.minPoint.asArray()+bb.maxPoint.asArray()])
    return dict(occ=sorted(occ),bodies=sorted(bodies),components=sorted(components),joints=sorted(j.name for j in root.allJoints),timeline=design.timeline.count)

def run(context):
    app=adsk.core.Application.get()
    doc=next(d for d in app.documents if d.creationId=='66980ae7-d1d8-48ed-bcd6-c785494a3fd8')
    state=snapshot(doc)
    assert hashlib.sha256(json.dumps(state['bodies'],sort_keys=True,separators=(',',':')).encode()).hexdigest()=='1626b467b7da6fa65817b643e92d8b4f19de40f7dbe98038d306a6f5445d3317'
    occ=dict(state['occ'])
    inv_path=Path('R:\\2024\\turn-lift-20261118\\native-refinement-import\\fusion-relieved-inventory.json');scan_path=Path('R:\\2024\\track-tread-candidate-20261118\\route-relieved\\scan.json')
    assert hashlib.sha256(inv_path.read_bytes()).hexdigest()=='4e87aed3b344dd80f22a6aa6fff2868322df6ea51b28eff0091bb1e3fe077f73'
    assert hashlib.sha256(scan_path.read_bytes()).hexdigest()=='3b311a1546d1e24ea92b80d16459174cbc547f2db92bcf3974b81b43f90d132f'
    inventory=json.loads(inv_path.read_bytes());scan=json.loads(scan_path.read_bytes())
    assert len(scan['inventory'])==len(inventory['bodies'])==len(state['bodies'])==734
    assert all(x['body']==y['name'] and (x['nativeGroup'] or 'fixed')==y['motionGroup'] and list(occ[x['path']])==y['transformCm'] for x,y in zip(inventory['bodies'],scan['inventory']))


    check(adsk.fusion.Design.cast(doc.products.itemByProductType("DesignProductType")),'R:\\2024\\track-tread-candidate-20261118\\route-relieved\\scan.json','R:\\2024\\six-speaker-20261118\\native-mid-loop-soft-delivery-24-0-poses.json','R:\\2024\\six-speaker-20261118\\native-mid-loop-soft-delivery-24-0-clearance.jsonl','5f2a1a4cb98931f31575444e95323c9cb78bab8df6d74f226c34dc53bd62bf96')
