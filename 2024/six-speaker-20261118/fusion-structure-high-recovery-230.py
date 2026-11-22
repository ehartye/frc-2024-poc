from pathlib import Path
import adsk.core,adsk.fusion,hashlib,json
def registered_document(doc):
    design=adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'));root=design.rootComponent
    occ=[];bodies=[];components=[]
    for o in root.allOccurrences:
        occ.append([o.fullPathName,o.transform2.asArray()])
        components.append([o.fullPathName,o.component.revisionId])
        for b in o.bRepBodies:
            bb=b.boundingBox
            bodies.append([o.fullPathName,b.name,b.revisionId,b.isVisible,bb.minPoint.asArray()+bb.maxPoint.asArray()])
    return dict(occ=sorted(occ),bodies=sorted(bodies),components=sorted(components),joints=sorted(j.name for j in root.allJoints),timeline=design.timeline.count)


import adsk.core,adsk.fusion,math,json,pathlib
C=adsk.core;F=adsk.fusion;P=pathlib.Path(r'R:/2024/turn-lift-20261118')
def run(context):
    app=adsk.core.Application.get()
    doc=next(d for d in app.documents if d.creationId=='66980ae7-d1d8-48ed-bcd6-c785494a3fd8')
    state=registered_document(doc)
    assert hashlib.sha256(json.dumps(state['bodies'],sort_keys=True,separators=(',',':')).encode()).hexdigest()=='1626b467b7da6fa65817b643e92d8b4f19de40f7dbe98038d306a6f5445d3317'
    occ=dict(state['occ'])
    inv_path=Path('R:\\2024\\turn-lift-20261118\\native-refinement-import\\fusion-relieved-inventory.json');scan_path=Path('R:\\2024\\track-tread-candidate-20261118\\route-relieved\\scan.json')
    assert hashlib.sha256(inv_path.read_bytes()).hexdigest()=='4e87aed3b344dd80f22a6aa6fff2868322df6ea51b28eff0091bb1e3fe077f73'
    assert hashlib.sha256(scan_path.read_bytes()).hexdigest()=='3b311a1546d1e24ea92b80d16459174cbc547f2db92bcf3974b81b43f90d132f'
    inventory=json.loads(inv_path.read_bytes());scan=json.loads(scan_path.read_bytes())
    assert len(scan['inventory'])==len(inventory['bodies'])==len(state['bodies'])==734
    assert all(x['body']==y['name'] and (x['nativeGroup'] or 'fixed')==y['motionGroup'] and list(occ[x['path']])==y['transformCm'] for x,y in zip(inventory['bodies'],scan['inventory']))


    docs=[doc]
    root=F.Design.cast(docs[0].products.itemByProductType('DesignProductType')).rootComponent;mgr=F.TemporaryBRepManager.get();inv=json.loads(pathlib.Path(r'R:/2024/track-tread-candidate-20261118/route-relieved/scan.json').read_bytes())['inventory'];parts=[];j=0
    for o in root.allOccurrences:
        for b in o.bRepBodies:
            r=inv[j];j+=1;assert r['name']==b.name
            if not r['exclusion']:parts.append((r,mgr.copy(b)))
    assert j==734
    def transform(theta,z):
        c,s=math.cos(theta),math.sin(theta);m=C.Matrix3D.create();m.setToRotation(-theta,C.Vector3D.create(0,1,0),C.Point3D.create(0,0,0));m.translation=C.Vector3D.create((178-c*178+s*25.4)/10,0,(z-s*178-c*25.4)/10);return m
    def snapshot(theta,z):
        items=[];cr=transform(theta,z);car=transform(0,z)
        for r,b in parts:
            g=r['motionGroup'];x=b
            if g!='fixed':
                x=mgr.copy(b);assert mgr.transform(x,cr if g=='cradle' else car)
            bb=x.boundingBox;items.append((r,x,bb.minPoint.asArray()+bb.maxPoint.asArray()))
        return items
    def common(x,y):
        b=mgr.copy(x);ok=mgr.booleanOperation(b,y,F.BooleanTypes.IntersectionBooleanType)
        return None if not ok else (b.volume*1000 if b.isValid and b.faces.count else 0)
    def hits(items):
        out={};tests=0
        for i,(r,x,a) in enumerate(items):
            for s,y,b in items[i+1:]:
                if r['motionGroup']==s['motionGroup'] or any(a[k+3]<b[k]-1e-5 or b[k+3]<a[k]-1e-5 for k in range(3)):continue
                tests+=1;v=common(x,y)
                if v is None or v>.01:out[(r['key'],s['key'])]=v
        return out,tests
    with pathlib.Path('R:\\2024\\six-speaker-20261118\\native-high-recovery-230-structure.jsonl').open('x') as f:
        ref,count=hits(snapshot(0,25.4));names={r['key']:r['name'] for r,b in parts};f.write(json.dumps(dict(label='floor',pairs=count,positive=len(ref)))+'\n');f.flush()
        poses=[(80.55500000026117, 1.046851559888418, 474.8344214960708), (80.57055000026169, 1.0468751570276296, 474.80832150662826), (80.62060000026335, 1.0466386585590672, 474.76473326989515), (81.32495000028673, 1.0421392098464664, 476.0412105489316), (81.49445000029236, 1.0418333524151242, 476.0558878280118), (81.50425000029269, 1.041798783867332, 476.0613075151963), (81.51620000029308, 1.0417082397047983, 476.0510833954813), (81.52360000029333, 1.0416936063688955, 476.04154719381506), (81.54620000029408, 1.041773332828696, 476.0266761595425)]
        for label,angle,z in poses:
            h,n=hits(snapshot(angle,z));changed=[]
            for pair,v in h.items():
                base=ref.get(pair,0.)
                if v is None or base is None or v-base>max(.01,1e-6*abs(base)):
                    changed.append(dict(parts=[names[k] for k in pair],baseMm3=base,currentMm3=v))
            f.write(json.dumps(dict(label=label,theta=angle,zMm=z,pairs=n,changes=changed))+'\n');f.flush()