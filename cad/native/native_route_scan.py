"""Run inside Fusion MCP with caller-supplied CONFIG; private outputs only.

Finite flat NOTE/solid witnesses, not dynamics or continuous clearance.
CONFIG binds an already reopened audit document, archive, visual manifest,
independent source part manifest, and a fresh output path. No design edits.
"""
import adsk.core, adsk.fusion
import collections, hashlib, json, math, pathlib, time

def digest(path):
    return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()

def attribute(entity, group, name):
    a=entity.attributes.itemByName(group,name)
    return a.value if a else None

def bounds(body):
    box=body.boundingBox
    return [v*10 for v in box.minPoint.asArray()+box.maxPoint.asArray()]

def run(context):
    start=time.perf_counter();app=adsk.core.Application.get()
    assert digest(CONFIG['archive'])==CONFIG['archiveSha256']
    assert digest(CONFIG['manifest'])==CONFIG['manifestSha256']
    output=pathlib.Path(CONFIG['output']);assert not output.exists()
    matches=[]
    for doc in app.documents:
        design=adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
        if design and doc.creationId==CONFIG['documentId'] and attribute(design.rootComponent,'frcAudit','job')==CONFIG['job']:
            matches.append((doc,design))
    assert len(matches)==1,'Audit identity must match exactly once'
    doc,design=matches[0];root=design.rootComponent
    manifest=json.loads(pathlib.Path(CONFIG['manifest']).read_text())
    expected={row['key']:row for row in manifest['bodyInventory']}
    manager=adsk.fusion.TemporaryBRepManager.get()
    inventory=[];targets=[];unclassified=[]
    occurrences={o.fullPathName:o for o in root.allOccurrences}
    def inherited(o,namespace,name):
        path=o.fullPathName
        while path:
            value=attribute(occurrences[path].component,namespace,name)
            if value is not None:return value
            path=path.rsplit('+',1)[0] if '+' in path else ''
        return None
    for oi,o in enumerate(root.allOccurrences):
        c=o.component;index=inherited(o,'frcSource','sourceRootIndex')
        role=inherited(o,'frcSource','role');group=inherited(o,'frcSource','motionGroup') or 'fixed'
        visual=inherited(o,'frcVisualization','excludeFromPhysicsAndMass')=='true'
        for bi,b in enumerate(o.bRepBodies):
            key=f'body-{oi+1}-{bi+1}'
            e=expected[key]
            assert e['name']==b.name and e['sourceRootIndex']==(int(index) if index else None) and e['group']==group,(key,'inventory mismatch',e,{'name':b.name,'index':index,'group':group})
            assert b.isValid and b.isSolid,(key,'invalid or non-solid')
            body=manager.copy(b);assert body and body.isSolid
            bb=bounds(body);proxy=bounds(b)
            assert max(abs(x-y) for x,y in zip(bb,proxy))<=1e-4,(key,'copy frame mismatch')
            exclusion='service-envelope' if role=='service' else ('explicit-visual-only' if visual else None)
            if index is None and not exclusion:unclassified.append(key)
            row={'key':key,'name':b.name,'sourceRootIndex':int(index) if index else None,'role':role,'motionGroup':group,'visible':b.isVisible,'boundsMm':bb,'volumeMm3':body.volume*1000,'transformCm':o.transform2.asArray(),'exclusion':exclusion}
            inventory.append(row)
            if not exclusion:targets.append((row,body))
    assert set(expected)=={r['key'] for r in inventory},'Full inventory differs'
    result={'schema':'native-flat-route/1','sourceSha256':CONFIG['archiveSha256'],'manifestSha256':CONFIG['manifestSha256'],'scriptSha256':CONFIG['scriptSha256'],'configurationSha256':hashlib.sha256(json.dumps(CONFIG,sort_keys=True).encode()).hexdigest(),'version':app.version,'documentId':doc.creationId,'inventory':inventory,'unclassified':unclassified,'method':{'units':'mm','volumeUnits':'mm3','kernel':'Fusion TemporaryBRepManager','broadphase':'native BRepBody.boundingBox AABB; touching proceeds to Boolean','construction':'origin torus then explicit translation','lengthToleranceMm':1e-4,'volumeToleranceMm3':1e-4,'majorRadiusMm':152.4,'tubeRadiusMm':25.4,'rotation':'flat axis +Z only'},'controls':[],'rows':[],'failures':[],'complete':False,'acceptance':False}
    def save():
        result['seconds']=time.perf_counter()-start
        output.write_text(json.dumps(result,indent=2),encoding='utf-8')
    save()
    assert not unclassified,'Unclassified extra native geometry'
    radius=177.8;expected_volume=2*math.pi**2*152.4*25.4**2
    def note_at(center):
        n=manager.createTorus(adsk.core.Point3D.create(0,0,0),adsk.core.Vector3D.create(0,0,1),15.24,2.54)
        m=adsk.core.Matrix3D.create();m.translation=adsk.core.Vector3D.create(*[v/10 for v in center]);assert manager.transform(n,m)
        actual=bounds(n);half=[radius,radius,25.4]
        expected_bounds=[center[i]-half[i] for i in range(3)]+[center[i]+half[i] for i in range(3)]
        error=max(abs(a-b) for a,b in zip(actual,expected_bounds))
        volume=n.volume*1000
        assert error<=1e-4 and abs(volume-expected_volume)<=expected_volume*1e-8,('placement',center,actual,expected_bounds,volume)
        return n,actual
    def common(a,b):
        copy=manager.copy(a)
        ok=manager.booleanOperation(copy,b,adsk.fusion.BooleanTypes.IntersectionBooleanType)
        assert ok,'Boolean failure'
        return copy.volume*1000 if copy.isValid and copy.faces.count else 0.
    for center in [[0.,0.,0.],[658.,0.5,25.4],[185.,-0.5,25.4]]:
        note,bb=note_at(center);same=common(note,note);far,_=note_at([center[0]+1000,center[1],center[2]]);separate=common(note,far)
        assert abs(same-expected_volume)<expected_volume*1e-8 and separate<=1e-4
        wrong=[bb[i]+(1 if i%3==0 else 0) for i in range(6)]
        assert max(abs(x-y) for x,y in zip(bb,wrong))>1e-4
        result['controls'].append({'centerMm':center,'boundsMm':bb,'selfIntersectionMm3':same,'separatedIntersectionMm3':separate,'wrongLocationRejected':True})
    xs=CONFIG['xMm']
    assert xs and len(set(xs))==len(xs) and all(178<=x<=658 for x in xs)
    pairs=0
    for lateral in CONFIG['lateralMm']:
        for x in xs:
            center=[float(x),lateral,25.4];note,nb=note_at(center);hits=[]
            for record,body in targets:
                bb=record['boundsMm']
                if any(nb[i+3]<bb[i]-1e-4 or bb[i+3]<nb[i]-1e-4 for i in range(3)):continue
                pairs+=1
                volume=common(body,note)
                if volume>1e-4:
                    reverse=common(note,body)
                    assert abs(volume-reverse)<=max(1e-4,volume*1e-7),('operand disagreement',record['key'])
                    hits.append({'key':record['key'],'name':record['name'],'overlapMm3':volume,'reverseOverlapMm3':reverse,'classification':'intended-belt-contact-hypothesis' if 'side pinch belt' in record['name'] else 'unresolved-material-overlap'})
            result['rows'].append({'centerMm':center,'boundsMm':nb,'hits':hits})
        result['booleanPairs']=pairs;save()
    result['complete']=True
    result['unresolvedHitRows']=sum(any(h['classification']=='unresolved-material-overlap' for h in row['hits']) for row in result['rows'])
    result['verdict']='FINITE_FLAT_WITNESSES_CLEAR_EXCEPT_INTENDED_BELT' if not result['unresolvedHitRows'] else 'UNRESOLVED_NATIVE_OVERLAP'
    assert digest(CONFIG['archive'])==CONFIG['archiveSha256'] and digest(CONFIG['manifest'])==CONFIG['manifestSha256']
    save()
    print(json.dumps({k:result[k] for k in ['complete','verdict','unresolvedHitRows','booleanPairs','seconds']}))
