"""Read-only native contact binding from the final 734-body export receipt."""
import argparse,hashlib,json,math,pathlib
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def bind(project,archive,receipt):
    cp=project/'cad/assembly/intake_components.json';rp=project/'sim/integrated/evidence/track-tread-candidate-20261118.json';c=json.loads(cp.read_bytes());public=json.loads(rp.read_bytes());native=json.loads(receipt.read_bytes());parameters=public['native']['parameters']
    assert sha(archive)==public['native']['finalArchiveSha256']==native['exportSha256']
    assert len(native['after'])==734 and native['complete'] and native['originalUnmodified'] and not native['featureErrors']
    assert sha(cp)==public['staticCapacity']['onePointFiveMm']['inputs']['cad/assembly/intake_components.json']
    assert parameters['shaftOutboardOffsetMm']==1 and parameters['outerTreadThicknessMm']==1.5 and parameters['returnRunRelief']
    p=c['pinch'];radius=p['teeth']*p['pitch']/(2*math.pi);separation=(p['beltTeethHypothesis']-p['teeth'])*p['pitch']/2;axis=p['innerFaceYAbs']+radius+1+parameters['shaftOutboardOffsetMm'];outer=radius+1+parameters['outerTreadThicknessMm'];inner=radius-1;cx=c['stageCenterX']+p['beltCenterX'];z=c['floorCenterZ'];rows=[]
    for side in [-1,1]:
        for kind,ri,ro,name in [('belt',inner,radius+1,f'intake candidate side pinch belt {side}'),('tread',radius+1,outer,f'candidate 1p5mm outer belt tread {side}')]:
            row=next(x for x in native['after'] if x['name']==name);bounds=[cx-separation/2-ro,side*axis-ro,z-p['beltWidth']/2,cx+separation/2+ro,side*axis+ro,z+p['beltWidth']/2];volume=p['beltWidth']*(2*separation*(ro-ri)+math.pi*(ro*ro-ri*ri));error=max(abs(a-b) for a,b in zip(bounds,row['boundsMm']));verror=abs(volume-row['volumeMm3']);assert error<1e-6 and verror<1e-5
            rows.append(dict(kind=kind,side=side,sourceRootIndex=int(row['sourceRootIndex']),boundsMm=bounds,volumeMm3=volume,maximumBoundsErrorMm=error,volumeErrorMm3=verror))
    g=dict(rearX=p['beltCenterX']-separation/2,frontX=p['beltCenterX']+separation/2,axisYAbs=axis,outerRadius=outer,innerRadius=inner,halfHeight=p['beltWidth']/2,treadThicknessMm=parameters['outerTreadThicknessMm'])
    assert abs(axis-outer-parameters['nominalContactFaceAbsYmm'])<1e-10
    return dict(schema='native-pressure-contact-binding/1',contactGeometryBound=True,acceptance=False,scriptSha256=sha(pathlib.Path(__file__)),nativeArchiveSha256=sha(archive),finalNativeReceiptSha256=sha(receipt),publicNativeReceiptSha256=sha(rp),baseConfigSha256=sha(cp),bodyCount=734,nativeParameters=parameters,effectiveGeometryMm=g,registeredBodies=rows,corridor=dict(relativeXM=[.178,.658],relativeYAbsM=.0005,relativeYawAbsRad=1e-8,fixedFloorCenterZM=c['floorCenterZ']/1000),limits=['Four native belt/tread solids bound by final export identity, volume and bounds; not a complete rigid-body collision model','Floor corridor has finite native witnesses, not continuous collision qualification','Only shallow contact within the 1.5 mm tread is modeled; substrate contact must stop','Prescribed flat floor and roll/pitch remain; this binding does not authorize lifted or tilted states','No Fusion access or native mutation'])
if __name__=='__main__':
    p=argparse.ArgumentParser()
    for name in ['project','native-archive','native-receipt','output']:p.add_argument('--'+name,type=pathlib.Path,required=True)
    a=p.parse_args();assert not a.output.exists();r=bind(a.project,a.native_archive,a.native_receipt);a.output.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8',newline='\n');print(json.dumps(r))
