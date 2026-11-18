"""Original reversible track/tread candidate; native import packet only."""
import argparse,copy,hashlib,json,math,pathlib,sys
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut,BRepAlgoAPI_Fuse

PROJECT=pathlib.Path(__file__).resolve().parents[2]
HERE=pathlib.Path(__file__).parent
sys.path[:0]=[str(PROJECT/'cad/assembly'),str(PROJECT/'cad')]
import intake_components as module
import build_robot
h=module.h
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def metrics(shape):
    properties=GProp_GProps();BRepGProp.VolumeProperties_s(shape,properties)
    center=properties.CentreOfMass();count=0;explorer=TopExp_Explorer(shape,TopAbs_SOLID)
    while explorer.More():count+=1;explorer.Next()
    return dict(volumeMm3=properties.Mass(),centroidMm=[center.X(),center.Y(),center.Z()],boundsMm=build_robot.bounds(shape),solidCount=count)

def capsule(cx,cy,z,separation,radius,width):
    left=h.cyl((cx-separation/2,cy,z),(0,0,1),radius,width)
    right=h.cyl((cx+separation/2,cy,z),(0,0,1),radius,width)
    bridge=h.box(cx-separation/2,cy-radius,z,separation,2*radius,width)
    return BRepAlgoAPI_Fuse(BRepAlgoAPI_Fuse(left,bridge).Shape(),right).Shape()

def run(args):
    output=args.output;output.mkdir(parents=True,exist_ok=False)
    original=json.loads((PROJECT/'cad/assembly/intake_components.json').read_text())
    revised=copy.deepcopy(original);revised['pinch']['innerFaceYAbs']+=1.
    if args.relieve_return_run:revised['pivotShaftInnerYAbs']=206.2
    before=module.build('floor',original,metadata=True);after=module.build('floor',revised,metadata=True)
    if args.relieve_return_run:
        # Original side-plate pocket: 0.5 mm nominal running clearance around
        # the new belt+tread exterior. Outer bracket/bearing frames stay fixed.
        p=revised['pinch'];radius=p['teeth']*p['pitch']/(2*math.pi)
        separation=(p['beltTeethHypothesis']-p['teeth'])*p['pitch']/2
        axis=p['innerFaceYAbs']+radius+1
        for side in (-1,1):
            tool=capsule(p['beltCenterX'],side*axis,-p['beltWidth']/2-.5,separation,radius+1+args.tread_mm+.5,p['beltWidth']+1)
            tool=h.pitch(tool,0,[original['stageCenterX'],0,original['floorCenterZ']])
            part=next(x for x in after if x['name']=='intake candidate original cradle side weldment '+str(side))
            part['shape']=BRepAlgoAPI_Cut(part['shape'],tool).Shape()
            assert metrics(part['shape'])['solidCount']==1,'Relief must retain one connected weldment solid'

    source_manifest=json.loads(args.source_manifest.read_text())
    source_indices={row['name']:row['sourceRootIndex'] for row in source_manifest['parts']}
    records=[];writer=h.STEPControl_Writer();changed=[]
    for a,b in zip(before,after):
        ma,mb=metrics(a['shape']),metrics(b['shape'])
        different=max(abs(x-y) for x,y in zip(ma['centroidMm']+ma['boundsMm'],mb['centroidMm']+mb['boundsMm']))>1e-7 or abs(ma['volumeMm3']-mb['volumeMm3'])>1e-5
        if not different:continue
        assert a['motion']==b['motion']=='cradle'
        pair=[]
        for kind,part,m in [('baseline',a,ma),('candidate',b,mb)]:
            assert h.BRepCheck_Analyzer(part['shape']).IsValid(),part['name']
            writer.Transfer(part['shape'],h.STEPControl_AsIs)
            row=dict(kind=kind,name=part['name'],originalName=a['name'],sourceRootIndex=source_indices[a['name']],motionGroup=part['motion'],role=part['role'],**m)
            records.append(row);pair.append(len(records)-1)
        changed.append(dict(originalName=a['name'],candidateName=b['name'],sourceRootIndex=source_indices[a['name']],packetRows=pair))
    p=revised['pinch'];radius=p['teeth']*p['pitch']/(2*math.pi);separation=(p['beltTeethHypothesis']-p['teeth'])*p['pitch']/2
    track=p['innerFaceYAbs']+radius+1
    for side in [-1,1]:
        outer=capsule(p['beltCenterX'],side*track,-p['beltWidth']/2,separation,radius+1+args.tread_mm,p['beltWidth'])
        inner=capsule(p['beltCenterX'],side*track,-p['beltWidth']/2-1,separation,radius+1,p['beltWidth']+2)
        tread=BRepAlgoAPI_Cut(outer,inner).Shape();tread=h.pitch(tread,0,[original['stageCenterX'],0,original['floorCenterZ']])
        assert h.BRepCheck_Analyzer(tread).IsValid()
        writer.Transfer(tread,h.STEPControl_AsIs)
        records.append(dict(kind='added-tread',name=f'candidate {args.tread_mm:g}mm outer belt tread {side}',sourceRootIndex=None,motionGroup='cradle',role='uncalibrated-compliant-tread',**metrics(tread)))
    path=output/'replacement-packet.step';assert writer.Write(str(path))==h.IFSelect_RetDone
    reader=h.STEPControl_Reader();assert reader.ReadFile(str(path))==h.IFSelect_RetDone;reader.TransferRoots();assert reader.NbRootsForTransfer()==len(records)
    note_radius=original['note']['majorRadius'];tube=original['note']['tubeRadius'];offset=.5
    clearances=[]
    for band in p['outerProfileBands']:
        zmin=0 if band['zMinMm']<=0<=band['zMaxMm'] else min(abs(band['zMinMm']),abs(band['zMaxMm']))
        note_extent=note_radius+math.sqrt(tube*tube-zmin*zmin)+offset
        clearance=track-band['proposedOuterRadiusMm']-note_extent
        assert clearance>0
        clearances.append(dict(**band,noteExtremeAbsYmm=note_extent,minimumYSeparationMm=clearance))
    report=dict(schema='track-tread-candidate/1',sourceArchiveSha256='24de65755a396d1e371112c65eb9a44a32884c122bf1e4387fd172645ea45423',sourceConfigSha256=sha(PROJECT/'cad/assembly/intake_components.json'),scriptSha256=sha(pathlib.Path(__file__)),stepSha256=sha(path),records=records,changed=changed,parameters=dict(shaftOutboardOffsetMm=1.,outerTreadThicknessMm=args.tread_mm,outerTreadDensityKgM3Hypothesis=1000.,nominalContactFaceAbsYmm=original['pinch']['innerFaceYAbs']+1-args.tread_mm,returnRunRelief=args.relieve_return_run,pivotInnerYAbsMm=revised['pivotShaftInnerYAbs'],reliefClearanceMm=.5 if args.relieve_return_run else 0.),revisedCarrierConfig=revised,clearanceBands=clearances,acceptance=False,limits=['Original new tread and symmetric carrier/drive geometry proposal, not selected purchased product or fabrication acceptance','Tread adhesion, stiffness, friction, wear and mass density are uncalibrated','All-phase band separation bounds NOTE versus pulley only; complete native route and mounting checks are separate','Other imported assembly parts and existing mechanical/electrical caveats remain unchanged'])
    (output/'manifest.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(changedParts=len(changed),packetRoots=len(records),addedTreadBodies=2,minimumPulleyYSeparationMm=min(x['minimumYSeparationMm'] for x in clearances),treadVolumesMm3=[row['volumeMm3'] for row in records if row['kind']=='added-tread'])))
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source-manifest',type=pathlib.Path,required=True);p.add_argument('--output',type=pathlib.Path,required=True);p.add_argument('--tread-mm',type=float,choices=[1.,1.5],required=True);p.add_argument('--relieve-return-run',action='store_true');run(p.parse_args())
