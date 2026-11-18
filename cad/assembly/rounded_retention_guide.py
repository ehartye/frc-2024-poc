"""Original rounded wear-guide geometry at exact indexed NOTE offset, not contact acceptance."""
import json,math,pathlib,hashlib,argparse
import build_connected_candidate as a
from extract_retention_patch import world,classify
from audit_cartridge_interfaces import common
from OCP.BRepAlgoAPI import BRepAlgoAPI_Fuse,BRepAlgoAPI_Cut
from OCP.BRepPrimAPI import BRepPrimAPI_MakeTorus
from OCP.TopAbs import TopAbs_IN,TopAbs_ON
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def pins():
 p=a.input_pins()
 for f in ['cad/assembly/rounded_retention_guide.py','cad/assembly/extract_retention_patch.py','cad/assembly/audit_cartridge_interfaces.py']:p[f]=sha(a.ROOT/f)
 return p

def build():
 frame=json.loads((a.ROOT/'cad/drivetrain/candidate.json').read_text());from bumper_candidate import CONFIG
 face=frame['frameNativeEnvelopeMm'][0]/2-3.175;bottom=CONFIG['topMm']-CONFIG['backingHeightMm'];r=.5;yc=125.;top=150.;cx=face-r
 s=a.h.box(face-1,-yc,bottom+r,1,2*yc,top-bottom-2*r)
 for z in [bottom+r,top-r]:s=BRepAlgoAPI_Fuse(s,a.h.cyl((cx,-yc,z),(0,1,0),r,2*yc)).Shape()
 for y in [-110,110]:s=BRepAlgoAPI_Cut(s,a.h.cyl((face-2,y,110),(1,0,0),2.25,3)).Shape()
 if not a.h.BRepCheck_Analyzer(s).IsValid():raise ValueError('invalid guide')
 return s,{'supportInboardFaceXmm':face,'guideFaceXmm':face-1,'guideThicknessMm':1,'halfWidthMm':yc,'bottomZmm':bottom,'topZmm':top,'longEdgeRadiusMm':r,'mountHoleHypothesis':{'diameterMm':4.5,'Ymm':[-110,110],'Zmm':110},'materialHypothesis':'thin low-friction polymer wear sheet, grade and fastening unresolved'}

def run(runtime):
 before=pins();path=runtime/'artifacts/fast-regulated-index.json';digest=sha(path);receipt=json.loads(path.read_text())
 def verify():
  if sha(path)!=digest or any(sha(runtime/f)!=v for f,v in receipt['inputs'].items()):raise ValueError('runtime changed')
 verify();c=json.loads((a.ROOT/'cad/assembly/intake_components.json').read_text());state=receipt['final'];offset=1000*state['relativeRobotXY'][0]-c['stageCenterX'];guide,g=build();fronts=[(n,s) for n,s,k in a.assembly.build('floor') if n in ['original rigid front backing support proposal','fixed bumper end backing 1']];rows=[]
 for deg in range(0,31,2):
  angle=math.radians(deg);z=c['floorCenterZ']+c['note']['majorRadius']*math.sin(angle);center=world([offset,0,0],angle,[c['stageCenterX'],0,z]);note=BRepPrimAPI_MakeTorus(a.h.gp_Ax2(a.h.gp_Pnt(*center),a.h.gp_Dir(-math.sin(angle),0,math.cos(angle))),c['note']['majorRadius'],c['note']['tubeRadius']).Shape();witness=[]
  for y in range(-120,121,10):
   for zz in range(60,150,5):
    point=[g['guideFaceXmm'],y,zz]
    if classify(note,point) in [TopAbs_IN,TopAbs_ON]:witness.append({'pointRobotMm':point,'guideOutwardNormal':[-1,0,0],'surface':'planar wear face'})
  for t in range(180,271,15):
   rad=math.radians(t)
   for y in range(-120,121,10):
    point=[g['supportInboardFaceXmm']-.5+.5*math.cos(rad),y,g['bottomZmm']+.5+.5*math.sin(rad)]
    if classify(note,point) in [TopAbs_IN,TopAbs_ON]:witness.append({'pointRobotMm':point,'guideOutwardNormal':[math.cos(rad),0,math.sin(rad)],'surface':'rounded lower edge'})
  interior=[]
  for w in witness:
   if w['surface']!='planar wear face':continue
   point=list(w['pointRobotMm']);point[0]+=.01
   if classify(note,point)==TopAbs_IN and classify(guide,point)==TopAbs_IN:interior.append(point)
  volume=common(note,guide)
  rows.append({'interiorMaterialWitnesses':interior,'booleanWitnessConsistent':not(interior and volume<=1e-5),'pitchDeg':deg,'noteCenterRobotMm':center,'guideOverlapMm3':volume,'structuralContacts':[{'part':n,'commonMm3':common(note,s)} for n,s in fronts],'guideSurfaceWitnesses':witness});print(deg,rows[-1]['guideOverlapMm3'],len(witness),flush=True)
 out=a.ROOT/'artifacts/rounded-retention-guide';out.mkdir(parents=True,exist_ok=True);dest=out/'original-guide.step';wr=a.h.STEPControl_Writer();wr.Transfer(guide,a.h.STEPControl_AsIs)
 if wr.Write(str(dest))!=a.h.IFSelect_RetDone:raise ValueError('export')
 rd=a.h.STEPControl_Reader();rd.ReadFile(str(dest));rd.TransferRoots()
 if rd.NbRootsForTransfer()!=1:raise ValueError('reopen')
 verify()
 if before!=pins():raise ValueError('CAD changed')
 q={'schema':'original-rounded-retention-guide/1','inputs':before,'indexedReceiptSha256':digest,'indexedInputs':receipt['inputs'],'preservedEntryState':state,'longitudinalOffsetMm':offset,'guide':g,'samples':rows,'originalStepSha256':sha(dest),'reopenedRoots':1,'adopted':False,'acceptance':False,'limits':['Unfastened original wear-guide proposal backed by existing structure; bores only, actual mounting stack and support bores not implemented','Guide adds1mm inward intrusion, so original structural contacts and extra undeformed foam occupancy remain visible','Surface witnesses provide geometric locations/normals, not forces, compression states or integrated motion','Rigid carried-pose probe at16angles, not a claimed attachment or actual continuation of entry velocities','Native Boolean zero with positive interior material witnesses is retained as UNKNOWN, never clearance','Contact compliance, friction, pitch drive work, wear, pressure and sufficient preload all unmodeled','Rounded long edges only; side edges outside sampled central contact require installation coverage check','No safety verdict from foam overlap, and no automatic exemption or successful compression assertion']};(out/'report.json').write_text(json.dumps(q,indent=2)+'\n',encoding='utf-8',newline='\n')
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--runtime',type=pathlib.Path,required=True);args=p.parse_args();run(args.runtime)
