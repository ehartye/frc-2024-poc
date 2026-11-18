"""Export the opened-throat assembly with actual proposed cradle/elevator parts."""
import pathlib,sys,json,hashlib
ROOT=pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'cad/assembly'))
import build_robot as robot
import intake_components
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

def build(state):
 config=json.loads((ROOT/'cad/assembly/intake_components.json').read_text());parts,dc=robot.build(shooter_pivot_x_mm=config.get('shooterPivotXmm'),shooter_support_y_mm=config.get('shooterSupportYAbs',228.6));result=[]
 for n,s,k in parts:
  if n in config.get('removedSourceMembers',[]):continue
  if n.startswith('UNRESOLVED') or n.startswith('original shooter upright') and '101.6' in n:continue
  if n in ['frame fastening bridge-0','frame fastening bridge-1']:
   fc=json.loads((ROOT/'cad/drivetrain/fastening.json').read_text());index=int(n[-1]);x=fc['bridgeCentersXmm'][index];bottom=fc['railTopZmm']+4
   s=robot.h.tube(x-12.7,-313,bottom,25.4,626,50.8,3.175,'y')
   for sy in [-228.6,228.6]:s=robot.cut(s,robot.h.cyl((x,sy,bottom-1),(0,0,1),2.667,52.8))
  if n in ['am-3920-2','am-2974-cut-2','frame fastening bridge-0','frame fastening bridge-1']:
   s=robot.cut(s,robot.h.box(50,-config['throatHalfWidthMm'],-10,400,2*config['throatHalfWidthMm'],410))
  result.append((n,s,k))
 top=config['replacementPortalTopZ'];gx=config['guide']['x'];result.append(('original replacement upper transverse portal',robot.h.tube(gx-12.7,-313,top,25.4,626,50.8,3.175,'y'),'support'))
 return result+intake_components.build(state)

def input_pins():
 pins=robot.input_pins()
 for name in ['cad/assembly/intake_components.py','cad/assembly/intake_components.json','cad/assembly/build_intake_candidate.py','cad/assembly/check_intake_contact.py','cad/assembly/verify_receipts.py','cad/assembly/audit_intake_motion.py','cad/components/check_pulley_outer_profile.py','cad/components/pulley-outer-profile.json']:pins[name]=hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
 return pins

def classify_part(name):
 original_words=['drilled bearing carrier','drilled motor slider','slotted carrier bridge','original shooter side rail','original shooter upright','original carrier connection web','stationary aluminum guide','lift carriage plate','pivot drop bracket','chain carriage anchor','bearing carrier','cradle side stock','pinch motor plate']
 estimate=any(x in name for x in original_words)
 return {'fidelity':'original authored stock/plate proposal' if estimate else 'source-sized or nominal interface representation; see component manifest','massModel':{'kind':'density_estimate','densityKgM3':2700,'material':'assumed aluminum, grade unspecified'} if estimate else None,'notIncluded':'supplier internals/threads and physical acceptance' if not estimate else 'joint/strength/tolerance qualification'}

def run():
 root=ROOT/'artifacts/intake-component-candidate';root.mkdir(exist_ok=True);reports=[];before=input_pins()
 for state in ['floor','rotated','handoff']:
  out=root/state;out.mkdir(exist_ok=True);parts=build(state);w=robot.h.STEPControl_Writer()
  for n,s,k in parts:
   assert robot.h.BRepCheck_Analyzer(s).IsValid(),n;w.Transfer(s,robot.h.STEPControl_AsIs)
  dest=out/'original-robot.step';assert w.Write(str(dest))==robot.h.IFSelect_RetDone
  rd=robot.h.STEPControl_Reader();assert rd.ReadFile(str(dest))==robot.h.IFSelect_RetDone;rd.TransferRoots();assert rd.NbRootsForTransfer()==len(parts)
  robot.render(parts,out,'rotating cradle / chain lift candidate: '+state)
  below=[{'part':n,'minZ':robot.bounds(s)[2]} for n,s,k in parts if robot.bounds(s)[2]<-0.01]
  reports.append({'partManifest':[dict(name=n,role=k,**classify_part(n)) for n,s,k in parts],'state':state,'parts':len(parts),'reopenedRoots':rd.NbRootsForTransfer(),'belowGround':below,'stepSha256':hashlib.sha256(dest.read_bytes()).hexdigest()})
 pins=input_pins()
 if pins!=before:raise RuntimeError('source changed during assembly export')
 q={'schema':'2024-intake-component-candidate/1','inputs':pins,'states':reports,'acceptance':False,'limits':['Original source-sized representations, not supplier native geometry','Complete mechanism fasteners/couplings/chainlinks/guarding and brakes unresolved','Whole moving NOTE/component interference not yet screened; no fit acceptance','Modified central chassis removes transverse load paths; outboard-stub structural design unqualified','No capture/retention/friction or power-off holding model','Source-known mass ledger not yet extended to this candidate']}
 (root/'report.json').write_text(json.dumps(q,indent=2)+'\n',encoding='utf-8');print(json.dumps(reports))
if __name__=='__main__':run()
