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
 parts,dc=robot.build();result=[]
 for n,s,k in parts:
  if n.startswith('UNRESOLVED') or n.startswith('original shooter upright') and '101.6' in n:continue
  if n in ['am-3920-2','am-2974-cut-2','frame fastening bridge-0','frame fastening bridge-1']:
   s=robot.cut(s,robot.h.box(50,-200,-10,400,400,410))
  result.append((n,s,k))
 return result+intake_components.build(state)

def run():
 root=ROOT/'artifacts/intake-component-candidate';root.mkdir(exist_ok=True);reports=[]
 for state in ['floor','rotated','handoff']:
  out=root/state;out.mkdir(exist_ok=True);parts=build(state);w=robot.h.STEPControl_Writer()
  for n,s,k in parts:
   assert robot.h.BRepCheck_Analyzer(s).IsValid(),n;w.Transfer(s,robot.h.STEPControl_AsIs)
  dest=out/'original-robot.step';assert w.Write(str(dest))==robot.h.IFSelect_RetDone
  rd=robot.h.STEPControl_Reader();assert rd.ReadFile(str(dest))==robot.h.IFSelect_RetDone;rd.TransferRoots();assert rd.NbRootsForTransfer()==len(parts)
  robot.render(parts,out,'rotating cradle / chain lift candidate: '+state)
  below=[{'part':n,'minZ':robot.bounds(s)[2]} for n,s,k in parts if robot.bounds(s)[2]<-0.01]
  reports.append({'state':state,'parts':len(parts),'reopenedRoots':rd.NbRootsForTransfer(),'belowGround':below,'stepSha256':hashlib.sha256(dest.read_bytes()).hexdigest()})
 pins=robot.input_pins()
 for name in ['cad/assembly/intake_components.py','cad/assembly/intake_components.json','cad/assembly/build_intake_candidate.py']:pins[name]=hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
 q={'schema':'2024-intake-component-candidate/1','inputs':pins,'states':reports,'acceptance':False,'limits':['Original source-sized representations, not supplier native geometry','Complete mechanism fasteners/couplings/chainlinks/guarding and brakes unresolved','Whole moving NOTE/component interference not yet screened; no fit acceptance','Modified central chassis removes transverse load paths; outboard-stub structural design unqualified','No capture/retention/friction or power-off holding model','Source-known mass ledger not yet extended to this candidate']}
 (root/'report.json').write_text(json.dumps(q,indent=2)+'\n',encoding='utf-8');print(json.dumps(reports))
if __name__=='__main__':run()
