"""Key-pose rigid NOTE contact audit, not continuous motion or contact dynamics."""
import json,pathlib,math,hashlib
import build_intake_candidate as assembly
from OCP.BRepPrimAPI import BRepPrimAPI_MakeTorus
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Ax2,gp_Pnt,gp_Dir
R=assembly.ROOT
import verify_receipts as verification

def run():
 before=assembly.input_pins();c=json.loads((R/'cad/assembly/intake_components.json').read_text());rows=[]
 for state in ['floor','rotated','handoff']:
  a=0 if state=='floor' else math.radians(c['pitchDeg']);z=c['floorCenterZ'] if state=='floor' else c['handoffCenterZ'] if state=='handoff' else c['note']['tubeRadius']+c['note']['majorRadius']*math.sin(a)
  note=BRepPrimAPI_MakeTorus(gp_Ax2(gp_Pnt(c['stageCenterX'],0,z),gp_Dir(-math.sin(a),0,math.cos(a))),c['note']['majorRadius'],c['note']['tubeRadius']).Shape();b=assembly.robot.bounds(note);hits=[]
  for n,s,k in assembly.build(state):
   if k=='service':continue
   q=assembly.robot.bounds(s)
   if min(min(b[i+3],q[i+3])-max(b[i],q[i]) for i in range(3))<=1e-6:continue
   gp=GProp_GProps();BRepGProp.VolumeProperties_s(BRepAlgoAPI_Common(note,s).Shape(),gp)
   if gp.Mass()>1e-4:hits.append({'part':n,'commonMm3':gp.Mass(),'intendedContactHypothesis':'side pinch belt' in n or 'nominal tread annulus' in n})
  rows.append({'state':state,'hits':hits})
 pins=assembly.input_pins();stable=verification.source_stable(before,pins);gate=verification.verify_exports(R/'artifacts/intake-component-candidate',pins);matches=stable and gate['valid']
 return {'schema':'intake-keypose-note-audit/2','assemblyInputs':before,'auditScriptSha256':hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest(),'sourceStableDuringAudit':stable,'matchesCurrentExports':matches,'exportStepHashes':gate['stepHashes'] if matches else None,'verificationFailures':gate['reasons']+([] if stable else ['source changed during audit']),'states':rows,'acceptance':False,'limits':['Intended contact does not prove force/friction','Three key poses only, not continuous sweep','Original source-sized shapes, not supplier native mating geometry']}
if __name__=='__main__':
 q=run();p=R/'artifacts/intake-component-candidate/note-contact.json';p.write_text(json.dumps(q,indent=2)+'\n',encoding='utf-8');bundle={'schema':'2024-assembly-verification-bundle/1','valid':q['matchesCurrentExports'],'buildReceiptSha256':verification.sha(p.parent/'report.json'),'contactReceiptSha256':verification.sha(p),'assemblyInputs':q['assemblyInputs'],'exportStepHashes':q['exportStepHashes']};(p.parent/'verification-bundle.json').write_text(json.dumps(bundle,indent=2)+'\n',encoding='utf-8');print(json.dumps({'valid':bundle['valid'],'states':q['states']}))
