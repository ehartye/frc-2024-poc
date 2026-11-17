"""Integrated original candidate with lower, upper and carriage attachment increments."""
import pathlib,json,math,hashlib
import upper_carriage_connections as upper
import verify_receipts as verification
from OCP.BRepPrimAPI import BRepPrimAPI_MakeTorus
from OCP.gp import gp_Ax2,gp_Pnt,gp_Dir
assembly=upper.assembly;h=upper.h;ROOT=assembly.ROOT

def input_pins():
 pins=assembly.input_pins()
 for name in ['lower_support_mounts.py','upper_carriage_connections.py','build_connected_candidate.py']:
  relative='cad/assembly/'+name;pins[relative]=hashlib.sha256((ROOT/relative).read_bytes()).hexdigest()
 return pins

def pose(state,c):
 a=0 if state=='floor' else math.radians(c['pitchDeg'])
 z=c['floorCenterZ'] if state=='floor' else c['handoffCenterZ'] if state=='handoff' else c['note']['tubeRadius']+c['note']['majorRadius']*math.sin(a)
 return a,z

def build(state):
 c=json.loads((ROOT/'cad/assembly/intake_components.json').read_text());a,z=pose(state,c)
 parts,lowerjoints=upper.lower.apply(assembly.build(state),c);parts,upperjoints=upper.apply(parts,z,c)
 return parts,lowerjoints+upperjoints

def run():
 before=input_pins();root=ROOT/'artifacts/connected-intake-candidate';root.mkdir(parents=True,exist_ok=True);states=[];contacts=[]
 c=json.loads((ROOT/'cad/assembly/intake_components.json').read_text())
 for state in verification.STATES:
  out=root/state;out.mkdir(exist_ok=True);parts,joints=build(state);writer=h.STEPControl_Writer()
  for n,s,k in parts:
   if not h.BRepCheck_Analyzer(s).IsValid():raise RuntimeError(n)
   writer.Transfer(s,h.STEPControl_AsIs)
  dest=out/'original-robot.step'
  if writer.Write(str(dest))!=h.IFSelect_RetDone:raise RuntimeError('export failed')
  reader=h.STEPControl_Reader()
  if reader.ReadFile(str(dest))!=h.IFSelect_RetDone:raise RuntimeError('reopen failed')
  reader.TransferRoots()
  if reader.NbRootsForTransfer()!=len(parts):raise RuntimeError('root mismatch')
  renderparts=[(n,s,k if k in h.COLORS or k=='service' else 'mechanism' if 'fastener' in k else 'support') for n,s,k in parts]
  assembly.robot.render(renderparts,out,'connected intake candidate: '+state)
  a,z=pose(state,c);note=BRepPrimAPI_MakeTorus(gp_Ax2(gp_Pnt(c['stageCenterX'],0,z),gp_Dir(-math.sin(a),0,math.cos(a))),c['note']['majorRadius'],c['note']['tubeRadius']).Shape();b=assembly.robot.bounds(note);hits=[];below=[];manifest=[]
  for n,s,k in parts:
   bounds=assembly.robot.bounds(s)
   if bounds[2]<-.01:below.append({'part':n,'minZ':bounds[2]})
   manifest.append(dict(name=n,role=k,**assembly.classify_part(n)))
   if k=='service' or min(min(b[i+3],bounds[i+3])-max(b[i],bounds[i]) for i in range(3))<=1e-6:continue
   v=upper.lower.volume(note,s)
   if v>1e-4:hits.append({'part':n,'commonMm3':v,'intendedContactHypothesis':'side pinch belt' in n or 'nominal tread annulus' in n})
  states.append({'state':state,'parts':len(parts),'reopenedRoots':reader.NbRootsForTransfer(),'stepSha256':verification.sha(dest),'belowGround':below,'partManifest':manifest,'attachmentJoints':joints})
  contacts.append({'state':state,'hits':hits})
 if before!=input_pins():raise RuntimeError('source changed during export')
 report={'schema':'2024-intake-component-candidate/1','inputs':before,'states':states,'acceptance':False,'limits':['Original authored and supplier-sized representations, not fabrication acceptance','Attachment increments integrated; chain/shaft/motor/slide and remaining shooter joints incomplete','Three NOTE key poses only, no continuous capture/contact/force validation','Structural loads, fastener preload, tool access and installation remain unqualified','Electrical source caveat: breaker supplier model is surface-only and PDH has an invalid solid; prior material-volume clearance is not accepted']}
 (root/'report.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
 gate=verification.verify_exports(root,before)
 contact={'schema':'connected-candidate-note-audit/1','inputs':before,'states':contacts,'matchesCurrentExports':gate['valid'],'exportStepHashes':gate['stepHashes'],'acceptance':False}
 (root/'note-contact.json').write_text(json.dumps(contact,indent=2)+'\n',encoding='utf-8')
 bundle={'schema':'2024-assembly-verification-bundle/1','valid':gate['valid'] and before==input_pins(),'buildReceiptSha256':verification.sha(root/'report.json'),'contactReceiptSha256':verification.sha(root/'note-contact.json'),'assemblyInputs':before,'exportStepHashes':gate['stepHashes']}
 (root/'verification-bundle.json').write_text(json.dumps(bundle,indent=2)+'\n',encoding='utf-8')
 print(json.dumps({'valid':bundle['valid'],'states':[(x['state'],x['parts'],x['belowGround']) for x in states],'contact':contacts}))
if __name__=='__main__':run()
