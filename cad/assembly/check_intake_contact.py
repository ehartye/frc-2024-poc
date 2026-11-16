"""Key-pose rigid NOTE contact audit, not continuous motion or contact dynamics."""
import json,pathlib,math
import build_intake_candidate as assembly
from OCP.BRepPrimAPI import BRepPrimAPI_MakeTorus
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Ax2,gp_Pnt,gp_Dir
R=assembly.ROOT

def run():
 c=json.loads((R/'cad/assembly/intake_components.json').read_text());rows=[]
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
 return {'schema':'intake-keypose-note-audit/1','states':rows,'acceptance':False,'limits':['Intended-contact label does not prove appropriate force/friction','Three key poses only; capture/rotation/lift/feed transition sweeps not covered','Original source-sized simplified component shapes, not supplier mating geometry']}
if __name__=='__main__':
 q=run();p=R/'artifacts/intake-component-candidate/note-contact.json';p.write_text(json.dumps(q,indent=2)+'\n',encoding='utf-8');print(json.dumps(q))
