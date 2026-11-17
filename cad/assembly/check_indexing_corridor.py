"""Sample candidate inward indexing offsets without moving or resetting runtime state."""
import json,math,pathlib,hashlib
import build_connected_candidate as a
from extract_retention_patch import world
from audit_intake_motion import poses
from audit_cartridge_interfaces import common
from OCP.BRepPrimAPI import BRepPrimAPI_MakeTorus
OFFSETS=[0,5,10,20,30,40]
def pins():
 p=a.input_pins()
 for f in ['cad/assembly/check_indexing_corridor.py','cad/assembly/extract_retention_patch.py','cad/assembly/audit_cartridge_interfaces.py']:p[f]=hashlib.sha256((a.ROOT/f).read_bytes()).hexdigest()
 return p

def run():
 before=pins();c=json.loads((a.ROOT/'cad/assembly/intake_components.json').read_text());base=[(n,s,k) for n,s,k in a.assembly.build('floor') if not n.startswith('intake candidate')];rows=[]
 for phase,deg,z in poses(c):
  parts=base+a.assembly.intake_components.build(c=c,pose=(deg,z));parts,_=a.upper.lower.apply(parts,c);parts,_=a.upper.apply(parts,z,c);parts,_=a.pads.apply(parts,z,c);bounds={n:a.assembly.robot.bounds(s) for n,s,k in parts};angle=math.radians(deg)
  for offset in OFFSETS:
   center=world([offset,0,0],angle,[c['stageCenterX'],0,z]);note=BRepPrimAPI_MakeTorus(a.h.gp_Ax2(a.h.gp_Pnt(*center),a.h.gp_Dir(-math.sin(angle),0,math.cos(angle))),c['note']['majorRadius'],c['note']['tubeRadius']).Shape();b=a.assembly.robot.bounds(note);hits=[]
   for n,s,k in parts:
    if k=='service':continue
    q=bounds[n]
    if min(min(b[j+3],q[j+3])-max(b[j],q[j]) for j in range(3))<=1e-6:continue
    v=common(note,s)
    if v>1e-5:hits.append({'part':n,'commonMm3':v,'intendedContactHypothesis':'side pinch belt' in n or 'nominal tread annulus' in n})
   rows.append({'offsetMm':offset,'phase':phase,'pitchDeg':deg,'centerZmm':z,'centerRobotMm':center,'hits':hits})
  print(phase,deg,'done',flush=True)
 if before!=pins():raise ValueError('input changed')
 out=a.ROOT/'artifacts/indexing-corridor';out.mkdir(parents=True,exist_ok=True);report={'schema':'sampled-indexing-corridor/1','inputs':before,'offsetCandidatesMm':OFFSETS,'samples':rows,'acceptance':False,'limits':['Six hypothetical longitudinal offsets at15prescribed poses; not a continuous corridor or actual indexed state','No runtime state reset, actual indexing control and shared force/power remain separate','Canonical711part original/source-sized geometry; native bearing correction remains separate','Intended belt/tread overlaps recorded but not force validation','Clear geometry does not repair insufficient normal force or prove retention']};(out/'report.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8',newline='\n')
if __name__=='__main__':run()
