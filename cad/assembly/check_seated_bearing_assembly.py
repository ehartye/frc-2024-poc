"""Changed-part assembly gate for the additive bearing seat candidate."""
import json,math,hashlib
import seated_bearing_assembly as candidate
from audit_cartridge_interfaces import common
from audit_intake_motion import poses
from OCP.BRepPrimAPI import BRepPrimAPI_MakeTorus
A=candidate.a;h=A.h

def pins():
 p=candidate.pins();p['cad/assembly/check_seated_bearing_assembly.py']=hashlib.sha256(__import__('pathlib').Path(__file__).read_bytes()).hexdigest();p['cad/assembly/audit_cartridge_interfaces.py']=hashlib.sha256((A.ROOT/'cad/assembly/audit_cartridge_interfaces.py').read_bytes()).hexdigest();return p

def overlap(a,b):return min(min(a[j+3],b[j+3])-max(a[j],b[j]) for j in range(3))>1e-6

def run():
 before=pins();c=json.loads((A.ROOT/'cad/assembly/intake_components.json').read_text());base=[(n,s,k) for n,s,k in A.assembly.build('floor') if not n.startswith('intake candidate')];rows=[]
 for phase,deg,z in poses(c):
  parts,changed=candidate.build_pose(c,deg,z,base);bounds={n:A.assembly.robot.bounds(s) for n,s,k in parts};hits=[];done=set();invalid=[]
  for n,s,k in parts:
   if n not in changed:continue
   if not h.BRepCheck_Analyzer(s).IsValid():invalid.append(n)
   for m,t,l in parts:
    key=tuple(sorted([n,m]))
    if n==m or l=='service' or key in done:continue
    done.add(key)
    if not overlap(bounds[n],bounds[m]):continue
    v=common(s,t)
    if v>1e-5:hits.append({'a':n,'b':m,'commonMm3':v})
  rows.append({'phase':phase,'pitchDeg':deg,'centerZmm':z,'partCount':len(parts),'changedPartCount':len(changed),'invalid':invalid,'hits':hits,'changedMinimumZmm':min(bounds[n][2] for n in changed)});print(phase,deg,z,len(hits),flush=True)
 if before!=pins():raise RuntimeError('inputs changed')
 out=A.ROOT/'artifacts/seated-bearing-assembly';out.mkdir(parents=True,exist_ok=True);report={'schema':'seated-bearing-changed-part-audit/1','inputs':before,'samples':rows,'adopted':False,'acceptance':False,'limits':['Finite15pose screen of ten changed parts against full711part assembly, not continuous clearance','Unchanged-to-unchanged pairs not rerun','Bearing stepped representation omits source detail, not automatic conservative material envelope','Shaft end retention, load, carpet and tolerance unresolved','Full NOTE entry/transfer and actual free final pose are separate pending gates']};(out/'report.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8',newline='\n')
if __name__=='__main__':run()
