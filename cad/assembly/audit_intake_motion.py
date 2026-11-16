"""Sample moving mechanism vs fixed assembly; report overlaps rather than assert fit."""
import json,math,pathlib,hashlib
import build_intake_candidate as a
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

def poses(c):
 for deg in range(0,61,10):yield 'rotate',deg,c['floorCenterZ']+c['note']['majorRadius']*math.sin(math.radians(deg))
 z0=c['floorCenterZ']+c['note']['majorRadius']*math.sin(math.radians(60))
 for j in range(1,9):yield 'lift',60,z0+(c['handoffCenterZ']-z0)*j/8

def run():
 c=json.loads((a.ROOT/'cad/assembly/intake_components.json').read_text());floor=a.intake_components.build('floor',c);end=a.intake_components.build('handoff',c)
 dynamic=[i for i in range(len(floor)) if max(abs(x-y) for x,y in zip(a.robot.bounds(floor[i][1]),a.robot.bounds(end[i][1])))>1e-6]
 fixed=[(n,s,a.robot.bounds(s)) for n,s,k in a.build('floor') if not n.startswith('intake candidate') and k!='service']
 fixed += [(n,s,a.robot.bounds(s)) for i,(n,s,k) in enumerate(floor) if i not in dynamic and k!='service']
 rows=[]
 for phase,deg,z in poses(c):
  parts=a.intake_components.build(c=c,pose=(deg,z));hits=[]
  for i in dynamic:
   n,s,k=parts[i];b=a.robot.bounds(s)
   for fn,fs,q in fixed:
    if min(min(b[j+3],q[j+3])-max(b[j],q[j]) for j in range(3))<=1e-5:continue
    gp=GProp_GProps();BRepGProp.VolumeProperties_s(BRepAlgoAPI_Common(s,fs).Shape(),gp)
    if gp.Mass()>1e-3:hits.append({'moving':n,'fixed':fn,'commonMm3':gp.Mass()})
  rows.append({'phase':phase,'pitchDeg':deg,'centerZ':z,'hits':hits})
 pins=a.robot.input_pins()
 for name in ['cad/assembly/intake_components.py','cad/assembly/intake_components.json','cad/assembly/build_intake_candidate.py','cad/assembly/audit_intake_motion.py']:pins[name]=hashlib.sha256((a.ROOT/name).read_bytes()).hexdigest()
 return {'schema':'intake-moving-fixed-audit/1','inputs':pins,'dynamicParts':len(dynamic),'fixedParts':len(fixed),'samples':rows,'acceptance':False,'limits':['Finite15pose screen, not continuous sweep','Moving-vs-moving internal joints not covered','Envelope collisions require exact component review; no intentional-pair exemptions suppress results']}
if __name__=='__main__':
 q=run();p=a.ROOT/'artifacts/intake-component-candidate/motion-audit.json';p.write_text(json.dumps(q,indent=2)+'\n',encoding='utf-8');print(json.dumps({'poses':len(q['samples']),'hitPairs':sorted({(h['moving'],h['fixed']) for r in q['samples'] for h in r['hits']})}))
