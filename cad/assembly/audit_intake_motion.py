"""Sample moving mechanism vs fixed assembly; report overlaps rather than assert fit."""
import json,math,pathlib,hashlib
import build_intake_candidate as a
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

def poses(c):
 for deg in [c['pitchDeg']*i/6 for i in range(7)]:yield 'rotate',deg,c['floorCenterZ']+c['note']['majorRadius']*math.sin(math.radians(deg))
 z0=c['floorCenterZ']+c['note']['majorRadius']*math.sin(math.radians(c['pitchDeg']))
 for j in range(1,9):yield 'lift',c['pitchDeg'],z0+(c['handoffCenterZ']-z0)*j/8

def run():
 before=a.input_pins();c=json.loads((a.ROOT/'cad/assembly/intake_components.json').read_text());floor=a.intake_components.build('floor',c);end=a.intake_components.build('handoff',c)
 metadata=a.intake_components.build('floor',c,metadata=True);dynamic=[i for i,p in enumerate(metadata) if p['motion']!='fixed']
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
  all_bounds=[q for n,s,q in fixed]+[a.robot.bounds(parts[i][1]) for i in dynamic if parts[i][2]!='service'];overall=[min(q[j] for q in all_bounds) for j in range(3)]+[max(q[j+3] for q in all_bounds) for j in range(3)]
  rows.append({'robotBoundsMm':overall,'phase':phase,'pitchDeg':deg,'centerZ':z,'hits':hits})
 pins=a.input_pins()
 if pins!=before:raise RuntimeError('source changed during motion audit')
 for name in ['cad/assembly/intake_components.py','cad/assembly/intake_components.json','cad/assembly/build_intake_candidate.py','cad/assembly/audit_intake_motion.py']:pins[name]=hashlib.sha256((a.ROOT/name).read_bytes()).hexdigest()
 return {'schema':'intake-moving-fixed-audit/1','inputs':pins,'dynamicParts':len(dynamic),'fixedParts':len(fixed),'samples':rows,'acceptance':False,'limits':['Finite15pose screen, not continuous sweep','Moving-vs-moving internal joints not covered','Envelope collisions require exact component review; no intentional-pair exemptions suppress results']}
if __name__=='__main__':
 q=run();p=a.ROOT/'artifacts/intake-component-candidate/motion-audit.json';p.write_text(json.dumps(q,indent=2)+'\n',encoding='utf-8',newline='\n');print(json.dumps({'poses':len(q['samples']),'hitPairs':sorted({(h['moving'],h['fixed']) for r in q['samples'] for h in r['hits']})}))
