"""Retain raw actuator packaging conflicts; native servo is private input."""
import pathlib,json,hashlib,math
import servo_opening_mount as m
from audit_cartridge_interfaces import common
s=m.s;h=m.h
def run(cache):
 before=s.pins()
 for name in ['servo_opening_mount.py','cassette_opening_cam.py','cassette-opening-source-candidate.json','check_servo_opening_mount.py','audit_cartridge_interfaces.py']:
  key='cad/assembly/'+name;before[key]=hashlib.sha256((s.a.ROOT/key).read_bytes()).hexdigest()
 p=cache/'REV-41-1097.STEP';assert hashlib.sha256(p.read_bytes()).hexdigest()==m.SOURCE_SHA
 r=h.STEPControl_Reader();r.ReadFile(str(p));r.TransferRoots();raw=r.OneShape()
 if not h.BRepCheck_Analyzer(raw).IsValid():raise ValueError('invalid native servo')
 native=m.register(raw);checks=[{'part':n,'commonMm3':common(shape,native)} for n,shape,k in m.local_parts(0) if k!='source-envelope' and ('mounting frame' in n or 'shank' in n)]
 rows=[];c=json.loads((s.a.ROOT/'cad/assembly/intake_components.json').read_text())
 for state in ['floor','rotated','handoff']:
  angle,z=s.a.pose(state,c);base,_=s.a.build(state)
  for theta in [0,math.pi]:
   u=m.cam.evaluate(theta)['contactTravelMm'];cass,removed=s.build(math.degrees(angle),z,u);others=[(n,p,k) for n,p,k in base if n not in removed]+[(n,p,k) for n,p,k,_ in cass];hits=[]
   for n,p,k in m.local_parts(theta):
    p=h.pitch(p,angle,[c['stageCenterX'],0,z]);b=s.a.assembly.robot.bounds(p)
    for nn,q,kk in others:
     a=s.a.assembly.robot.bounds(q)
     if min(min(a[i+3],b[i+3])-max(a[i],b[i]) for i in range(3))<=1e-6:continue
     v=common(p,q)
     if v>1e-5:hits.append({'a':n,'b':nn,'commonMm3':v})
   rows.append({'pose':state,'theta':theta,'uMm':u,'hits':hits});print(state,u,len(hits),flush=True)
 if any(hashlib.sha256((s.a.ROOT/k).read_bytes()).hexdigest()!=v for k,v in before.items()):raise ValueError('inputs changed')
 return {'inputs':before,'nativeSha':m.SOURCE_SHA,'nativeMountChecks':checks,'rows':rows,'accepted':False,'limits':['Bracket and four shank witnesses clear nominal native servo; no tolerance/preload acceptance','Whole-robot conflicts are raw: source servo envelope can overbound; actual authored follower/backbone intersection blocks design','Follower/base proposed weld overlap retained, not silently accepted as a fabrication joint','No servo horn/cam coupling or bracket-to-backbone attachment completed','Endpoint-only cam constraint samples are not actual dynamics; follower can separate and u is not prescribed','No negative side, internal added-parts or continuous sweep acceptance']}
if __name__=='__main__':
 report=run(pathlib.Path.home()/'.cache/frc-component-cartridge');out=s.a.ROOT/'artifacts/spring-cassette/servo-mount-first-screen.json';out.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8',newline='\n')
