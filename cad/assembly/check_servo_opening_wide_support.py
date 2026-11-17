"""Full-robot sampled relocation screen. Raw contacts, never exclusions."""
import pathlib,json,math,hashlib
import servo_opening_wide_support as m
from audit_cartridge_interfaces import common
s=m.s;h=m.h
def run():
 pins=s.pins()
 for n in ['servo_opening_mount.py','servo_opening_relocation.py','cassette_opening_cam.py','cassette-opening-source-candidate.json','audit_cartridge_interfaces.py','check_servo_opening_relocation.py','servo_opening_fasteners.py','servo-opening-fastener-source.json','check_servo_opening_fasteners_screen.py','servo_opening_wide_support.py','check_servo_opening_wide_support.py']:
  p=s.a.ROOT/'cad/assembly'/n;pins['cad/assembly/'+n]=hashlib.sha256(p.read_bytes()).hexdigest()
 c=json.loads((s.a.ROOT/'cad/assembly/intake_components.json').read_text());rows=[]
 for state in ['floor','rotated','handoff']:
  angle,z=s.a.pose(state,c);base,_=s.a.build(state)
  for theta,u in [(t,u) for t in [0,math.pi/4,math.pi/2,3*math.pi/4,math.pi] for u in sorted(set([0,m.old.cam.evaluate(t)['contactTravelMm']]))]:
   cass,removed=s.build(math.degrees(angle),z,u);other=[(n,p,k) for n,p,k in base if n not in removed]+[(n,p,k) for n,p,k,_ in cass];hits=[]
   for n,p,k in m.local_parts(theta,u):
    p=h.pitch(p,angle,[c['stageCenterX'],0,z]);b=s.a.assembly.robot.bounds(p)
    for nn,q,kk in other:
     a=s.a.assembly.robot.bounds(q)
     if min(min(a[i+3],b[i+3])-max(a[i],b[i]) for i in range(3))<=1e-6:continue
     v=common(p,q)
     if v>1e-5:hits.append({'a':n,'b':nn,'commonMm3':v})
   rows.append({'pose':state,'theta':theta,'uMm':u,'hits':hits});print(state,theta,hits,flush=True)
 for p,sha in pins.items():
  if hashlib.sha256((s.a.ROOT/p).read_bytes()).hexdigest()!=sha:raise ValueError('source changed')
 return {'inputs':pins,'rows':rows,'adopted':False,'limits':['Sampled geometry only, no load or dynamics acceptance','No native servo or horn source geometry redistributed','Contact travel is a unilateral boundary, not prescribed actual displacement']}
if __name__=='__main__':
 r=run();p=s.a.ROOT/'cad/assembly/evidence/servo-opening-wide-support.json';p.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8',newline='\n')
