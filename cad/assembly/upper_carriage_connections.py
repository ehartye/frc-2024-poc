"""Additive original upper-portal gussets and carriage/drop-bracket yokes."""
import json,math,pathlib,hashlib
import lower_support_mounts as lower
assembly=lower.assembly;h=lower.h
from OCP.BRepAlgoAPI import BRepAlgoAPI_Fuse

def stack_along(name,seat,axis,total):
 rows=lower.fasteners.stack(name,0,0,0,-total,19.05)
 tr=h.gp_Trsf()
 if axis[0]:tr.SetRotation(h.gp_Ax1(h.gp_Pnt(0,0,0),h.gp_Dir(0,1,0)),axis[0]*math.pi/2)
 else:tr.SetRotation(h.gp_Ax1(h.gp_Pnt(0,0,0),h.gp_Dir(1,0,0)),-axis[1]*math.pi/2)
 return [(n,h.move(h.BRepBuilderAPI_Transform(s,tr,True).Shape(),*seat),k) for n,s,k in rows]

def apply(parts,zc,c=None):
 if len({n for n,s,k in parts})!=len(parts):raise ValueError('duplicate part identifiers')
 c=c or json.loads((assembly.ROOT/'cad/assembly/intake_components.json').read_text());shapes={n:s for n,s,k in parts};extra=[];joints=[]
 gx=c['guide']['x'];top=c['replacementPortalTopZ']
 for side in [-1,1]:
  gy=side*c['guide']['yAbs'];guide='intake candidate stationary aluminum guide '+str(side);portal='original replacement upper transverse portal'
  name='original upper portal gusset '+str(side);plate=h.box(gx+12.7,gy-25.4,top-50.8,3.175,50.8,88.9);hardware=[]
  for target,yy,zz in [(guide,gy,top-25.4),(guide,gy,top-12.7),(portal,gy-12.7,top+25.4),(portal,gy+12.7,top+25.4)]:
   seat=(gx+15.875,yy,zz);bore=h.cyl((gx+8.525,yy,zz),(1,0,0),2.667,8.35)
   plate=assembly.robot.cut(plate,bore);shapes[target]=assembly.robot.cut(shapes[target],bore)
   hardware+=stack_along(name+str((yy,zz)),seat,(1,0,0),6.35)
  extra.append((name,plate,'original-drilled-gusset'));extra+=hardware;joints.append({'name':name,'connects':[guide,portal],'motion':'fixed','stackMm':6.35})
  # A proposed welded U-shaped yoke bridges the real lateral gap between plates.
  x=c['stageCenterX'];drop='intake candidate pivot drop bracket '+str(side);carriage='intake candidate lift carriage plate '+str(side)
  near=side*(c['pivotBracketYAbs']+6.35);far=gy+20 if side>0 else gy-20
  lo,hi=sorted([near,far]);z0=zc+c['carriageOffsetMm']+30
  yoke=h.box(x-17,lo,z0+21.825,34,hi-lo,3.175)
  for y in [lo,hi-3.175]:yoke=BRepAlgoAPI_Fuse(yoke,h.box(x-17,y,z0,34,3.175,25)).Shape()
  name='original carriage drop yoke '+str(side);hardware=[]
  for target,face,d in [(drop,near-side*6.35,-side),(carriage,far-side*3.175,-side)]:
   for xx in [x-8.5,x+8.5]:
    zz=z0+10;seat=(xx,face,zz);bore=h.cyl((xx,face-d*10.525,zz),(0,d,0),2.667,11.525)
    yoke=assembly.robot.cut(yoke,bore);shapes[target]=assembly.robot.cut(shapes[target],bore)
    hardware+=stack_along(name+str((xx,face)),seat,(0,d,0),9.525)
  extra.append((name,yoke,'original-drilled-yoke'));extra+=hardware;joints.append({'name':name,'connects':[drop,carriage],'motion':'carriage','stackMm':9.525})
 return [(n,shapes[n],k) for n,s,k in parts]+extra,joints

def run():
 import audit_intake_motion
 before=assembly.input_pins();script=hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest();dependency=hashlib.sha256(pathlib.Path(lower.__file__).read_bytes()).hexdigest()
 c=json.loads((assembly.ROOT/'cad/assembly/intake_components.json').read_text());rows=[];last=None
 fixedbase=[p for p in assembly.build('floor') if not p[0].startswith('intake candidate')]
 for phase,deg,zc in audit_intake_motion.poses(c):
  # Rebuild pose using the explicit kinematic module, not endpoint interpolation.
  base=fixedbase+assembly.intake_components.build(c=c,pose=(deg,zc))
  lowerparts,_=lower.apply(base,c);parts,joints=apply(lowerparts,zc,c);new=parts[len(lowerparts):];bounds={n:assembly.robot.bounds(s) for n,s,k in parts};hits=[]
  for i,(n,s,k) in enumerate(new):
   for fn,fs,fk in parts[:len(lowerparts)+i]:
    a=bounds[n];b=bounds[fn]
    if min(min(a[j+3],b[j+3])-max(a[j],b[j]) for j in range(3))<=1e-5:continue
    v=lower.volume(s,fs)
    if v>1e-3:hits.append({'a':n,'b':fn,'commonMm3':v})
  rows.append({'phase':phase,'pitchDeg':deg,'centerZ':zc,'hits':hits});last=(parts,new,joints)
 if before!=assembly.input_pins() or script!=hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest() or dependency!=hashlib.sha256(pathlib.Path(lower.__file__).read_bytes()).hexdigest():raise RuntimeError('sources changed')
 return {'schema':'upper-carriage-connections/1','assemblyInputs':before,'scriptSha256':script,'lowerSupportScriptSha256':dependency,'addedParts':len(last[1]),'joints':last[2],'samples':rows,'acceptance':False,'limits':['Additive candidate, not canonical export','Original plate/weldment and nominal fastener geometry; no load or tool-access qualification','15 finite poses, not continuous collision proof','Existing raw drive-envelope engagement hypotheses unchanged','Chain attachment and shaft torque transfer remain incomplete']}
if __name__=='__main__':
 q=run();out=assembly.ROOT/'artifacts/intake-component-candidate/upper-carriage-connections.json';out.write_text(json.dumps(q,indent=2)+'\n',encoding='utf-8',newline='\n');print(json.dumps({'poses':len(q['samples']),'added':q['addedParts'],'hits':[(r['phase'],r['pitchDeg'],h) for r in q['samples'] for h in r['hits']]}))
