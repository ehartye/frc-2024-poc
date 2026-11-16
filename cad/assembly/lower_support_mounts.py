"""Additive original drilled lower-post brackets; not yet the canonical export."""
import math,json,pathlib,hashlib
import build_intake_candidate as assembly
from OCP.BRepAlgoAPI import BRepAlgoAPI_Fuse,BRepAlgoAPI_Common
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
import fasteners
h=assembly.robot.h

def axis_stack(name,seat,direction):
 rows=fasteners.stack(name,0,0,0,-6.35,19.05)
 tr=h.gp_Trsf();tr.SetRotation(h.gp_Ax1(h.gp_Pnt(0,0,0),h.gp_Dir(1,0,0)),-direction*math.pi/2)
 return [(n,h.move(h.BRepBuilderAPI_Transform(s,tr,True).Shape(),*seat),k) for n,s,k in rows]

def apply(parts,c=None):
 c=c or json.loads((assembly.ROOT/'cad/assembly/intake_components.json').read_text())
 shapes={n:s for n,s,k in parts};roles={n:k for n,s,k in parts};extra=[];joints=[]
 bottom=c['guide']['bottom']
 for side in [-1,1]:
  for kind,x,y,d,post,bridge in [
   ('guide',c['guide']['x'],side*c['guide']['yAbs'],side,'intake candidate stationary aluminum guide '+str(side),'frame fastening bridge-0'),
   ('shooter',279.4,side*c['shooterSupportYAbs'],-side,f'original shooter upright {side} 279.4','frame fastening bridge-1')]:
   name=f'original lower {kind} bracket {side}'
   ya,yb=sorted([y+d*12.7,y+d*15.875]);fa,fb=sorted([y+d*12.7,y+d*41.275])
   leg=h.box(x-12.7,ya,bottom,25.4,yb-ya,38.1)
   foot=h.box(x-12.7,fa,bottom,25.4,fb-fa,3.175)
   bracket=BRepAlgoAPI_Fuse(leg,foot).Shape();hardware=[]
   for z in [bottom+12.7,bottom+25.4]:
    seat=(x,y+d*15.875,z);bore=h.cyl((x,seat[1]-d*7.35,z),(0,d,0),2.667,8.35)
    bracket=assembly.robot.cut(bracket,bore);shapes[post]=assembly.robot.cut(shapes[post],bore)
    hardware+=axis_stack(name+' leg '+str(z),seat,d)
   for distance in [22.225,34.925]:
    xx=x;yy=y+d*distance;bore=h.cyl((xx,yy,bottom-4.175),(0,0,1),2.667,8.35)
    bracket=assembly.robot.cut(bracket,bore);shapes[bridge]=assembly.robot.cut(shapes[bridge],bore)
    hardware+=fasteners.stack(name+' foot '+str(distance),xx,yy,bottom+3.175,bottom-3.175,19.05)
   extra.append((name,bracket,'original-drilled-bracket'));extra+=hardware
   joints.append({'bracket':name,'post':post,'bridge':bridge,'materialStackMm':6.35,'screw':'am-1047 #10-32 x 0.75in nominal','nut':'am-1042 nominal','installation':'Fit internal nuts through open stock ends before upper portal/rail closure; tool access not qualified'})
 return [(n,shapes[n],roles[n]) for n,s,k in parts]+extra,joints

def volume(a,b):
 p=GProp_GProps();BRepGProp.VolumeProperties_s(BRepAlgoAPI_Common(a,b).Shape(),p);return p.Mass()

def run():
 before=assembly.input_pins();script=hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest()
 base=assembly.build('floor');parts,joints=apply(base);new=parts[len(base):];bounds={n:assembly.robot.bounds(s) for n,s,k in parts};hits=[]
 for i,(n,s,k) in enumerate(new):
  for fn,fs,fk in parts[:len(base)+i]:
   a=bounds[n];b=bounds[fn]
   if min(min(a[j+3],b[j+3])-max(a[j],b[j]) for j in range(3))<=1e-5:continue
   v=volume(s,fs)
   if v>1e-3:hits.append({'a':n,'b':fn,'commonMm3':v})
 motion=[]
 import audit_intake_motion
 c=json.loads((assembly.ROOT/'cad/assembly/intake_components.json').read_text())
 for phase,deg,z in audit_intake_motion.poses(c):
  overlaps=[]
  for moving in assembly.intake_components.build(c=c,pose=(deg,z),metadata=True):
   if moving['motion']=='fixed':continue
   a=assembly.robot.bounds(moving['shape'])
   for n,s,k in new:
    b=bounds[n]
    if min(min(a[j+3],b[j+3])-max(a[j],b[j]) for j in range(3))<=1e-5:continue
    v=volume(moving['shape'],s)
    if v>1e-3:overlaps.append({'moving':moving['name'],'hardware':n,'commonMm3':v})
  motion.append({'phase':phase,'pitchDeg':deg,'centerZ':z,'hits':overlaps})
 out=assembly.ROOT/'artifacts/intake-component-candidate/lower-support-hardware.step'
 writer=h.STEPControl_Writer()
 for n,s,k in new:
  if not h.BRepCheck_Analyzer(s).IsValid():raise RuntimeError(n)
  writer.Transfer(s,h.STEPControl_AsIs)
 if writer.Write(str(out))!=h.IFSelect_RetDone:raise RuntimeError('export failed')
 reader=h.STEPControl_Reader()
 if reader.ReadFile(str(out))!=h.IFSelect_RetDone:raise RuntimeError('reopen failed')
 reader.TransferRoots()
 if reader.NbRootsForTransfer()!=len(new):raise RuntimeError('wrong hardware root count')
 # A nominal first leg screw against the undrilled post is the blocked-load-path control.
 screw=next(s for n,s,k in new if 'guide bracket -1 leg' in n and n.endswith('-shaft'))
 original=next(s for n,s,k in base if n=='intake candidate stationary aluminum guide -1')
 if before!=assembly.input_pins() or script!=hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest():raise RuntimeError('sources changed')
 return {'schema':'lower-support-bracket-candidate/1','assemblyInputs':before,'scriptSha256':script,'hardwareStepSha256':hashlib.sha256(out.read_bytes()).hexdigest(),'hardwareReopenedRoots':reader.NbRootsForTransfer(),'addedParts':len(new),'joints':joints,'rawIntersections':hits,'movingHardwareSamples':motion,'undrilledPostNegativeMm3':volume(screw,original),'acceptance':False,'limits':['Additive candidate, not canonical STEP export','Nominal proxy screws/nuts without threads or preload','Added hardware checked against floor assembly and moving parts at15poses; not continuous motion','Upper portal, carriage yoke, chain anchors and shaft torque coupling incomplete','Installation reach, edge distances, bracket bending and impact loads unqualified']}
if __name__=='__main__':
 q=run();out=assembly.ROOT/'artifacts/intake-component-candidate/lower-support-mounts.json';out.write_text(json.dumps(q,indent=2)+'\n',encoding='utf-8');print(json.dumps({'addedParts':q['addedParts'],'hits':q['rawIntersections'],'negativeMm3':q['undrilledPostNegativeMm3'],'movingCollisionSamples':sum(bool(r['hits']) for r in q['movingHardwareSamples'])}))
