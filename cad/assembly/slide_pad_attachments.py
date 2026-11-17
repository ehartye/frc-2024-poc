"""Original detachable slide-pad backing brackets. Millimetres; nominal geometry only."""
import math,json,pathlib
import upper_carriage_connections as upper
lower=upper.lower; assembly=upper.assembly; h=upper.h
from OCP.BRepAlgoAPI import BRepAlgoAPI_Fuse
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCone
from OCP.gp import gp_Ax2

RECESS=0.5
HEAD_D=0.307*25.4  # Pencom standard maximum, not a measured am-1570 head
SHANK_D=0.138*25.4
CONE_H=(HEAD_D-SHANK_D)/2/math.tan(math.radians(41))

def transform(shape,seat,axis):
 t=h.gp_Trsf()
 if axis[0]:t.SetRotation(h.gp_Ax1(h.gp_Pnt(0,0,0),h.gp_Dir(0,1,0)),axis[0]*math.pi/2)
 else:t.SetRotation(h.gp_Ax1(h.gp_Pnt(0,0,0),h.gp_Dir(1,0,0)),-axis[1]*math.pi/2)
 return h.move(h.BRepBuilderAPI_Transform(shape,t,True).Shape(),*seat)

def pad_stack(name,seat,axis):
 # Local +Z runs outward from sliding face. Length includes flat head.
 head=BRepPrimAPI_MakeCone(HEAD_D/2,SHANK_D/2,CONE_H).Shape()
 head=assembly.robot.cut(head,lower.fasteners.hex_prism(0,0,-.1,1.984375,1.6764))
 shaft=h.cyl((0,0,CONE_H),(0,0,1),SHANK_D/2,15.875-CONE_H)
 nut=lower.fasteners.hex_prism(0,0,9.5,7.9375,3.556)
 nut=assembly.robot.cut(nut,h.cyl((0,0,9.4),(0,0,1),SHANK_D/2,3.756))
 return [(name+' '+n,transform(s,seat,axis),'selected-fastener-dimensional-proxy') for n,s in [('head',head),('shaft',shaft),('nut',nut)]]

def apply(parts,zc,c=None):
 c=c or json.loads((assembly.ROOT/'cad/assembly/intake_components.json').read_text())
 shapes={n:s for n,s,k in parts}; extra=[]; joints=[]; gx=c['guide']['x']
 for side in [-1,1]:
  gy=side*c['guide']['yAbs'];carriage='intake candidate lift carriage plate '+str(side)
  for zz in [zc+205,zc+265]:
   for xx in [-1,1]:
    pad='intake candidate UHMW slide pad '+str((side,zz,xx))
    face=gx+xx*12.85; outer=face+xx*6; backlo=min(outer,outer+xx*4)
    ya,yb=sorted([gy-side*10,gy+side*20]); footlo=gy+16.825 if side>0 else gy-20
    bx=gx+(-13.6 if xx<0 else 13.4)
    flo,fhi=sorted([backlo if xx>0 else backlo+4,bx-7 if xx>0 else bx+7])
    # Machined single-piece L bracket, not an unqualified welded joint.
    plate=h.box(backlo,ya,zz-2,4,yb-ya,24)
    plate=BRepAlgoAPI_Fuse(plate,h.box(flo,footlo,zz-2,fhi-flo,3.175,24)).Shape()
    name='original pad backing '+str((side,zz,xx));hardware=[]
    for z in [zz+5,zz+15]:
     seat=(face+xx*RECESS,gy,z)
     through=h.cyl((face-xx,gy,z),(xx,0,0),1.9,12)
     # Constant-radius recess then matching 82-degree countersink.
     recess=h.cyl((face-xx*.1,gy,z),(xx,0,0),HEAD_D/2,RECESS+.1)
     cone=BRepPrimAPI_MakeCone(gp_Ax2(h.gp_Pnt(*seat),h.gp_Dir(xx,0,0)),HEAD_D/2,1.9,(HEAD_D/2-1.9)/math.tan(math.radians(41))).Shape()
     for tool in [through,recess,cone]:shapes[pad]=assembly.robot.cut(shapes[pad],tool)
     plate=assembly.robot.cut(plate,through)
     hardware+=pad_stack(name+' pad '+str(z),seat,(xx,0,0))
    for z in [zz+4,zz+16]:
     seat=(bx,gy+side*16.825,z); bore=h.cyl((bx,gy+side*15.825,z),(0,side,0),2.667,11.525)
     plate=assembly.robot.cut(plate,bore);shapes[carriage]=assembly.robot.cut(shapes[carriage],bore)
     rows=lower.fasteners.stack(name+' carriage '+str(z),0,0,0,-9.525,19.05,button=True)
     hardware += [(n,transform(s,seat,(0,-side,0)),k) for n,s,k in rows]
    extra.append((name,plate,'original-machined-6061-assumed'));extra+=hardware
    joints.append({'name':name,'pad':pad,'carriage':carriage,'wearBeforeHeadExposureMm':RECESS,'guideClearanceMm':.15,'remainingPadBelowCountersinkMm':6-RECESS-CONE_H,'padThreadBeyondNutMm':15.875-9.5-3.556,'motion':'carriage translation','carriageScrewAxisXmm':bx})
 return [(n,shapes[n],k) for n,s,k in parts]+extra,joints


def run():
 import hashlib,audit_intake_motion
 from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
 from OCP.BRepGProp import BRepGProp
 from OCP.GProp import GProp_GProps
 def pins():
  files=['cad/assembly/slide_pad_attachments.py','cad/assembly/upper_carriage_connections.py','cad/assembly/lower_support_mounts.py','cad/assembly/audit_intake_motion.py','cad/electrical/fasteners.py']
  return {'assembly':assembly.input_pins(),'dependencies':{f:hashlib.sha256((assembly.ROOT/f).read_bytes()).hexdigest() for f in files}}
 before=pins();c=json.loads((assembly.ROOT/'cad/assembly/intake_components.json').read_text());base=assembly.build('floor');rows=[]
 for phase,deg,zc in audit_intake_motion.poses(c):
  posed=[p for p in base if not p[0].startswith('intake candidate')]+assembly.intake_components.build(c=c,pose=(deg,zc))
  posed,_=lower.apply(posed,c);posed,_=upper.apply(posed,zc,c);parts,joints=apply(posed,zc,c)
  bounds={n:assembly.robot.bounds(s) for n,s,k in parts};hits=[]
  for i,(n,s,k) in enumerate(parts[len(posed):]):
   if not h.BRepCheck_Analyzer(s).IsValid():raise ValueError('invalid added shape '+n)
   for fn,fs,fk in parts[:len(posed)+i]:
    a=bounds[n];b=bounds[fn]
    if min(min(a[j+3],b[j+3])-max(a[j],b[j]) for j in range(3))<=1e-5:continue
    op=BRepAlgoAPI_Common(s,fs)
    if not op.IsDone() or not h.BRepCheck_Analyzer(op.Shape()).IsValid():raise ValueError('invalid common operation')
    gp=GProp_GProps();BRepGProp.VolumeProperties_s(op.Shape(),gp);v=gp.Mass()
    if v < -1e-7:raise ValueError('negative common volume')
    if v>1e-3:hits.append({'a':n,'b':fn,'commonMm3':v})
  rows.append({'phase':phase,'pitchDeg':deg,'centerZ':zc,'hits':hits})
 if before!=pins():raise ValueError('source changed during run')
 return {'schema':'slide-pad-attachments/1','sourcePins':before,'addedPartCount':len(parts)-len(posed),'joints':joints,'samples':rows,'mechanicalAcceptance':False,'limits':['Standard-dimensional screw proxies, exact purchased head geometry unverified','6061 original bracket material hypothesis; no preload, creep, wear or structural qualification','0.5 mm nominal wear before head exposure, not an allowable service interval','Install pad screws before installing guide; in-place driver access not available','15 finite poses on pinned assembly, not continuous motion proof','No existing assembly conflicts are waived by this attachment-only screen']}

if __name__=='__main__':
 import sys
 result=run();out=pathlib.Path(sys.argv[1]) if len(sys.argv)>1 else assembly.ROOT/'artifacts/pad-attachments.json'
 out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8',newline='\n')
 print(json.dumps({'poses':len(result['samples']),'added':result['addedPartCount'],'hits':[x for r in result['samples'] for x in r['hits']]}))
