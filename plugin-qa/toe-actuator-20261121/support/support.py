from pathlib import Path
import math,sys,json,hashlib,runpy
import numpy as np
from OCP.gp import gp_Pnt,gp_Dir,gp_Ax2,gp_Trsf
from OCP.TopLoc import TopLoc_Location
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox,BRepPrimAPI_MakeCylinder
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut,BRepAlgoAPI_Fuse
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopoDS import TopoDS
from OCP.STEPControl import STEPControl_Writer,STEPControl_AsIs
q=Path(__file__).parent;prev=q.parent;w=runpy.run_path(str(prev/'rack-fit.py'));b=runpy.run_path(str(prev/'bounds-fit.py'));sys.path.insert(0,str(Path.home()/'repos/agent-frc/scripts'))
from stock_sections import entities
from mechanical_motion import bounds,common,distance
from showcase_contacts import load_solids
def box(lo,hi):return BRepPrimAPI_MakeBox(gp_Pnt(*lo),gp_Pnt(*hi)).Shape()
def cyl(point,direction,r,depth):return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(*point),gp_Dir(*direction)),r,depth).Shape()
def xf(part,R,d):
 g=gp_Trsf();g.SetValues(*np.column_stack([R,d]).ravel());return part.Moved(TopLoc_Location(g))
def cut(part,tool):return BRepAlgoAPI_Cut(part,tool).Shape()
def union(parts):
 out=parts[0]
 for other in parts[1:]:out=BRepAlgoAPI_Fuse(out,other).Shape()
 return out
def vecs(a,b):return np.minimum(a,b).tolist(),np.maximum(a,b).tolist()
def rect(a,b):return box(*vecs(a,b))
def GP(part):
 gp=GProp_GProps();BRepGProp.VolumeProperties_s(part,gp,True);return gp.Mass(),np.array(gp.CentreOfMass().Coord())
def headRows():
 rows=[]
 def add(n,shape,mat='6061',j=None,m=None):rows.append(dict(name=n,shape=shape,material=mat,j=j,m=m));return shape
 back=box([-110,-100,89],[16,100,101]);old=back
 for sig in (-1,1):
  back=cut(back,rect([4,sig*49,88],[17,sig*101,102]))
 back=cut(back,box([-111,-7,88],[-77,7,102]))
 back=cut(back,box([-84,-7,88],[-16,7,102])) # central toe passes through deck when withdrawn
 # Steel wrist journals through two outboard machined ears retained to central plate by positive base shoulder.
 for sig in [-1,1]:
  yc=87*sig;lug=box([-82,yc-3,101],[-58,yc+3,207])
  lug=cut(lug,cyl([-70,yc-4,195],[0,1,0],6.45,8));foot=box([-88,yc-9,101],[-52,yc+9,105])
  # Ear and foot form a single machined stock. Four through fasteners; end supports provisional.
  lug=union([lug,foot])
  for x in [-83,-57]:
   for y in [yc-5.5,yc+5.5]:
    bore=cyl([x,y,88],[0,0,1],1.7,18);lug=cut(lug,bore);back=cut(back,bore)
  add('wrist journal foot '+str(sig),lug)
 # Shoulder geometry integrated in each guide tower. Coordinates are longitudinal guide stations/cross axis.
 RR=np.eye(3)
 parts=w['make'](rx=-106,rz=200);stock={}
 for p in parts:
  if p.s not in stock:
   src=next(prev.glob(p.s+'.[sS][tT][eE][pP]'));stock[p.s]=list(load_solids(src)[0].values())
  mass={'REV-21-1651':142,'REV-41-1602':34,'REV-41-1615':66.5}
  mats={'REV-41-1332':'POM','REV-41-1503':'PA6','REV-41-1607':'6061','REV-41-1608':'steel'}
  vols=[GP(s)[0] for s in stock[p.s]];total=sum(vols)
  for idx,src in enumerate(stock[p.s]):add(p.name+' leaf '+str(idx),xf(src,p.A,p.t),mats.get(p.s,'purchased'),p.k,mass[p.s]*vols[idx]/total if p.s in mass else None)
 bearing=load_solids(q/'bearing.step')[0]
 # Axis arrangement: local Z shaft & X vertical, actual long direction lateral uniformly (perpendicular to toe-height separation).
 for k,ct,axis,rr,span in [('side',[-4,0,132],[1,0,0],np.array([[0,0,1],[0,-1,0],[1,0,0]]),24),('rear',[-106,-25,200],[0,1,0],np.array([[0,1,0],[0,0,1],[1,0,0]]),24)]:
  cen=np.array(ct);ax=np.array(axis);assert np.linalg.det(rr)>.99
  for sign in [-1,1]:
   base=cen+ax*(span/2*sign-2.25)
   for name,s in bearing.items():add(k+' bearing '+str(sign)+' '+name,xf(s,rr,base),'steel')
   # Stationary alloy cheeks clamp a preassembled block at its back face.
   normalSign=sign; inner=cen+ax*sign*(12+2.25)
   if k=='side':
    # Aft plate is limited by the gearbox face behind it at X=-21.
    thickness=2.5 if sign<0 else 4
    x=inner[0];lo,hi=vecs([x,-18,101],[x+sign*thickness,18,144]);cheek=box(lo,hi)
    cheek=cut(cheek,cyl([lo[0]-1,0,132],[1,0,0],8.0,thickness+2))
    for y in [-13.8564,13.8564]:
     for z in [-8,8]:cheek=cut(cheek,cyl([lo[0]-1,y,132+z],[1,0,0],1.7,thickness+2))
   else:
    y=inner[1]; thickness=2.5 if sign<0 else 4
    lo,hi=vecs([-123,y,101],[-89,y+sign*thickness,212]);cheek=box(lo,hi)
    cheek=cut(cheek,cyl([-106,lo[1]-1,200],[0,1,0],8,thickness+2))
    for x in [-119.8564,-92.1436]:
     for z in [-8,8]:cheek=cut(cheek,cyl([x,lo[1]-1,200+z],[0,1,0],1.7,thickness+2))
   add(k+' cheek '+str(sign),cheek)
  # Equivalent 5 mm hex with bearing datums, plus the needed stock extension into output.
  from OCP.BRepBuilderAPI import BRepBuilderAPI_MakePolygon,BRepBuilderAPI_MakeFace
  from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
  from OCP.gp import gp_Vec
  pg=BRepBuilderAPI_MakePolygon()
  for i in range(6):
   a=i*math.pi/3;pg.Add(gp_Pnt(5/math.sqrt(3)*math.cos(a),5/math.sqrt(3)*math.sin(a),-26))
  pg.Close();sh=BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(pg.Wire()).Face(),gp_Vec(0,0,42)).Shape();add(k+' shaft blank',xf(sh,rr,cen),'steel')
 # Adjustable backed C guides in head coordinates. For sides local U = outward along joint, V = +Z, W = +X; for rear U=-X, V=+Z, W=+Y.
 for label,sign,zh,stations,lenS in [('left',1,153.45,[-1,21],2),('right',-1,110.55,[-1,21],2),('rear',0,221.45,[87,118],3)]:
  rear=sign==0; front= -41 if rear else 52.25
  # positive U is direction of opening. Backing reaches actual posts via a machined outboard lug or the offset rear elbow.
  def piece(u0,u1,v0,v1,w0,w1):
   if rear:return rect([-u0,-19+w0,zh+v0],[-u1,-19+w1,zh+v1])
   return rect([2+w0,sign*u0,zh+v0],[2+w1,sign*u1,zh+v1])
  L=w['D'];low,high=(41,41+3*L) if rear else (front-2*L,front)
  slider=piece(low,high,-4.5,4.5,3,6)
  # Standoffs on both real 3.4 mm slotted holes per segment, captured from the stable mounting-pad face.
  for i in range(lenS):
   base=41+L*(i+.5) if rear else front-L*(i+.5)
   for ud in [base-16,base+16]:
    p=np.array([-ud,-19,zh]) if rear else np.array([2,sign*ud,zh]);n=np.array([0,1,0]) if rear else np.array([1,0,0])
    # Native slot ends source W0.25..2.75, small rounded lead-in. Clamp through it from outboard.
    slider=union([slider,cyl(p,n,3.25,3.02)])
    slider=cut(slider,cyl(p-n*.25,n,1.7,7))
  if not rear:
   # Outboard tongue joins the actual post in Y; put vertical drop over the same post and leave 2 mm overlap into the toe post.
   slider=union([slider,piece(high-3,80,-4.5,4.5,3,6),piece(75,81,-(zh-89),3,3,8)])
  else:
   # Rear toe sweeps through X=-70. Rise aft of wrist shaft, using a low fore/aft bridge to the pickup post.
   # At full retraction vertical link is near X=-87, maintaining independent capture of the original toe.
   slider=union([slider,box([-144,-16,zh-4],[-136,2,zh+4]),box([-144,-2,82],[-136,2,zh+3]),box([-144,-3,82],[-72,3,87])])
  add(label+' moving rack backer and finger link',slider,'6061',label)
  for j,u in enumerate(stations):
   # Replaceable POM saddle; tongue runs with a nominal 0.20 mm per face. Enclosing roof and two side lips carry outward/inward loads.
   gib=piece(u-5,u+5,-7,7,1.6,10)
   gib=cut(gib,piece(u-6,u+6,-4.7,4.7,1.55,6.2))
   add(label+' stationary POM guide '+str(j),gib,'POM')
   # Simple bracket contiguous with the gib's stationary back face; physically seated on deck. Rear bracket requires an aft outrigger.
   block=piece(u-5,u+5,101-zh,7,10,13)
   add(label+' guide stanchion '+str(j),block)
 # Witness switches mounted on the fixed wrist, three physical finger posts form observed actuators.
 sw=next(iter(load_solids(q/'switch.step')[0].values()))
 for label,orientation,offset in [('left',[[1,0,0],[0,0,-1],[0,1,0]],[-.93,40.92,75]),('right',[[1,0,0],[0,0,1],[0,-1,0]],[-.93,-40.92,75]),('rear',[[0,0,1],[1,0,0],[0,1,0]],[-7.92,-10.93,75])]:
  R=np.array(orientation);t=np.array(offset);add(label+' home switch',xf(sw,R,t),'purchased',None,2.)
  center=t+R@np.array([0,0,2.71]);plate=box([center[0]-7,center[1]-7,78.25],[center[0]+7,center[1]+7,89]); 
  for hx in [-4.76,4.76]:
   site=t+R@np.array([hx,0,2.71]);hole=cyl([*site[:2],78],[0,0,1],1.1,24);plate=cut(plate,hole);back=cut(back,hole)
  add(label+' limit switch saddle',plate)
 add('slotted head backbone',back)
 return rows,old
if __name__=='__main__':
 rows,old=headRows();H=hashlib.sha256
 def extract(row,stroke):
  lab=row['j'];t=[0,0,0] if not lab else ({'rear':[-stroke,0,0],'left':[0,stroke,0],'right':[0,-stroke,0]}[lab]);return xf(row['shape'],np.eye(3),t)
 result=[];wrt=STEPControl_Writer()
 for row in rows:
  v,c=GP(row['shape']); assert BRepCheck_Analyzer(row['shape']).IsValid(),row['name']
  solids=len(list(entities(row['shape'],TopAbs_SOLID)))
  m=row['m'] if row['m'] is not None else v*{'6061':.00270,'POM':.00141,'PA6':.00113,'steel':.00785}[row['material']]
  result.append({k:v for k,v in row.items() if k not in ['shape','m']}|dict(volumeMm3=v,massG=m,cgMm=c.tolist(),bbox=bounds(row['shape']),solids=solids));wrt.Transfer(row['shape'],STEPControl_AsIs)
 wrt.Write(str(q/'wrist-local.step'))
 (q/'roster.json').write_text(json.dumps(result,indent=2))
 print('solids',len(rows),'mass',sum(r['massG']for r in result),'disconnected',[(r['name'],r['solids'])for r in result if r['solids']!=1],flush=True)

