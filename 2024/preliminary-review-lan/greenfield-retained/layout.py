"""Fresh pivoting belt launcher. All design dimensions in mm. Run from this directory."""
from pathlib import Path
from math import sin,cos,radians,sqrt
import sys,json,hashlib,itertools
import numpy as np
from OCP.gp import gp_Pnt,gp_Dir,gp_Ax2,gp_Vec,gp_Trsf
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox,BRepPrimAPI_MakeCylinder,BRepPrimAPI_MakePrism,BRepPrimAPI_MakeTorus
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakePolygon,BRepBuilderAPI_MakeFace,BRepBuilderAPI_Transform
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut,BRepAlgoAPI_Fuse
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.STEPCAFControl import STEPCAFControl_Writer
from OCP.STEPControl import STEPControl_AsIs
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString as TS
from OCP.TDataStd import TDataStd_Name
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.IFSelect import IFSelect_RetDone
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent))
from tube import lead
sys.path.insert(0,'C:/Users/ehart/repos/agent-frc/scripts')
from mechanical_motion import bounds,common,distance
from package_mesh import surface_mesh
from collections import Counter
G={}; meta={}; moving=[]; held='Chambered NOTE'; park=125; pick=-50
pivot=(185,0,501)
slide=[]
def box(x,y,z,dx,dy,dz):return BRepPrimAPI_MakeBox(gp_Pnt(x,y,z),dx,dy,dz).Shape()
def cyl(p,axis,r,length):return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(*p),gp_Dir(*axis)),r,length).Shape()
def cut(a,b):return BRepAlgoAPI_Cut(a,b).Shape()
def fuse(a,b):return BRepAlgoAPI_Fuse(a,b).Shape()
def prism(points,delta):
    wire=BRepBuilderAPI_MakePolygon()
    for p in points:wire.Add(gp_Pnt(*p))
    wire.Close()
    return BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(wire.Wire()).Face(),gp_Vec(*delta)).Shape()
def ringprofile(p,wall,z,h):
    # Offset each convex edge inward and intersect adjacent offset lines.
    pts=np.array(p,dtype=float); edges=np.roll(pts,-1,axis=0)-pts
    normal=np.stack((-edges[:,1],edges[:,0]),axis=1)/np.linalg.norm(edges,axis=1)[:,None]
    offsets=(normal*pts).sum(axis=1)+wall
    q=[np.linalg.solve(normal[[i-1,i]],offsets[[i-1,i]]) for i in range(len(p))]
    return cut(prism([(*a,z) for a in p],(0,0,h)),prism([(*a,z-1) for a in q],(0,0,h+2)))
def mat(theta,x,y,z):
    a=radians(theta);return np.array([[cos(a),0,-sin(a),x],[0,1,0,y],[sin(a),0,cos(a),z],[0,0,0,1.]])
def place(sh,m):
    t=gp_Trsf();t.SetValues(*m[:3].ravel());return BRepBuilderAPI_Transform(sh,t,True).Shape()
def add(n,shape,g,cat='structure',material='6061 aluminum allocation',local=False):
    e=TopExp_Explorer(shape,TopAbs_SOLID);assert e.More(),n
    sh=e.Current();e.Next();assert not e.More() and BRepCheck_Analyzer(sh).IsValid(),n
    if local:moving.append(n);sh=place(sh,mat(park,*pivot))
    G[n]=sh;meta[n]=dict(category=cat,group=g,material=material)
def local(n,s,g='Pivoting cassette',cat='mechanism',material='Aluminum'):
    add(n,s,g,cat,material,True)
def ann(p,ax,r,ri,length):return cut(cyl(p,ax,r,length),cyl(p,ax,ri,length))
Y=(0,1,0);Z=(0,0,1);X=(1,0,0)
frame=[(-340,-275),(-285,-330),(340,-330),(340,330),(-285,330),(-340,275)]
add('Chassis perimeter',ringprofile(frame,18,116,30),'Chassis')
# Removable plywood, cushion and covers are represented by one clearance envelope.
def outset(poly,d):
    p=np.array(poly,float);v=np.roll(p,-1,axis=0)-p;n=np.stack((-v[:,1],v[:,0]),axis=1)/np.linalg.norm(v,axis=1)[:,None];k=(n*p).sum(axis=1)-d
    return [tuple(np.linalg.solve(n[[i-1,i]],k[[i-1,i]])) for i in range(len(p))]
add('Blue bumper set',ringprofile(outset(frame,82),82,52,124),'Bumpers','bumper','Combined wood, foam and fabric envelope')
for x,tag in [(-160,'rear'),(150,'front')]:
    add('Bay crossmember · '+tag,box(x,-240,116,20,480,30),'Chassis')
for sign,side in [(1,'left'),(-1,'right')]:
    y=lambda a,b:min(sign*a,sign*b)
    add('Chassis sill · '+side,box(-320,y(240,264),116,642,24,30),'Chassis')
    # A continuous A-shaped plate with a round apex and a triangular window.
    yp=y(248,260)
    outline=[(-141,146),(-110,146),(15,424),(165,424),(253,146),(286,146),(193,442),(-4,442)]
    add('Pivot A-frame · '+side,prism([(x,yp,z) for x,z in outline],(0,12,0)),'Pivot support')
    add('Slide base · '+side,box(-47,yp,442,273,12,7),'Pivot support')
    add('Slide guide · '+side,box(-45,yp+2,449,269,8,6),'Pivot support','mechanism','Profile rail envelope')
    truck=box(165,yp-1,455,40,14,68)
    add('Translating bearing block · '+side,cut(truck,cyl((185,yp-2,501),Y,17,16)),'Pivot support','mechanism','Linear truck and bearing housing')
    add('Pivot bearing · '+side,ann((185,yp-1,501),Y,17,9,14),'Pivot support','mechanism','Steel bearing envelope')
    slide.extend(['Translating bearing block · '+side,'Pivot bearing · '+side])
    local('Cassette cheek · '+side,cut(box(-17,y(218,226),-15,92,8,43),cyl((0,yp-600,0),Y,9,1200)))
    local('Cassette side beam · '+side,box(75,y(218,226),-10,527,8,24))
    local('Entry bearing bracket · '+side,cut(box(602,y(218,226),-3,35,8,32),cyl((625,y(218,226),13),Y,6,8)))
    local('Belt motor shelf · '+side,box(71,y(171,218),-10,52,47,12))
    local('Belt motor · '+side,cyl((97,sign*195.8,-43),Z,18,33),'Cassette drive','electrical','Brushless motor envelope')
    local('Belt drive shaft · '+side,cyl((97,sign*195.8,-10),Z,5,74),'Cassette drive','mechanism','Steel')
    # Direct shaft passes through the shelf.
    shelf='Belt motor shelf · '+side
    G[shelf]=cut(G[shelf],place(cyl((97,sign*195.8,-11),Z,5.4,14),mat(park,*pivot)))
    local('Nose pulley shelf · '+side,cut(box(537,y(172,218),0,42,46,8),cyl((558,sign*195.8,0),Z,5.4,8)))
    local('Idler axle · '+side,cyl((558,sign*195.8,0),Z,5,64),'Cassette drive','mechanism','Steel')
    def capsule(rad):
        yc=sign*195.8
        block=box(97,yc-rad,32,461,2*rad,32)
        return fuse(fuse(block,cyl((97,yc,32),Z,rad,32)),cyl((558,yc,32),Z,rad,32))
    local('Opposed traction belt · '+side,cut(capsule(18),capsule(16)),'Cassette drive','mechanism','Elastomer belt')
    for s,station in [(97,'drive'),(558,'idler')]:
        local('Belt pulley · '+side+' '+station,ann((s,sign*195.8,32),Z,16,5,32),'Cassette drive','mechanism','Polymer/aluminum')
    local('NOTE runner · '+side,box(31,y(100,145),19,578,45,6),'Piece support','structure','UHMW runner')
    for x,suffix in [(249,'front'),(-249,'rear')]:
        label=suffix.title()+' '+side;g='Swerve · '+label
        add(label+' tire',ann((x,y(265,293),50.8),Y,50.8,7,28),g,'mechanism','Wheel and tread envelope')
        add(label+' wheel axle',cyl((x,y(255,303),50.8),Y,7,48),g,'mechanism','Steel')
        add(label+' module deck',box(x-58,y(175,308),104,116,133,12),g,'mechanism','Gearcase / mounting plate')
        for ya in (255,295):
            bl=box(x-16,y(ya,ya+8),37,32,8,67)
            add(label+' axle support '+str(ya),cut(bl,cyl((x,y(ya,ya+8),50.8),Y,7,8)),g,'mechanism','Gearcase')
        for dx,kind in [(-24,'drive'),(24,'steer')]:
            add('Swerve '+kind+' motor · '+label,cyl((x+dx,sign*201,116),Z,20,64),g,'electrical','Motor envelope')
local('Pivot torque shaft',cyl((0,-282,0),Y,9,564),'Pivot support','mechanism','Steel shaft')
add('Pivot reducer',cut(box(140,-301,468,90,40,66),cyl((185,-301,501),Y,9,40)),'Pivot support','mechanism','Right-angle reducer envelope; motor on inboard-facing end')
add('Pivot motor',cyl((78,-281,501),X,28,62),'Pivot support','electrical','Brushless motor')
slide.extend(['Pivot motor','Pivot reducer'])
add('Slide screw',cyl((-49,275,428),X,5,279),'Pivot support','mechanism','Steel lead screw')
add('Slide drive motor',cyl((-102,275,428),X,14,53),'Pivot support','electrical','Geared motor envelope')
for x in [-49,220]:
    add('Slide bearing mount '+str(x),cut(box(x,254,416,10,32,26),cyl((x,275,428),X,5,10)),'Pivot support')
add('Travel nut and clevis',cut(box(178,261,420,14,24,48),cyl((178,275,428),X,5,14)),'Pivot support','mechanism','Threaded nut and offset bracket')
slide.append('Travel nut and clevis')
for pos in (125,370,599):
    local('Runner crossbar '+str(pos),box(pos,-218,14,10,436,5),'Piece support','structure')
local('Rear piece stop',box(28,-48,19,9.2,96,45),'Piece support','mechanism','Padded stop')
local('Nose pickup roller',ann((625,-217,13),Y,12,6,434),'Cassette drive','mechanism','Rubber over aluminum')
local('Nose roller shaft',cyl((625,-233,13),Y,6,466),'Cassette drive','mechanism','Steel')
local('Nose motor',cyl((625,233,13),Y,14,38),'Cassette drive','electrical','Small geared motor; mount allowance open')
add('Electronics deck',box(-190,-248,146,380,496,5),'Electrical bay','structure','Sandwich panel')
add('Battery and terminal case',box(-155,-220,151,185,80,173),'Electrical bay','electrical','Lead-acid battery envelope')
for name,x,y,dx,dy,h in [('PDH',-185,95,165,110,42),('roboRIO flat',-169,-98,149,124,39),('radio',-3,48,105,82,35),('main breaker',45,-207,52,35,34)]:
    add(name,box(x,y,151,dx,dy,h),'Electrical bay','electrical','Purchased electrical envelope')
for name,pts in [('Pack lead',[(19,-186,324),(19,-186,348),(65,-186,348),(65,-186,185)]),('Distribution lead',[(82,-172,177),(82,-150,177),(129,-150,177),(129,174,177),(-20,174,177)])]:
    add(name,lead(pts,4.5,9),'Electrical bay','reserved-space','Cable routing corridor')
# A shared source of geometry for the viewed poses, manufacturing-neutral STEP, and tests.
exec((HERE/'changes.py').read_text(encoding='utf8'))
exec((HERE/'service_layout.py').read_text(encoding='utf8'))
exec((HERE/'stop_support.py').read_text(encoding='utf8'))
exec((HERE/'stop_fastening.py').read_text(encoding='utf8'))
exec((HERE/'strap_bolts.py').read_text(encoding='utf8'))
torus=BRepPrimAPI_MakeTorus(152.4,25.4).Shape()
def pivot_x(angle):
    a=radians(angle);c=cos(a);v=sin(a)
    reach=max(-18*c,643*c)+max(44*v,-67*v)
    return min(185,634-reach)
def centerpose(angle,s,n):
    return mat(angle,pivot_x(angle),0,pivot[2])@mat(0,s,0,n)
seat=centerpose(park,215,50.4)
add(held,place(torus,seat),'Game piece','game-piece','2024 nominal foam NOTE')
def dump(p,o):p.write_text(json.dumps(o,separators=(',',':'),ensure_ascii=False,allow_nan=False),encoding='utf8')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write_step(path):
    doc=TDocStd_Document(TS('BinXCAF'));tab=XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    for n,s in G.items():TDataStd_Name.Set_s(tab.AddShape(s,False),TS(n))
    writer=STEPCAFControl_Writer();assert writer.Transfer(doc,STEPControl_AsIs) and writer.Write(str(path))==IFSelect_RetDone
def viewmesh(s):
    m=surface_mesh(s); points=np.array(m['vertices']);v,idx=np.unique(np.round(points,6),axis=0,return_inverse=True)
    faces=idx[np.array(m['triangles'])];faces=faces[np.array([len(set(f))==3 for f in faces])]
    edges=Counter(tuple(sorted((i,j))) for a,b,c in faces for i,j in [(a,b),(b,c),(c,a)])
    t=v[faces]-v.mean(axis=0);vol=abs(np.einsum('ij,ij->i',t[:,0],np.cross(t[:,1],t[:,2])).sum()/6)
    return dict(v=v.tolist(),f=faces.tolist(),volume=m['volumeMm3'],closed=all(x==2 for x in edges.values()),error=abs(vol-m['volumeMm3']))

