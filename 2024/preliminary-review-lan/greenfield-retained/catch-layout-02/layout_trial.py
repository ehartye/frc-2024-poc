"""Editable left-cheek sector, sliding catch dog and translating bracket in the registered parked root."""
from pathlib import Path
import json,math,hashlib,sys,numpy as np
p=Path(__file__).resolve().parent;root=p.parent;sys.path.insert(0,str(root))
from layout import *
from showcase_contacts import load_solids
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.STEPControl import STEPControl_Writer
from math import cos,sin,radians,asin,degrees
d=json.loads((p/'parent-review.json').read_bytes())
path=root/'swap-blade/shared-cassette-swappable.step';parent,_=load_solids(path)
assert hashlib.sha256(path.read_bytes()).hexdigest()==d['step']
parent={n.replace(chr(194)+chr(183),chr(183)):v for n,v in parent.items()};assert set(parent)==set(d['bodies'])
G.clear();G.update(parent);attr={};park=mat(125,185,0,501);Y=(0,1,0)
def addpart(n,sh,role,material,job):
    assert BRepCheck_Analyzer(sh).IsValid(),n
    G[n]=place(sh,park) if role=='arm' else sh
    attr[n]=dict(role=role,material=material,purpose=job)
def rz(r,a):
    t=radians(a);return (r*cos(t),r*sin(t))
edge=-10.;segpoly=[rz(r,a) for r,angles in [(70,range(-10,21,2)),(38,range(20,-11,-2))] for a in angles]
def stock(poly,y,t):return prism([(x,y,z) for x,z in poly],(0,t,0))
sector=stock(segpoly,226,10)
def cross(x,y,z,r,l):return cyl((x,y,z),Y,r,l)
armbolts=[(44,1),(64,4),(54,13)]
cheek='Cassette cheek · left';block='Translating bearing block · left'
for i,(x,z) in enumerate(armbolts,1):
    sector=cut(sector,cross(x,225,z,3,12))
    G[cheek]=cut(G[cheek],place(cross(x,217,z,3,10),park))
    # Counterbored screw from outboard, threaded receiving envelope in cheek.
    sector=cut(sector,cross(x,232.9,z,5,5))
    screw=fuse(cross(x,219,z,3,13.9),cross(x,232.9,z,4.9,3))
    addpart('Catch sector screw '+str(i),screw,'arm','Steel','Clamp and locate the angular stop on the cassette cheek.')
addpart('Catch sector',sector,'arm','Steel','Transfer lowering torque from cassette cheek into the seated dog.')
# Radial stop face tangent to round dog at exactly the configured seat angle.
R=55.;theta=158+edge-degrees(asin(6/R));dx,dz=rz(R,theta);px=185+dx;pz=501+dz
lug=stock([(px-11,pz-9),(px-2,pz+12),(px+12,pz+11),(205,489),(205,457),(165,457),(161,487)],237,10)
lug=cut(lug,cross(185,236,501,19,12));lug=cut(lug,cross(185,236,478.6,6,12))
lug=fuse(lug,cross(px,247,pz,10,15))
lug=cut(lug,cross(px,236,pz,6.2,28))
brktbolts=[(172,463),(198,463),(172,479),(198,479)]
for i,(x,z) in enumerate(brktbolts,1):
    lug=cut(lug,cross(x,236,z,3,12));lug=cut(lug,cross(x,236,z,5,4.1))
    G[block]=cut(G[block],cross(x,246,z,3,13))
    addpart('Catch receiver screw '+str(i),fuse(cross(x,237.1,z,4.9,3),cross(x,240.1,z,3,15)),'slide','Steel','Hold the dog receiver to the translating bearing block.')
addpart('Catch receiver',lug,'slide','Aluminum','Guide the axial dog and react its side load into the translating truck.')
addpart('Catch dog',cross(px,237.2,pz,6,38),'dog','Steel','Advance 10.8 mm behind the sector before allowing load transfer.')
# Coaxial guide/actuator envelope with mounting flange, hollow for the dog stroke.
case=ann((px,262,pz),Y,11,6.35,29)
addpart('Catch actuator and spring',case,'slide','Actuator and spring envelope','Retract the dog during scoring and deploy it for the climb.')
new=list(attr);ch=[cheek,block];D=mat(0,0,-10.8,0)
def near(a,b):
    a,b=bounds(a),bounds(b);return all(min(a[j+3],b[j+3])-max(a[j],b[j])>1e-6 for j in range(3))
def hits(parts):
    out=[]
    for i,n in enumerate(new):
        for b,sh in parts.items():
            if b==n or (b in new and new.index(b)>i):continue
            if near(parts[n],sh):
                v=common(parts[n],sh)
                if v>.001:out.append([n,b,round(v,5)])
    return out
# Installed root and distinct mechanism poses; use real case groups for all old members.
src=d['cases'][0]['rig'];sl=set(n for n,r in next(c for c in d['cases'] if c['key']=='intake')['rig'].items() if r=='slide')
def posed(angle,depth):
    x=pivot_x(angle);slidePose=mat(0,x-185,0,0)
    armPose=mat(angle,x,0,501)@np.linalg.inv(park)
    parts={}
    for n,sh in G.items():
        role=attr[n]['role'] if n in attr else 'slide' if n in sl else src.get(n,'frame')
        trans=armPose if role in ('arm','flap') else slidePose if role in ('slide','dog') else np.eye(4)
        if role=='dog':trans=trans@mat(0,0,-depth,0)
        parts[n]=place(sh,trans) if not np.allclose(trans,np.eye(4)) else sh
    return parts
checks={}
for angle,depth in [(125,0),(-50,0),(90,0),(150,0),(160,10.8),(158,10.8),(157,10.8),(165,10.8)]:
    arr=posed(angle,depth);checks[str((angle,depth))]=dict(collision=hits(arr),dogSectorMm=distance(arr['Catch dog'],arr['Catch sector']),dogSectorCommon=common(arr['Catch dog'],arr['Catch sector']))
file=p/'catch-install-trial.step';write_step(file)
summary=dict(parent=d['step'],step=hashlib.sha256(file.read_bytes()).hexdigest(),n=len(G),axis=[px,pz],r=R,checks=checks,attr=attr)
(p/'layout-results.json').write_text(json.dumps(summary,indent=2))
print(json.dumps(checks))

