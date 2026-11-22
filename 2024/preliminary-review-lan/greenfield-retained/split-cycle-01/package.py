"""Package a shared feed jackshaft and the two original reversible launcher motors in cassette coordinates (mm)."""
from pathlib import Path
import sys,json,hashlib,numpy as np
p=Path(__file__).resolve().parent;sys.path.insert(0,str(p.parent/'assembly-recheck-01'))
from sweep import shapes,roles,park,ip,place,src
from layout import box,cyl,fuse,cut,ann,common,bounds,distance,mat,write_step,G,X,Y,Z
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
def vol(s):
    g=GProp_GProps();BRepGProp.VolumeProperties_s(s,g);return g.Mass()
local={n:place(s,ip) for n,s in shapes.items()}
removed=[];added={};groups={};railParts=[]
def add(n,s,kg,category):
    assert BRepCheck_Analyzer(s).IsValid() and vol(s)>0,n
    local[n]=s;added[n]=kg;groups[n]=category
def remove(n):
    removed.append(n);del local[n]
def profile(x0,x1,y,r):
    return fuse(fuse(box(x0,y-r,32,x1-x0,2*r,32),cyl((x0,y,32),Z,r,32)),cyl((x1,y,32),Z,r,32))
def shift(x):return mat(0,x,0,0)
for sign,side in ((1,'left'),(-1,'right')):
    y=195.8*sign
    for cat in ('Belt motor','Belt motor shelf','Belt drive shaft','Nose pulley shelf','Idler axle','Belt pulley','Opposed traction belt','Compliant belt tread'):
        for n in tuple(local):
            if n.startswith(cat+' · '+side):remove(n)
    for name,x0,x1 in [('Feed',97.,352.),('Launch',407.,600.)]:
        for label,outer,inner in [('belt',18,16),('tread',24,18)]:
            n=f'{name} {label} · {side}'
            shape=cut(profile(x0,x1,y,outer),profile(x0,x1,y,inner));add(n,shape,vol(shape)*(1.2e-6 if label=='belt' else .25e-6),'traction');railParts.append(n)
        for suffix,x in [('drive',x0),('idler',x1)]:
            add(f'{name} {suffix} pulley · {side}',ann((x,y,32),Z,16,5,32),.022,'traction')
            if name=='Feed' and suffix=='drive':continue
            if suffix=='drive':
                for item in ('Belt motor','Belt motor shelf','Belt drive shaft'):
                    label=item+' · '+side
                    add('Launch '+label,place(shapes[label],shift(x-97)@ip),0.,'retained launch drive')
            elif name=='Launch':
                seat=cut(box(589,y-17,19,22,34,6),cyl((600,y,18),Z,5.4,8))
                add('Launch idler bridge seat · '+side,seat,.012,'pulley support')
                add('Launch idler axle · '+side,cyl((600,y,19),Z,5,45),.027,'pulley support')
            else:
                for item in ('Nose pulley shelf','Idler axle'):
                    label=item+' · '+side
                    add(name+' '+label,place(shapes[label],shift(x-558)@ip),0. if name=='Launch' else (.018 if item=='Idler axle' else .023),'pulley support')
    # Two opposed right-angle gearheads drive the inboard feed pulleys with one transverse input.
    # Cabinet envelopes include gears/bearings; horizontal and vertical shaft bores register the interfaces.
    face=y-10 if sign>0 else y-10
    shell=box(85,face,-28,24,20,27)
    shell=cut(shell,cyl((97,-230,-16),Y,4,460));shell=cut(shell,cyl((97,y,-16),Z,5,17))
    add('Feed right-angle gearbox · '+side,shell,.055,'feed drive')
    bracket=box(83, min(y-10,218) if sign>0 else -218,-1,28,218-185.8,5)
    bracket=cut(bracket,cyl((97,y,-2),Z,5.4,8));add('Feed gearhead flange · '+side,bracket,.014,'feed drive')
    add('Feed output shaft · '+side,cyl((97,y,-12),Z,5,76),.040,'feed drive')
# The through shaft enters both gearheads to their intersecting pitch datums. The final 4 mm at each corner belongs to the gears.
shaft=cyl((97,-191.8,-16),Y,4,383.6);add('Feed transverse jackshaft',shaft,.055,'feed drive')
# Central housing at the right side. Parallel motor axis is 29 mm from the through shaft, leaving room for a two-gear reduction.
housing=box(86,-124,-27,53,18,22)
housing=cut(housing,cyl((97,-125,-16),Y,4,20));housing=cut(housing,cyl((126,-125,-16),Y,3,20))
add('Feed input reduction housing',housing,.065,'feed drive')
add('Feed motor output',cyl((126,-129,-16),Y,3,17),.005,'feed drive')
add('Feed motor',cyl((126,-173,-16),Y,13,44),.105,'feed drive')
add('Feed input pilot flange',cut(box(112,-129,-29,28,5,26),cyl((126,-130,-16),Y,3.2,8)),.006,'feed drive')
# Tie the reducer case into the existing transverse frame of the cassette.
support=box(125,-123,-5,10,16,19)
add('Feed reducer mount',support,.012,'feed drive')
bbox={n:np.array(bounds(s)) for n,s in local.items()}
hits=[];count=0
for i,n in enumerate(added):
    for m in local:
        if m==n or (m in added and m>n) or m=='Chambered NOTE':continue
        a,b=bbox[n],bbox[m]
        if (np.minimum(a[3:],b[3:])-np.maximum(a[:3],b[:3])>1e-5).all():
            v=common(local[n],local[m]);count+=1
            if v>.001:hits.append([n,m,v])
from OCP.TopLoc import TopLoc_Location
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
def root_geometry(shape):
    assembled=place(shape,park)
    return BRepBuilderAPI_Transform(assembled.Located(TopLoc_Location()),assembled.Location().Transformation(),True).Shape()
G.clear();G.update({n:root_geometry(s) for n,s in local.items()});write_step(p/'split-drive-layout.step')
r=dict(parent=hashlib.sha256(src.read_bytes()).hexdigest(),layout=hashlib.sha256((p/'split-drive-layout.step').read_bytes()).hexdigest(),solids=len(local),added=added,categories=groups,removed=removed,parkedCandidates=count,overlaps=hits,
scope='Direct solid preliminary envelope for one shared transverse feed drive and two existing relocated independent launch drives. Gearhead internal tooth flanks, purchase selection, phase drag, shop fits and fastener layout remain open.')
(p/'package.json').write_text(json.dumps(r,indent=2));print(count,hits,flush=True)

