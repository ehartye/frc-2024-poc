"""Move the slide drive to the forward support and add a coupling and supported motor face."""
from pathlib import Path
import numpy as np, json, sys, hashlib
p=Path(__file__).resolve().parent;root=p.parent
sys.path.insert(0,str(root/'assembly-recheck-01'))
from sweep import load_solids,name
from layout import box,cyl,ann,place,common,cut,fuse,bounds,X,G,write_step,viewmesh
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from OCP.BRepCheck import BRepCheck_Analyzer
def V(s):q=GProp_GProps();BRepGProp.VolumeProperties_s(s,q);return q.Mass()
def sha(f):return hashlib.sha256(f.read_bytes()).hexdigest()
parent=root/'tread-underlay-01/field-review.step';raw={name(n):s for n,s in load_solids(parent)[0].items()};G.clear();G.update(raw)
center=(275,428)
def shaft(x,length,r):return cyl((x,*center),X,r,length)
# Four-sided yoke formed by opposite ribs, a window for the coupling, and two face plates.
rear=cut(box(230,255,411,3,40,34),shaft(229,5,6));front=cut(box(253,255,411,3,40,34),shaft(252,5,5))
bridge=fuse(fuse(rear,box(232.9,255,411,20.2,3,34)),box(232.9,292,411,20.2,3,34));bridge=fuse(bridge,front)
a='Slide drive motor';mount='Forward slide motor yoke';coupler='Slide screw coupling';drive='Slide motor output shaft';tail='Slide screw extension'
G[a]=shaft(256,53,14);G[mount]=bridge;G[coupler]=ann((233.5,*center),X,9,4,18);G[drive]=shaft(243,13,4);G[tail]=shaft(230,12.8,4)
new=[a,mount,coupler,drive,tail]
dx={n:V(G[n]) for n in new};failed=[];pairs=0
for n in new:
    assert BRepCheck_Analyzer(G[n]).IsValid()
    for other in G:
        if other==n or other in new and new.index(other)<new.index(n):continue
        A=bounds(G[n]);B=bounds(G[other])
        if np.any(np.array(A[:3])>np.array(B[3:])+.001) or np.any(np.array(B[:3])>np.array(A[3:])+.001):continue
        v=common(G[n],G[other]);pairs+=1
        if v>.01:failed.append([n,other,v])
meas=dict(sourceMotorVol=V(raw[a]),motorVol=dx[a],radialClearMm=9,shaftGapMm=.2,overlapScrewMm=9.3,overlapMotorMm=8.5,bracketToHostMm2=common(place(bridge,np.array([[1,0,0,-.1],[0,1,0,0],[0,0,1,0],[0,0,0,1.]])),G['Slide bearing mount 220'])/.1)
mass=(dx[mount]+dx[coupler])*2.7e-6+(dx[drive]+dx[tail])*7.85e-6
write_step(p/'forward-slide.step');f=p/'forward-slide.step';assert abs(dx[a]-V(raw[a]))<.01
(p/'motor-fit.json').write_text(json.dumps(dict(parent=sha(parent),source=sha(f),new=new,checks=pairs,positive=failed,nominalAddedKg=mass,allowanceKg=[mass+.018,mass+.042],seats=meas,scope='Package-level motor case, shaft engagement, bored coupling and supported face, nominal envelope. Supplier fit and torque, positive retention, fasteners and service access remain to be checked.'),indent=2))
(p/'motor-mesh.json').write_text(json.dumps({n:viewmesh(G[n]) for n in new},separators=(',',':')));print(pairs,failed,meas,mass)

