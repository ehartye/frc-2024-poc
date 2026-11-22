"""Replace 0.75 mm of the outside backing thickness with compliant tread. Pulley seats and contact envelope stay fixed."""
from pathlib import Path
import sys,json,numpy as np,math,hashlib
p=Path(__file__).resolve().parent;root=p.parent;sys.path.insert(0,str(root/'assembly-recheck-01'));sys.path.insert(1,str(root/'follower-01'))
from sweep import load_solids,name,park,ip
from layout import box,cyl,Y,Z,mat,cut,fuse,common,place,write_step,G,viewmesh,BRepPrimAPI_MakeTorus
from stroke import scheduled
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
def V(s):g=GProp_GProps();BRepGProp.VolumeProperties_s(s,g);return g.Mass()
def profile(a,b,y,r):return fuse(fuse(box(a,y-r,32,b-a,2*r,32),cyl((a,y,32),Z,r,32)),cyl((b,y,32),Z,r,32))
base=root/'keeper-drive-02/keeper-drive.step';G.clear();G.update({name(n):s for n,s in load_solids(base)[0].items()})
retained={};changed=[];dm=0
for side,sign in [('left',1),('right',-1)]:
    for nm,a,b in [('Feed',97,352),('Launch',407,600)]:
        walls=[profile(a,b,sign*195.8,r) for r in (16,17.25,24)]
        for j,(part,rho) in enumerate([('belt',1.2e-6),('tread',.25e-6)]):
            n=nm+' '+part+' · '+side;retained[n]=G[n]
            G[n]=place(cut(walls[j+1],walls[j]),park);changed.append(n);dm+=(V(G[n])-V(retained[n]))*rho
src=root/'split-field-02/R02-center-high-96.json';rec=json.loads(src.read_bytes());row=min(rec['trajectory'],key=lambda v:abs(v[0]-1.87));assert row[0]==1.87
q=rec['pitchDegrees'];W,X,Yc,Zc=row[4:8]
rr=np.array([[1-2*(Yc**2+Zc**2),2*(X*Yc-Zc*W),2*(X*Zc+Yc*W)],[2*(X*Yc+Zc*W),1-2*(X**2+Zc**2),2*(Yc*Zc-X*W)],[2*(X*Zc-Yc*W),2*(Yc*Zc+X*W),1-2*(X**2+Yc**2)]])
matrix=np.eye(4);matrix[:3,:3]=rr;matrix[:3,3]=np.array(row[1:4])*1000;foot=mat(q,169.3625892825837,0,501);M=park@np.linalg.inv(foot)@matrix;torus=BRepPrimAPI_MakeTorus(152.4,25.4).Shape()
occ=[]
for n in changed:
    if ' belt ' not in n:continue
    diff=place(torus,M)
    occ.append([n,common(diff,retained[n]),common(diff,G[n]),common(place(diff,mat(0,0,2 if 'left' in n else -2,0)),G[n])])
write_step(p/'field-review.step');sha=lambda f:hashlib.sha256(f.read_bytes()).hexdigest()
r=dict(parent=sha(base),step=sha(p/'field-review.step'),changed=changed,netNominalKg=dm,controlTrace=sha(src),atS=row[0],backingComparisons=occ,hardBackingOuterMm=17.25,pulleyRadiusMm=16,compliantOuterMm=24,
scope='Same tread contact envelope, 1.25 mm backing and 6.75 mm compliant tread. Neutral-axis drive ratio, compound stiffness, timing-belt pitch, flex life and actual Article force-travel remain assumptions.')
(p/'result.json').write_text(json.dumps(r,indent=2));(p/'mesh.json').write_text(json.dumps({n:viewmesh(G[n]) for n in changed},separators=(',',':')))
print(dm,occ)

