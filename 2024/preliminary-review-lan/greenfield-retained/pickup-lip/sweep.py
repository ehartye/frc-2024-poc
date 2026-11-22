from pathlib import Path
import json,numpy as np,math
p=Path(__file__).resolve().parent
src=(p/'geometry.py').read_text(encoding='utf8');exec(src[:src.index('checks=[]')]);mf=meta
def xbounds(a,m):
    c=(a[:3]+a[3:])/2;half=(a[3:]-a[:3])/2;center=m[:3,:3]@c+m[:3,3];span=np.abs(m[:3,:3])@half;return np.r_[center-span,center+span]
def near(a,b):return all(min(a[j+3],b[j+3])-max(a[j],b[j])>1e-4 for j in range(3))
# Only changed rigid relative placements are compared, including the altered slide command.
fixed={n:G[n] for n in prev if n not in moving+slide and n!=held}
fixedb={n:np.asarray(bounds(s)) for n,s in fixed.items()}
trucks={n:G[n] for n in slide};arm={n:place(G[n],np.linalg.inv(rest)) for n in G if n in moving and n!=held}
boxes={n:np.array(bounds(s)) for n,s in {**arm,**trucks}.items()};hits={};cnt=0;ctrl=0
for deg in np.arange(-51.5,165.01,1):
    m=xyz(deg);tx=mat(0,m[0,3]-185,0,0);full=correction(deg)>.0001 or deg< -50
    for n in (list(arm)+list(slide) if full else new):
        body=arm[n] if n in arm else trucks[n];mm=m if n in arm else tx;bound=xbounds(boxes[n],mm)
        for other,ob in fixedb.items():
            if not near(bound,ob):continue
            if n in slide and deg>=-50 and not full:continue
            value=common(place(body,mm),fixed[other]);cnt+=1
            if value>.01 and value>hits.get((n,other),[0])[0]:hits[n,other]=[value,deg]
# Exact intersections of each added part against the prior free-shot trajectory at every fine physics step in reach.
traj=json.loads((root/'mission-physics/production-fine.json').read_bytes());shotTests=0;shotHits=[];minDist=1e5
from OCP.gp import gp_Quaternion
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
for r in traj['trace']:
    q=r['q'];v=np.array(q[:3])*1000;v[:2]-=np.array(traj['pivot'][:2])*1000-[185,0]
    tr=gp_Trsf();tr.SetRotation(gp_Quaternion(*q[4:],q[3]));tr.SetTranslationPart(gp_Vec(*v))
    piece=BRepBuilderAPI_Transform(torus,tr,True).Shape();bb=np.array(bounds(piece))
    for n in new:
        xb=np.array(bounds(G[n]));lower=float(np.linalg.norm(np.maximum(np.maximum(bb[:3]-xb[3:],xb[:3]-bb[3:]),0)))
        if lower>minDist:continue
        gap=distance(piece,G[n]);shotTests+=1;minDist=min(minDist,gap)
        if gap<1e-5:shotHits.append([r['t'],n,common(piece,G[n])])
cl=json.loads((root/'mission-physics/hooked.json').read_bytes());low=9999
for r in cl['trace']:
    if r['t']<4.1:continue
    q=r['q'];tr=gp_Trsf();tr.SetRotation(gp_Quaternion(*q[1:],q[0]));rot=np.array([[tr.Value(i,j) for j in range(1,4)] for i in range(1,4)])
    m=xyz(r['angle']);pts=verts@m[:3,:3].T+m[:3,3];low=min(low,float((pts@rot.T)[:,2].min()+r['p'][2]*1000))
out=dict(scope='1 degree relative-placement samples on the new arms and altered slide range. Existing nominal guide and lead-screw interfaces listed separately. Shot checked at archived 12 ms display samples. Disabled climb sampled on inherited motion.',exactPairs=cnt,positivePairs=[dict(pair=list(k),volumeMm3=v[0],deg=v[1]) for k,v in hits.items()],minSampledShotGapMm=minDist,shotNarrowTests=shotTests,shotHits=shotHits,minDisabledLipHeightMm=low)
(p/'sweep.json').write_text(json.dumps(out,indent=2));print(out)

