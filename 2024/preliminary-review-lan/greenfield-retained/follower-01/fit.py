"""Install two carpet followers on extended cassette cheeks. Input dimensions in mm."""
from probe import *
from math import pi
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
def volume(s):
    props=GProp_GProps();BRepGProp.VolumeProperties_s(s,props);return props.Mass()
def intersects(a,b):
    A,B=bounds(a),bounds(b)
    return all(min(A[j+3],B[j+3])>max(A[j],B[j])+1e-6 for j in range(3))
x,z,r=604.,10.7,13.;toPark=np.linalg.inv(toPick)
G.clear();G.update(names);mods=[];adds=[];kg=0;checks={};mats={};items={}
for sign,side in [(1,'left'),(-1,'right')]:
    n='Cassette side beam extension · '+side;old=place(names[n],toPick)
    yy=226 if sign==1 else -230
    ear=prism([(a,yy,b) for a,b in [(590,19),(600,5.4),(606.5,5.4),(609.5,8.4),(609.5,12),(606,15.8),(591,25)]],(0,4,0))
    ax=(0,sign,0)
    def cy(st,rad,w):return cyl((x,sign*st,z),ax,rad,w)
    modified=cut(fuse(old,ear),cy(224,2.5,8));kg+=(volume(modified)-volume(old))*2.7e-6
    join=common(ear,old);assert join>20 and BRepCheck_Analyzer(modified).IsValid()
    G[n]=place(modified,toPark);mods.append(n)
    # Flanged axle shoulder ends 0.15 mm proud of tread, tightened to locknut.
    axle=fuse(fuse(cy(223.6,4,2.4),cy(226,2.5,12.45)),cy(238.45,2,4))
    wheel=cut(cy(230.3,r,8),cy(230,2.60,9))
    washer=ann((x,sign*230,z),ax,4.5,2.51,.25)
    nut=ann((x,sign*238.45,z),ax,4,2,3.7)
    for part,shape,density,matl in [('Carpet follower',wheel,1.4e-6,'Acetal'),('Follower shoulder axle',axle,7.85e-6,'Steel'),('Follower thrust washer',washer,7.85e-6,'Steel'),('Follower locknut',nut,7.85e-6,'Steel')]:
        label=part+' · '+side;G[label]=place(shape,toPark);adds.append(label);mats[label]=matl;kg+=density*volume(shape)
    checks[side]=dict(joinOverlapMm3=join,loadSideMm=230.30,outerHeadClearanceMm=distance(place(G[adds[-4]],toPick),place(names['Nose motor'],toPick)))
hits=[]
for n in adds+mods:
    for m,v in G.items():
        if n==m or ((m in mods+adds) and m>n):continue
        if intersects(G[n],v):
            overlap=common(G[n],v)
            if overlap>.001:hits.append([n,m,overlap])
print('hits',hits)
result=dict(source=hashlib.sha256(src.read_bytes()).hexdigest(),wheelCenterPickup=[x,234.3,z],radius=r,installedAddedKg=kg,checkedParked=checks,overlaps=hits,modified=mods,added=mats)
if not hits:
    write_step(p/'paired-follower.step');result['step']=hashlib.sha256((p/'paired-follower.step').read_bytes()).hexdigest()
    (p/'mesh.json').write_text(json.dumps({n:viewmesh(G[n]) for n in mods+adds},separators=(',',':')))
(p/'fit.json').write_text(json.dumps(result,indent=2))


