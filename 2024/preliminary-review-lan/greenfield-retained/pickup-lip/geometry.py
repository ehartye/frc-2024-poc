from pathlib import Path
import sys,json,math,numpy as np,hashlib
p=Path(__file__).resolve().parent;root=p.parent;sys.path.insert(0,str(root))
from layout import *
from mechanical_motion import common,distance,bounds
# Rebuild the preceding parked issue from its private authorship script, excluding its exporter.
base=(root/'mission-physics/stock.py').read_text(encoding='utf8')
exec(base[base.index("src=(root/"):base.index('out={n:')])
# Shift the full discharge/retention pivot so the entire folding leaf clears the seated front rim.
oldAxis=399.5;axis=457.5;flapName='Mouth retention flap';delta=mat(0,axis-oldAxis,0,12)
oldMove=mat(park,*pivot);inverseBase=np.linalg.inv(oldMove)
for name in (flapName,'Flap axle'):G[name]=place(G[name],oldMove@delta@inverseBase)
for side,y in [('left',218),('right',-226)]:
    name='Lid edge rail · '+side
    ledge=box(36,208 if side=='left' else -218,76,372,10,4)
    sidewall=cut(box(36,y,76,430,8,12),box(172,y,75,16,8,15))
    rise=prism([(405,y,88),(466,y,88),(466,y,102),(446,y,102)],(0,8,0))
    G[name]=place(cut(fuse(fuse(sidewall,ledge),rise),cyl((axis,y,96),Y,2,8)),oldMove)
hingeChanged=[flapName,'Flap axle','Lid edge rail · left','Lid edge rail · right']
prev=set(G);new=[];desc={}
pa=-50;pickpose=mat(pa,pivot_x(pa),0,501);ip=np.linalg.inv(pickpose);rest=mat(park,*pivot)
def localxz(x,z):
    v=ip@np.array([x,0,z,1]);return [v[0],v[2]]
# The four polygons share the same pickup datum for geometry and physics.
# 1.2-mm vertical tip capped with a blunt radius by the machined cross section.
top=[[593.8,24],[600,17],[607.5,8.9],[613,5],[618.2,4.1],[618.8,3.95],[619.05,3.6],[618.95,3.26],[618.55,3],[612.7,3.2],[606.5,6.3],[600,13.5],[593.8,20.5]]
back=[[593.8,20.5],[600,13.5],[600,10],[593.8,15.5]]
profiles={'Replaceable pickup lip':[localxz(*pt) for pt in top],'Pickup lip crossbar':[localxz(*pt) for pt in back]}
def make(n,prof,y,w,material,purpose,why,drill=False):
    sh=prism([(x,y,z) for x,z in prof],(0,w,0))
    if drill:sh=cut(sh,cyl((625,y-1,13),Y,7.5,w+2))
    local(n,sh,'Pickup lip','mechanism' if 'Replaceable' in n else 'structure',material)
    new.append(n);desc[n]=dict(**meta[n],purpose=purpose,rationale=why)
make('Replaceable pickup lip',profiles['Replaceable pickup lip'],-217,434,'UHMW polyethylene','Get under the leading rim of a floor NOTE and feed it up to the nose roller.','A blunt polymer edge offers a replaceable wear surface. The roller remains exposed across its working width.')
make('Pickup lip crossbar',profiles['Pickup lip crossbar'],-226,452,'6061 aluminum','Back the full blade width and transfer its load to the side extensions.','The long span needs support behind the thin leading edge.')
prof=[[568,-4],[598,-4],localxz(593.8,15.5),localxz(600,10),localxz(600,13.5),localxz(593.8,20.5),localxz(593.8,24),[598,11],[568,11]]
from OCP.BRepGProp import BRepGProp
# Monotone convex hull of the side-plate blank; trim the axle below.
points=sorted(set(tuple(pt) for pt in prof+[[620,26],[635,28]]))
def turn(a,b,c):return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
def edge(pts):
    result=[]
    for v in pts:
        while len(result)>=2 and turn(result[-2],result[-1],v)<=0:result.pop()
        result.append(v)
    return result[:-1]
prof=edge(points)+edge(points[::-1])
for side,y in [('left',226),('right',-230)]:
    name='Cassette side beam extension · '+side
    make(name,prof,y,4,'6061 aluminum','Carry the pickup crossbar from the existing cassette side beam.','A 30 mm side lap transfers load around the nose shaft. The end of the extension supports the crossbar.',True)
shapes={n:place(G[n],np.linalg.inv(rest)) for n in new}
# Conservative proposed slide correction. Move all slide-mounted stock together.
verts=np.vstack([np.array(viewmesh(sh)['v']) for sh in shapes.values()])
def correction(angle):
    c=cos(radians(angle));s=sin(radians(angle))
    return max(0.,pivot_x(angle)+max(verts[:,0]*c-verts[:,2]*s)-640)
def xyz(angle):
    return mat(angle,pivot_x(angle)-correction(angle),0,501)
checks=[];parkPairs=[];mins={};nearHits=[]
for i,n in enumerate(new+hingeChanged):
    for b in sorted((prev-set(hingeChanged))|set((new+hingeChanged)[:i])):
        a,c=bounds(G[n]),bounds(G[b])
        if all(min(a[j+3],c[j+3])-max(a[j],c[j])>-0.002 for j in range(3)):
            v=common(G[n],G[b])
            if v>.001:parkPairs.append([n,b,v])
for angle in np.arange(pa,165.01,.1):
    m=xyz(angle);vs=verts@m[:3,:3].T+m[:3,3];checks.append([angle,float(vs[:,0].max()),float(vs[:,2].min()),correction(angle),m[0,3]])
# Exact distances to the unchanged bumper. Bound each 1-degree interval by rigid-body travel.
angs=sorted(set([pa,-51.,*range(-50,166),125]))
radius=max(np.linalg.norm(verts[:,[0,2]],axis=1));vslide=2*radius+680
# Local closing-rate bound: choose minimum distance of endpoints minus maximum possible half-path travel.
bb=G['Blue bumper set']
for angle in angs:
    m=xyz(angle)
    mins[angle]=min(distance(place(sh,m),bb) for sh in shapes.values())
for a,b in zip(angs,angs[1:]):
    gap=min(mins[a],mins[b])-math.radians(b-a)*.5*(radius+vslide)
    nearHits.append(gap)
def profile2(n):return profiles[n]
out=dict(parentStep=hashlib.sha256((root/'mission-physics/selected-cassette.step').read_bytes()).hexdigest(),pickAngle=pa,pickPivot=xyz(pa)[:3,3].tolist(),profiles=profiles,parts={n:dict(**desc[n],**viewmesh(G[n])) for n in new},tipMm=[619.05,3],nativeParkedOverlap=parkPairs,bumperSweptLowerMm=min(nearHits),sampleFloorMm=min(i[2] for i in checks),maxFrontMm=max(i[1] for i in checks),maxAdditionalRetractionMm=max(i[3] for i in checks),minSlideX=min(i[4] for i in checks),noseGapMm=distance(G[new[0]],G['Nose pickup roller']),addedVolumeMm3={n:viewmesh(G[n])['volume'] for n in new},bladeSeatAreaMm2=434*math.hypot(6.2,7),projectFrontLimitMm=644.8)
out['bladeSeatAreaMm2']=434*math.hypot(6.2,7)
out['nominalBeamLapPerSideMm2']=450
out['hingeChanges']={n:viewmesh(G[n]) for n in hingeChanged}
out['flapAxleLocalMm']=[axis,0,96]
out['interSampleFrontUpperMm']=out['maxFrontMm']+math.radians(.05)*(radius+vslide)
out['pitchLowLimitControlMm']=float((verts@mat(pa-.5,*pivot)[:3,:3].T+[0,0,501])[:,2].min())
(p/'layout.json').write_text(json.dumps(out,separators=(',',':')),encoding='utf8')
write_step(p/'shared-cassette-lip.step')

