"""Twin captured tracks use the cassette pivot as a drum shaft. Input lengths are mm in the cassette frame."""
from pathlib import Path
import sys,json,hashlib,math
import numpy as np
p=Path(__file__).resolve().parent;root=p.parent;sys.path.insert(0,str(root/'assembly-recheck-01'))
from sweep import load_solids,name,park,ip
from layout import mat,box,cyl,ann,prism,cut,fuse,common,distance,bounds,place,viewmesh,G,write_step,X,Y,Z
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.BRepCheck import BRepCheck_Analyzer
def vol(s):
    props=GProp_GProps();BRepGProp.VolumeProperties_s(s,props);return props.Mass()
def read(f):return json.loads(f.read_bytes())
def digest(f):return hashlib.sha256(f.read_bytes()).hexdigest()
def save(f,o):f.write_text(json.dumps(o,indent=2,ensure_ascii=False),encoding='utf8')
source=root/'tread-underlay-01/field-review.step'
assert digest(source)==read(root/'endgame-route-01/guided-hooks.json')['source']
doc=read(root.parent/'www/data.json');layout=doc['concepts']['field'];active=next(c for c in layout['cases'] if c['key']=='field-r08')
original={name(k):s for k,s in load_solids(source)[0].items()}
members={k:place(s,ip) for k,s in original.items() if k in active['rig'] and k not in active['removed'] and active['rig'][k] in ('arm','flap','keeper')}
moving=set(read(root/'endgame-route-01/guided-hooks.json')['hookGroup'])
assert len(moving)==7
parts={};role={};rows={};bb={}
def add(k,s,group,rho):
    assert BRepCheck_Analyzer(s).IsValid() and vol(s)>0,k
    parts[k]=s;role[k]=group;rows[k]=dict(volumeMm3=vol(s),kg=vol(s)*rho*1e-9,role=group,densityKgM3=rho)
def offset(x=0,y=0,z=0):return mat(0,x,y,z)
# The stock damper/keeper mast and both purchased-profile hook models stay on the existing crosspiece.
cross='Shared lift crosspiece';add(cross,box(172,-212,180,16,424,12),'carriage',2700)
subtracted=vol(members[cross])*2700e-9
for side,hand in ((1,'left'),(-1,'right')):
    def block(x,y,z,dx,dy,dz):
        return box(x,y if side==1 else -y-dy,z,dx,dy,dz)
    def axle(x,y,z,rad,dy):return cyl((x,y if side==1 else -y-dy,z),Y,rad,dy)
    def ring(x,y,z,ro,ri,dy):return cut(axle(x,y,z,ro,dy),axle(x,y-.1,z,ri,dy+.2))
    def face(points,y,th):
        return prism([(x,y if side==1 else -y,z) for x,z in points],(0,side*th,0))
    def tagged(n,s,r,rho):add(n+' · '+hand,s,r,rho)
    # 2 mm C section open upward, with 4 mm lips; stationary outer side lands on the old forward post.
    track=cut(block(-111,194,151,323,24,26),block(-112,196,153,325,20,22))
    track=cut(track,block(-112,200,174,325,12,5))
    tagged('Hook guide channel',track,'cassette',2700)
    block1=block(153,196.2,153.2,54,19.6,21.6);neck=block(158,200.3,174.8,44,11.4,5.2)
    shoe=fuse(block1,neck)
    # Two mounting holes go through the beam and shoe. Steel fasteners are separate weight allowances.
    for x in [176,184]:
        shoe=cut(shoe,cyl((x,side*206,152),Z,1.65,30))
    tagged('Captive hook wear block',shoe,'carriage',1420)
    for x in [176,184]:
        parts[cross]=cut(parts[cross],cyl((x,side*206,179),Z,2.2,15))
    # Rear cantilever cleat ties a triangular web to the rail side and back to the pivot cheek.
    gusset=face([(-103,151),(-67,151),(69,26),(64,16),(30,16),(-94,126)],226,3)
    gusset=cut(gusset,axle(0,225,0,31,6))
    tagged('Guide rear anchor web',gusset,'cassette',2700)
    cleat=block(-103,218,151,36,8,26)
    # Full height outside lip joins the outer web which ends flush below the rail.
    cheek=fuse(cleat,block(-103,226,151,36,3,26))
    tagged('Guide rear anchor cleat',cheek,'cassette',2700)
    # Proposed driven cable drum. Bore and flanges are concentric with the retained pivot shaft.
    drum=ring(0,197,0,28,9.1,14)
    drum=cut(drum,axle(0,196.9,0,24.5,14.2))
    drum=fuse(drum,ring(0,197,0,32,9.1,2));drum=fuse(drum,ring(0,209,0,32,9.1,2))
    tagged('Pivot cable drum',drum,'drum',2700)
    # Rear redirect mounted between rail-side ears. The horizontal return leaves the slide closed at its rear load stop.
    wheel=ring(-97,197,164,13,3.1,10)
    tagged('Hoist rear redirect sheave',wheel,'cassette',2700)
    tagged('Hoist rear redirect axle',axle(-97,194,164,3,24),'cassette',7850)
# Account for final drilled beam.
rows[cross]['volumeMm3']=vol(parts[cross]);rows[cross]['kg']=rows[cross]['volumeMm3']*2700e-9
moving.add(cross)
originalArm={k:s for k,s in members.items() if k not in moving}
for k,s in parts.items():bb[k]=np.array(bounds(s))
for k,s in members.items():
    if k not in parts:bb[k]=np.array(bounds(s))
solids={**members,**parts};mobile={*moving,*(k for k,v in role.items() if v=='carriage')}
def candidate(a,b):return np.all(a[:3]<=b[3:]+.001) and np.all(b[:3]<=a[3:]+.001)
fixedHits=[];pairs=0;near=[]
for stroke in [0,55,110,165,220,230]:
    for i,(a,sa) in enumerate(solids.items()):
        for j,(b,sb) in enumerate(solids.items()):
            if j<=i or (a not in parts and b not in parts):continue
            changing=(a in mobile)!=(b in mobile)
            if not changing and stroke:continue
            dx=-stroke*(int(b in mobile)-int(a in mobile));off=np.array([dx,0,0,dx,0,0])
            if not candidate(bb[a],bb[b]+off):continue
            sb2=place(sb,offset(dx)) if dx else sb
            v=common(sa,sb2);pairs+=1
            if v>.01:fixedHits.append([stroke,a,b,v])
            elif not stroke:near.append([a,b,distance(sa,sb2)])
out=dict(parent=digest(source),materialParts=rows,removedBeamKg=subtracted,netStockKg=sum(v['kg'] for v in rows.values())-subtracted,targetStrokeMm=230,samplesMm=[0,55,110,165,220,230],narrowPairs=pairs,intersections=fixedHits,parkedNear=near)
G.clear();G.update({k:place(s,park) for k,s in parts.items()});write_step(p/'hoist-stock.step')
out['step']=digest(p/'hoist-stock.step');save(p/'stock.json',out)
print(out['netStockKg'],pairs,[(a,b,c,round(v,1)) for a,b,c,v in fixedHits[:35]])

