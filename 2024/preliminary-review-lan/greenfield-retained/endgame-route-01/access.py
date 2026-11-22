"""Fixed-heading, continuous slab checks for the updated preliminary robot and terminal field traffic."""
from pathlib import Path
from ast import parse,literal_eval,Assign
import json, math, sys, hashlib, heapq
import numpy as np
p=Path(__file__).resolve().parent;root=p.parent
sys.path.insert(0,str(root/'follower-01'))
from stroke import scheduled, mat
def read(f):return json.loads(f.read_bytes())
def sha(f):return hashlib.sha256(f.read_bytes()).hexdigest()
d=read(root.parent/'www/data.json');layout=d['concepts']['field']
bodies={**d['bodies'],**layout['add']};c=read(p/'original-park.json') if (p/'original-park.json').exists() else next(k for k in layout['cases'] if k['key']=='field-park')
roles={n:role for n,role in c['rig'].items() if n not in c['removed']}
assert len(roles)==len(bodies.keys()-(set(layout['remove'])-layout['add'].keys())-set(c['removed']))
park=mat(125,185,501);ip=np.linalg.inv(park);hinge=mat(0,457.5,96)
def pose(a,g=0):
    pivot=mat(a,scheduled(a),501)
    return dict(frame=np.eye(4),slide=mat(0,scheduled(a)-185),arm=pivot@ip,flap=pivot@hinge@mat(90*np.clip((a-102)/18,0,1))@np.linalg.inv(hinge)@ip,keeper=pivot@mat(0,0,g)@ip)
names=list(roles);roles['Common keeper yoke and sliding rack']='keeper'
local=[np.array(bodies[n]['v']) for n in names]
def boxes(angle,gate=0):
    tr=pose(angle,gate);out=[]
    for n,v in zip(names,local):
        t=tr[roles[n]];w=v@t[:3,:3].T+t[:3,3];out.append([w.min(axis=0),w.max(axis=0)])
    return np.array(out)
fieldpath=Path('C:/Users/ehart/Documents/FRC Native/2024/six-speaker-20261118/field-source/whole-field-sliced-bounds.json')
field=read(fieldpath);syntax=parse(Path('C:/Users/ehart/repos/frc-2024-poc/sim/pressure/field_route_clearance.py').read_text())
ignore=set.union(*(literal_eval(n.value) for n in syntax.body if isinstance(n,Assign) and n.targets[0].id in ('SURFACES','GAME_PIECES')))
items=[];ids=[];tridata=None;skipped=0
for part in field['parts']:
    if part['path'][1] in ignore:continue
    for a,b in part.get('tightBounds',[part['bounds']]):
        a=np.array(a)*1000;b=np.array(b)*1000
        if b[2]<.01 or a[2]>1450:continue
        if part['path'][1]=='GE-24200_Blue' and part['path'][-1]=='GE-24205_hanging' and b[0]-a[0]<100:
            skipped+=1;continue
        items.append([a,b]);ids.append('/'.join(part['path']))
assert skipped==1
sys.path.insert(0,str(root/'chain-entry-01'));from sections import tri
for iy in np.arange(tri[:,:,1].min(),tri[:,:,1].max(),12):
    chosen=tri[(tri[:,:,1].max(axis=1)>=iy)&(tri[:,:,1].min(axis=1)<=iy+12)]
    if len(chosen):
        v=chosen.reshape(-1,3);lo=v.min(axis=0);hi=v.max(axis=0);lo[1]=max(lo[1],iy);hi[1]=min(hi[1],iy+12)
        items.append([lo,hi]);ids.append('working chain')
peers=read(root/'park-inputs.json')['frames']
for j,r in enumerate(peers[0]['actors'][1:],1):
    xy=np.array([f['actors'][j]['p'] for f in peers])*25.4;half=25.4*(np.array([r['length'],r['width']])/2+r['bumper'])
    assert all(f['actors'][j]['heading']%180==0 for f in peers)
    items.append([np.r_[xy.min(axis=0)-half,-100],np.r_[xy.max(axis=0)+half,1800]]);ids.append(r['id'])
items=np.array(items)
class Field:
    def __init__(self,angle,gate=0):
        b=boxes(angle,gate);sz=b[:,:,2];self.b=b;ix,j=np.where((items[None,:,0,2]<=sz[:,1,None]+.3)&(items[None,:,1,2]>=sz[:,0,None]-.3))
        self.lo=items[j,0,:2]-b[ix,1,:2]-.3;self.hi=items[j,1,:2]-b[ix,0,:2]+.3;self.ix=ix;self.j=j
        self.wallLo=-b[:,0,:2].min(axis=0);self.wallHi=np.array([651.25,323.25])*25.4-b[:,1,:2].max(axis=0)
        rect,first=np.unique(np.round(np.c_[self.lo,self.hi],4),axis=0,return_index=True);self.unique=first
    def hits(self,a,b,detail=False):
        a=np.array(a);b=np.array(b);v=b-a
        if np.any(np.minimum(a,b)<self.wallLo) or np.any(np.maximum(a,b)>self.wallHi):return ['field boundary']
        ix=np.arange(len(self.lo)) if detail else self.unique
        ix=ix[np.all((self.lo[ix]<=np.maximum(a,b))&(self.hi[ix]>=np.minimum(a,b)),axis=1)]
        if not len(ix):return []
        lo=self.lo[ix];hi=self.hi[ix];entry=np.zeros(len(ix));leave=np.ones(len(ix))
        for k in (0,1):
            if abs(v[k])>1.e-9:
                t1=(lo[:,k]-a[k])/v[k];t2=(hi[:,k]-a[k])/v[k];entry=np.maximum(entry,np.minimum(t1,t2));leave=np.minimum(leave,np.maximum(t1,t2))
        return sorted(set((names[self.ix[i]],ids[self.j[i]]) for i in ix[entry<=leave]))
def times(pts):
    def T(mm):
        l=mm/1000;v=1.2;a=1.4
        return 2*math.sqrt(l/a) if l<=v*v/a else l/v+v/a
    return [T(np.linalg.norm(b-a)) for a,b in zip(pts[:-1],pts[1:])]
def search(f,start,goal):
    if f.hits(start,start) or f.hits(goal,goal):return []
    if not f.hits(start,goal):return [start,goal]
    step=225.;K=(0,0);dist={K:0};prev={};queue=[(0,0,K)]
    while queue:
        _,cost,k=heapq.heappop(queue);a=start+step*np.array(k)
        if cost>dist[k]:continue
        if np.linalg.norm(a-goal)<650 and not f.hits(a,goal):break
        for dx,dy in ((0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)):
            key=(k[0]+dx,k[1]+dy);b=start+step*np.array(key);c=cost+np.linalg.norm(b-a)
            if c<dist.get(key,1e8) and not f.hits(a,b):
                dist[key]=c;prev[key]=k;heapq.heappush(queue,(c+np.linalg.norm(b-goal),c,key))
    else:return []
    raw=[goal]
    while True:
        raw.append(start+step*np.array(k))
        if k==K:break
        k=prev[k]
    raw.reverse();out=[start];i=0
    while i<len(raw)-1:
        i=max(j for j in range(i+1,len(raw)) if not f.hits(out[-1],raw[j]));out.append(raw[i])
    return out
def transition(xy,a,b,gate=0):
    # Reject every component/field box separated by less than the maximum inter-sample motion.
    nearby=np.all((items[:,0,:2]<xy+1600)&(items[:,1,:2]>xy-1600),axis=1)
    obs=items[nearby];idx=np.flatnonzero(nearby);gap=1e6;at=None
    for q in np.linspace(a,b,1+math.ceil(abs(b-a)/.25)):
        shape=boxes(q,gate);shape[:,:,:2]+=xy
        if len(obs)==0:continue
        sep=np.maximum(obs[None,:,0,:]-shape[:,None,1,:],shape[:,None,0,:]-obs[None,:,1,:]).max(axis=2)
        i,j=np.unravel_index(sep.argmin(),sep.shape)
        if sep[i,j]<gap:gap=float(sep[i,j]);at=[float(q),names[i],ids[idx[j]]]
    return dict(minimumSlabMm=gap,at=at,pass5mm=gap>5.)
if __name__=='__main__':
    original=read(root/'six-piece-01/park-issued.json');parkpts=np.array(original['waypointsIn'])*25.4
    pr=Field(-18);pq=transition(parkpts[0],125,-18)
    parkcheck=dict(route=[pr.hits(a,b,True) for a,b in zip(parkpts[:-1],parkpts[1:])],transition=pq,elapsed=2.9+sum(times(parkpts)))
    ap=read(root/'keeper-drive-02/root-entry.json')['poses']['0'];angle=ap['th'];mouth=np.array([ap['x'],4104.32]);waiting=mouth+[800,0]
    high=Field(125);stage=Field(angle,44);result={}
    for k,start in [('terminal snapshot',parkpts[0]),('sixth shot',np.array([1540.,5550.]))]:
        path=search(high,start,waiting);elapsed=sum(times(path)) if path else None
        result[k]=dict(path=[v.tolist() for v in path],times=times(path),driveSeconds=elapsed,straightControl=high.hits(start,waiting,True)[:6],startState=high.hits(start,start,True)[:4],waitingState=high.hits(waiting,waiting,True)[:4])
    out=dict(layout=layout['step'],field=sha(fieldpath),peers=sha(root/'park-inputs.json'),placedBodies=len(names),park=parkcheck,climb=result,openGateTurn=transition(waiting,125,angle,44),finalApproachBlocked=stage.hits(waiting,mouth,True),waiting=waiting.tolist(),mouth=mouth.tolist(),angle=angle,finalDrive=times([waiting,mouth])[0],scope='Whole-robot mesh bounds, fixed heading, unfurled gates for the final approach, exact route-segment slab tests, 0.30 mm stock/field expansion. Peer envelopes bound the five supplied actors throughout the terminal 24 seconds.')
    (p/'access.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,ensure_ascii=True)[-3500:],flush=True)


