from travel import *
from math import hypot
from heapq import heappop,heappush
d=json.loads((HERE/'park-inputs.json').read_bytes())
frames=d['frames']; first=frames[0]['actors']
start=np.array(first[0]['p']);goal=np.array(next(t['at'] for t in d['targets'] if t['id']=='blue-park'))
# Restrict the empty robot to its low, horizontal pose. All placed stock is included.
mx=mat(0,pivot_x(0),0,501)@inverse;shift=mat(0,pivot_x(0)-185,0,0)
ab=np.array([bounds(posegeom(sh,mx if n in moving else shift if n in slide else identity)) for n,sh in G.items() if n!=held])
mins=ab[:,:3].min(axis=0);maxs=ab[:,3:].max(axis=0)
lo=mins[:2]/25.4;hi=maxs[:2]/25.4
# Conservative swept peer boxes include every sample across the 24-second interval.
obs=[];names=[]
for j,r in enumerate(first[1:],1):
    pp=np.array([f['actors'][j]['p'] for f in frames]);half=np.array([r['length'],r['width']])/2+r['bumper']
    assert all(abs(f['actors'][j]['heading'])%180==0 for f in frames)
    obs.append([*(pp.min(axis=0)-half),*(pp.max(axis=0)+half)]);names.append(r['id'])
for obj in d['obstacles']:
    p=np.array(obj['poly']);obs.append([*p.min(axis=0),*p.max(axis=0)]);names.append(obj['name'])
obs=np.array(obs);low=obs[:,:2]-hi;high=obs[:,2:]-lo
def hits(a,b,padding=.15):
    a=np.array(a);b=np.array(b)
    if np.any(np.minimum(a,b)+lo<0) or np.any(np.maximum(a,b)+hi>[651.25,323.25]):return ['wall']
    # Slab intersection over continuous motion of fixed-heading rectangular outer bound.
    v=b-a;l=low-padding;h=high+padding;mn=np.zeros(len(obs));mx=np.ones(len(obs))
    for i in (0,1):
        if abs(v[i])<1e-9:
            mn=np.where((a[i]<l[:,i])|(a[i]>h[:,i]),2,mn)
        else:
            f=(l[:,i]-a[i])/v[i];g=(h[:,i]-a[i])/v[i]
            mn=np.maximum(mn,np.minimum(f,g));mx=np.minimum(mx,np.maximum(f,g))
    return [names[j] for j in np.flatnonzero(mn<=mx)]
delta=8; queue=[(0,0,(0,0))];prev={};cost={(0,0):0};end=None
while queue:
    _,g,q=heappop(queue)
    if g>cost[q]+1e-6:continue
    a=start+delta*np.array(q)
    if np.linalg.norm(a-goal)<18 and not hits(a,goal):end=q;break
    for dx,dy in itertools.product((-1,0,1),repeat=2):
        if dx==dy==0:continue
        nxt=(q[0]+dx,q[1]+dy);b=start+delta*np.array(nxt);c=g+delta*hypot(dx,dy)
        if c<cost.get(nxt,1e9) and not hits(a,b):
            prev[nxt]=q;cost[nxt]=c;heappush(queue,(c+np.linalg.norm(goal-b),c,nxt))
assert end is not None
raw=[goal];q=end
while True:
    raw.append(start+delta*np.array(q))
    if q==(0,0):break
    q=prev[q]
raw.reverse();pts=[raw[0]];i=0
while i<len(raw)-1:
    i=max(j for j in range(i+1,len(raw)) if not hits(pts[-1],raw[j]));pts.append(raw[i])
lengths=[np.linalg.norm(b-a) for a,b in zip(pts[:-1],pts[1:])]
# Stop at every direction change, 1.2 m/s cap, 1.4 m/s² symmetric accel.
acc=1.4;cap=1.2
def duration(length):
    length*=.0254
    return 2*(length/acc)**.5 if length<=cap*cap/acc else length/cap+cap/acc
times=[duration(l) for l in lengths];total=sum(times)
def point(t):
    for a,b,l,T in zip(pts[:-1],pts[1:],lengths,times):
        if t<=T:
            v=min(cap,acc*T/2);rise=v/acc
            x=.5*acc*t*t if t<rise else .0254*l-.5*acc*(T-t)**2 if t>T-rise else v*(t-rise/2)
            return (a+(b-a)*x/(l*.0254)).tolist()
        t-=T
    return goal.tolist()
# A point on the actual left bumper cover at the target falls inside the conservative triangle of upright centers.
tri=np.array([[220.97,110.845],[220.97,212.405],[133.01,161.63]])
test=goal+np.array([0, (330+40)/25.4]);signs=[np.linalg.det(np.array([b-a,test-a])) for a,b in zip(tri,np.roll(tri,-1,axis=0))]
assert all(x>0 for x in signs) or all(x<0 for x in signs)
out=dict(stage='TELEOP 111..135 s',projectedStartIn=start.tolist(),goalIn=goal.tolist(),waypointsIn=np.array(pts).tolist(),timesS=times,routeS=total,remainingS=24-total,fieldClear=all(not hits(a,b) for a,b in zip(pts[:-1],pts[1:])),straightControl=hits(start,goal),maximumRobotHeightMm=maxs[2],lowestUnloadedChainEdgeMm=717.55-13.335,lowerBoundMm=mins.tolist(),upperBoundMm=maxs.tolist(),bumperInsideUprightTriangleIn=test.tolist(),peerFrames=481,otherActors=names[:5],speedCapMps=cap,accelMps2=acc,viewPositionsMm=(np.array([point(i) for i in range(25)])*25.4).tolist(),scope='Fixed heading, empty horizontal cassette at branch entry, all peers and post envelopes included. Loaded chains, truss underside and actual match state remain field assumptions.')
dump(HERE/'park-rehearsal.json',out);print(total,pts,out['straightControl'],maxs,flush=True)

