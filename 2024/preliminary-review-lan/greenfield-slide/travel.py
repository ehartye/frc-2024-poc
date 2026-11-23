from check import *
from OCP.BRep import BRep_Builder
from OCP.TopoDS import TopoDS_Compound
from functools import lru_cache
identity=np.eye(4); mbase=mat(park,*pivot); inverse=np.linalg.inv(mbase); invseat=np.linalg.inv(seat)
def floorpose(f):
    # An independently tilted torus tangent to the carpet and the inclined runner plane.
    angle=radians(pick); phi=pick*f; n=50.4+152.4*abs(sin(radians(phi-pick)))
    z=25.4+152.4*abs(sin(radians(phi)))
    s=(z-pivot[2]-n*cos(angle))/sin(angle)
    p=centerpose(pick,s,n);return mat(phi,*p[:3,3]),s
m0,s0=floorpose(0); mend,send=floorpose(1)
cases=['Preload at launch angle','Deploy to floor','Tip and draw onto runners','Raise loaded cassette','Battery withdrawal','Battery at supported service angle']
def poses(case,t):
    theta=park;piece=seat;batt=identity
    if case==cases[1]:theta=park+(pick-park)*t;piece=m0
    if case==cases[2]:
        theta=pick
        piece=floorpose(t*2)[0] if t<=.5 else centerpose(pick,send+(215-send)*(2*t-1),50.4)
    if case==cases[3]:theta=pick+(park-pick)*t;piece=centerpose(theta,215,50.4)
    if case==cases[4]:batt=mat(0,0,0,370*t)
    if case==cases[5]:theta=80;piece=centerpose(theta,215,50.4);batt=mat(0,0,0,370*t)
    return {'slide':mat(0,pivot_x(theta)-185,0,0),'arm':mat(theta,pivot_x(theta),0,pivot[2])@inverse,'piece':piece@invseat,'battery':batt,'fixed':identity}
def group(n):return 'arm' if n in moving else 'slide' if n in slide else 'piece' if n==held else 'battery' if n=='Battery and terminal case' else 'fixed'
def posegeom(s,m):return s if np.allclose(m,identity,atol=1e-12) else place(s,m)
def transformed_bounds(box,m):
    pts=np.array(list(itertools.product(*zip(box[:3],box[3:]))));p=pts@m[:3,:3].T+m[:3,3]
    return [*p.min(axis=0),*p.max(axis=0)]
def sample(case):
    issues={};no_measure=[];narrow=0;pair_pose=0;results=[]; min_floor=float('inf')
    ids=[n for n in G if not(case in cases[4:] and n in ('Pack lead','Distribution lead',held))]
    pairs=[(a,b) for a,b in itertools.combinations(ids,2) if group(a)!=group(b)]
    # Identical-relative-pose pairs need one exact check. Reuse solids with fixed transforms.
    seen=set()
    for i in range(25):
        t=i/24;ms=poses(case,t);boxes={n:transformed_bounds(AAB[n],ms[group(n)]) for n in ids}
        cache={}
        def sh(n):
            if n not in cache:cache[n]=posegeom(G[n],ms[group(n)])
            return cache[n]
        for n in moving+([held] if held in ids else []):
            if boxes[n][2]<min_floor:
                min_floor=min(min_floor,bounds(sh(n))[2])
        for a,b in pairs:
            if not close(boxes[a],boxes[b]):continue
            ga,gb=group(a),group(b)
            rel=np.linalg.inv(ms[ga])@ms[gb]; key=(a,b,tuple(np.round(rel[:3].ravel(),6)))
            if key in seen:continue
            seen.add(key);pair_pose+=1
            if not close(bounds(sh(a)),bounds(sh(b))):continue
            narrow+=1
            try:v=common(sh(a),sh(b))
            except Exception as ex:no_measure.append([a,b,t,str(ex)]);continue
            if v>.001 and ((a,b) not in issues or v>issues[a,b]['volumeMm3']):issues[a,b]=dict(t=t,volumeMm3=round(v,4))
    results=[dict(pair=list(k),status='fail',witness=v) for k,v in issues.items()]
    results.extend(dict(pair=[a,b],status='open',witness={'t':t}) for a,b,t,_ in no_measure)
    return dict(results=results,narrow=narrow,distinctCandidates=pair_pose,samples=25,minFloorMm=round(min_floor,5),unresolved=len(no_measure))
def corridor():
    # Y is invariant under rotation about the transverse axis. Analytical for every NOTE pose.
    lo=min(bounds(G['Pivot A-frame · left'])[1],-bounds(G['Pivot A-frame · right'])[4])
    # Intentional inward shift at park, bringing the upper apex directly into the rim plane is a poor control.
    # Translate the real NOTE bodily through the actual plate near its pivot instead.
    bad=centerpose(park,70,50.4);bad[1,3]=220
    hit=common(place(torus,bad),G['Pivot A-frame · left'])
    return dict(minLateralGapMm=lo-177.8,controlMm3=hit)
if __name__=='__main__':
    findings={}
    for case in cases:
        findings[case]=sample(case);print(case,findings[case],flush=True);dump(HERE/'travel.json',findings)
    print(corridor(),flush=True)

