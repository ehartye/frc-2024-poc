"""Screen a common pickup/climb boom. Candidate loads and contact are preliminary."""
from travel import *
from math import degrees,atan2,acos,hypot
from hang import hang
# Journal near the pivot, above the NOTE. One non-extending transverse support;
# a pivoted chain latch hangs from this bearing and can align to the chain.
s,h=180,180
parts={'Climb yoke left':box(s-8,218,14,16,8,h-14),'Climb yoke right':box(s-8,-226,14,16,8,h-14),
       'Climb yoke bridge':box(s-8,-226,h,16,452,12)}
# Coordinates are the idealised load application at the center of the crosspiece.
# No credit for an unmodeled hook latch, ratchet or fastener.
target=717.55+13.335; rad=hypot(s,h)
contact=180-degrees(np.arcsin((target-501)/rad))-degrees(atan2(h,s))
end=165
cases=[];hits=[]
stocks={n:localpart for n,localpart in G.items() if n!=held and meta[n]['group'] not in ['Capture latch','Solo climber']}
local={n:place(sh,inverse) for n,sh in stocks.items() if n in moving}
fixed={n:sh for n,sh in stocks.items() if n not in moving}
# Evaluate the whole cassette at new climb angles, newly attached yoke throughout floor-to-hang excursion.
for angle in sorted(set([*np.linspace(-50,125,36),*np.linspace(contact,165,39),contact,165])):
    pm=mat(angle,pivot_x(angle),0,501);sv=mat(0,pivot_x(angle)-185,0,0)
    static={n:(posegeom(sh,sv) if n in slide else sh) for n,sh in fixed.items()}
    fixbox={b:bounds(other) for b,other in static.items()}
    candidates={**(local if angle>125 else {}),**parts}
    for n,sh in candidates.items():
        body=place(sh,pm);bb=bounds(body)
        for b,other in static.items():
            if close(bb,fixbox[b]) and (v:=common(body,other))>.01:hits.append(dict(angle=angle,first=n,second=b,volume=v))
    arm=pm@np.array([s,0,h,1]);cases.append([angle,arm[0],arm[2]])
# Test the alternative stays outside the nominal NOTE and inside lateral rails.
piece=place(torus,mat(0,215,0,50.4));gaps={n:distance(sh,piece) for n,sh in parts.items()}
w=68.56*9.80665
angles=np.linspace(contact,end,501);horizontal=s*np.cos(np.radians(angles))-h*np.sin(np.radians(angles))
peak=w*max(abs(horizontal))/1000
xf,zf=cases[-1][1:3];reaction=650.635+13.335
# Compute exact stock projection as a support-function bound on BRep vertices.
# Include the bare chassis and alternative yoke in their FINISHED hang pose.
pose=mat(end,pivot_x(end),0,501)
endstocks={**{k:posegeom(v,mat(0,pivot_x(end)-185,0,0)) if k in slide else v for k,v in fixed.items()},**{k:place(v,pose) for k,v in {**local,**parts}.items()}}
boxes={n:bounds(b) for n,b in endstocks.items()}
clear=[]
for cx,cy,cz in itertools.product((-40,40),(-25,25),(220,350)):
    down=np.array([cx-xf,cy,cz-zf]);down/=np.linalg.norm(down)
    distances={n:np.dot(np.where(down>=0,bb[3:],bb[:3])-[xf,0,zf],down) for n,bb in boxes.items()}
    k=max(distances,key=distances.get);clear.append(dict(cg=[cx,cy,cz],limiting=k,boundMm=reaction-distances[k]))
out=dict(step=sha(HERE/'retained-cassette.step'),loadPointLocalMm=[s,0,h],loadedAngleDeg=[contact,end],hookAtEndXZ=[xf,zf],
 coarseSamples=len(cases),positiveStockPairs=hits,noteGaps=gaps,provisionalGrossMassKg=68.56,peakStaticNm=peak,
 lockedDoubleLoadNm=2*peak,shaftDiameterMm=18,shaftPureTorsionMPa=16*peak*1000/(np.pi*18**3),
 minimumHangBoundMm=min(r['boundMm'] for r in clear),hangCorners=clear,assumedLoadedChainCenterMm=650.635,
 limitations='Uses a point at the cassette yoke. A rotating gated hook, chain motion, drive and bilateral load-rated stops, slide lock and measured CG are required. Static torque excludes boom self-weight and dynamics; parked study mass is a conservative comparison input.')
dump(HERE/'shared-arm.json',out);print({k:v for k,v in out.items() if k not in ['positiveStockPairs','hangCorners']},'hits',len(hits),hits[:2],flush=True)

