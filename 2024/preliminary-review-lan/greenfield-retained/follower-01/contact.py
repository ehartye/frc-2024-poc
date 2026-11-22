"""Torque-limited floor seeking with two unilateral damped contacts; no commanded tip height."""
from pathlib import Path
import json,math,itertools
p=Path(__file__).resolve().parent
f=json.loads((p/'fit.json').read_bytes());g=json.loads((p.parent/'pickup-lip/layout.json').read_bytes())
xz=f['wheelCenterPickup'];alpha=math.radians(-50);px,_,pz=g['pickPivot']
def local(x,z):
    dx=x-px;dz=z-pz
    return (math.cos(alpha)*dx+math.sin(alpha)*dz,-math.sin(alpha)*dx+math.cos(alpha)*dz)
wc=local(xz[0],xz[2]);profile=g['profiles']['Replaceable pickup lip']
def run(h,roll,groundL,groundR,k,T,radius=13):
    q=math.radians(-48.0);v=0;mn=100;mx=-100;settle=None;peaks=[0,0];dt=.00025;last=None
    for it in range(4801):
        c,s=math.cos(q),math.sin(q);wheel=pz+h+wc[0]*s+wc[1]*c
        lever=(wc[0]*c-wc[1]*s)/1000
        reactions=[]
        for i,(side,ground) in enumerate(((1,groundL),(-1,groundR))):
            comp=ground-wheel+radius-side*234.3*math.sin(math.radians(roll))
            force=max(0.,k*comp-200.*lever*v) if comp>0 else 0.
            reactions.append(force);peaks[i]=max(peaks[i],force)
        # Residual gravity-compensated downward torque with viscous velocity limiting.
        acc=(lever*sum(reactions)-T-12*v)/2.6
        t=it*dt
        tip=pz+h+min(x*s+z*c for x,z in profile);span=217*abs(math.sin(math.radians(roll)))
        if it>=3200:mn=min(mn,tip-span);mx=max(mx,tip+span)
        if abs(v)>.002 or sum(reactions)<1:settle=t
        last=[math.degrees(q),tip,reactions,v];v+=dt*acc;q+=v*dt
    return dict(setup=[h,roll,groundL,groundR,k,T],final=last,lateMinMm=mn,lateMaxMm=mx,settledAfterS=settle,peakN=peaks)
cases=[run(h,r,a,b,k,t) for h,r,(a,b),k,t in itertools.product([-2,3],[-.10,0,.10],[(-.25,.25),(.25,-.25),(0,0)],[8,24],[3,6])]
nom=run(0,0,0,0,16,5);bad=run(0,.55,.9,-.9,8,6);off=run(3,0,0,0,16,5,radius=10)
res=dict(nominal=nom,trials=cases,highRollControl=bad,wornRadiusControl=off,targetHeightMm=[3.5,6.5],sampleWindowS=[.8,1.2],passCount=sum(3.5<=u['lateMinMm'] and u['lateMaxMm']<=6.5 for u in cases),limits='Two planar lane heights relative to the floor under the NOTE. Tire stiffness, roll, floor load, arm inertia and residual torque are assumptions for measurement.')
(p/'response.json').write_text(json.dumps(res,indent=2));print(res['passCount'],len(cases),min(u['lateMinMm'] for u in cases),max(u['lateMaxMm'] for u in cases),nom, bad['lateMaxMm'],off['lateMinMm'])

