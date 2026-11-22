"""Resolve the two broad route failures using the occupied shapes at the actual first-pickup angle and heading."""
from pathlib import Path
import sys,json,math,numpy as np
p=Path(__file__).resolve().parent;root=p.parent
sys.path.insert(0,str(root/'endgame-route-01'))
from access import items,ids,pose,sha
r=json.loads((root/'split-field-02/plan.json').read_bytes());leg=r['plan'][0];assert leg['id']=='middle'
d=json.loads((root.parent/'../../'.replace('../../','www/data.json')).read_bytes());c=d['concepts']['field'];bodies={**d['bodies'],**c['add'],**json.loads((p/'mesh.json').read_bytes())}
base=next(q for q in c['cases'] if q['key']=='field-r08');rig={k:v for k,v in base['rig'].items() if k not in base['removed'] and k!='Chambered NOTE'};rig.update({k:'arm' for k in json.loads((p/'mesh.json').read_bytes())})
q=leg['yaw'];rot=np.array([[math.cos(q),-math.sin(q),0],[math.sin(q),math.cos(q),0],[0,0,1]])
sol=[]
for n,b in zip(ids,items):
    if n.startswith('FE-') or n=='working chain':sol.append((n,b))
for L in r['plan'][1:]:
    xy=np.array(L['mark'])*1000;sol.append((L['id'],np.array([np.r_[xy-177.8,0],np.r_[xy+177.8,50.8]])))
keys=[n for n,b in sol];bs=np.array([b for n,b in sol])
def hits(a,b,minmax):
    a=np.r_[np.array(a)*1000,0];vel=np.r_[np.array(b)*1000,0]-a
    lo=bs[:,0]-minmax[1]-5;hi=bs[:,1]-minmax[0]+5;t0=np.zeros(len(keys));t1=np.ones(len(keys))
    for k in range(3):
        if abs(vel[k])<1e-9:t0[(a[k]<lo[:,k])|(a[k]>hi[:,k])]=2
        else:
            t=(lo[:,k]-a[k])/vel[k];u=(hi[:,k]-a[k])/vel[k];t0=np.maximum(t0,np.minimum(t,u));t1=np.minimum(t1,np.maximum(t,u))
    return [(keys[i],t0[i],t1[i]) for i in np.flatnonzero(t0<=t1)]
for variant in ('dev','R01','R08'):
    src=root/'split-field-02'/(variant+'-middle-96.json');carry=json.loads(src.read_bytes())
    angles=carry['pitchDegrees'];heading=pose(angles)
    # During the first loaded segment this leg intentionally carries at pickup pitch and fixed heading.
    assert abs(carry['startLift']-2.82-leg['loadedT'][0])<1e-8
    # Actual pickup slide position uses the same initialization as the carry simulation.
    from stroke import scheduled
    sliderRest=json.loads((root/'pickup-lip/layout.json').read_bytes())['pickPivot'][0];x0=scheduled(angles);delta=sliderRest-x0
    for k in ['slide','arm','flap']:heading[k][:3,3]+=[delta,0,0]
    out=[]
    for n,group in rig.items():
        m=heading[group];v=np.array(bodies[n]['v'])@m[:3,:3].T+m[:3,3]
        if group=='flap':
            # The passive flap can swing on admission. Its arc is inside this full local 80mm disk.
            h=heading['arm']@np.r_[np.array([185+457.5*math.cos(math.radians(125))-96*math.sin(math.radians(125)),0,501+457.5*math.sin(math.radians(125))+96*math.cos(math.radians(125))]),1]
            v=np.vstack([v,h[:3]+[-80,-50,-80],h[:3]+[80,50,80]])
        vv=v@rot.T
        bb=[vv.min(0),vv.max(0)]
        for typ,a,b in [('pickup',leg['out'][-1],leg['loaded'][0]),('carry',leg['loaded'][0],leg['loaded'][1])]:
            for hit in hits(a,b,bb):out.append([n,typ,*hit])
    (p/('first-'+variant+'.json')).write_text(json.dumps(dict(angle=angles,slideInitialMm=sliderRest,bodyCount=len(rig),marginMm=5,remaining=out,scope='Fixed heading and actual pickup pitch throughout both segments. Each actual part contributes its own occupied swept bounds. Dormant NOTES and stationary field include a 5 mm guard. Free NOTE in cassette independently tracked.'),indent=2));print(variant,out[:8],len(out))

