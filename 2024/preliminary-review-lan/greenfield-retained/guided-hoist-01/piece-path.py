"""Rehearse each saved NOTE pose against the new hoist stock in the source cassette frame."""
from pathlib import Path
import numpy as np,json,math
p=Path(__file__).resolve().parent;root=p.parent
src=(p/'stop-slot.py').read_text(encoding='utf8');exec(compile(src.split('partsAll=')[0],'corrected package','exec'))
def m4(a):
    m=np.eye(4);m[:3,:]=np.array(a).reshape(3,4);return m
centers=np.linspace(0,2*math.pi,192,endpoint=False)
loop=np.c_[215+152.4*np.cos(centers),152.4*np.sin(centers),np.full(192,50.4),np.ones(192)]
parent=(park@loop.T).T[:,:3];out={}
for key in ['field-96','field-48','field-r01','field-r08','field-seat-recovery']:
    c=next(k for k in layout['cases'] if k['key']==key)
    A=np.array(c['series']['arm']).reshape(-1,3,4);Q=np.array(c['series']['piece']).reshape(-1,3,4)
    world=parent@Q[:,:,:3].transpose(0,2,1)+Q[:,:,3][:,None,:];arm=(world-A[:,:,3][:,None,:])@A[:,:,:3];points=arm@ip[:3,:3].T+ip[:3,3]
    a=points.min(axis=1)-25.4;b=points.max(axis=1)+25.4
    worst=1e8;row=None;candidates=[]
    for k,s in parts.items():
        bb=np.array(bounds(s));dd=np.maximum(np.maximum(bb[:3]-b,a-bb[3:]),0)
        boxDistance=np.linalg.norm(dd,axis=1)
        for i in np.flatnonzero(boxDistance<min(worst+5,65)):
            dif=np.maximum(np.maximum(bb[:3]-points[i],points[i]-bb[3:]),0.)
            bound=np.linalg.norm(dif,axis=1).min()-25.4-math.pi*152.4/192
            if bound<worst:worst=float(bound);row=[k,int(i),c['time'][i]]
            if bound<=0:candidates.append([k,int(i)])
    out[key]=dict(sampledPoses=len(a),minLowerBoundMm=worst,at=row,nonseparatedBounds=candidates)
save(p/'piece-path.json',out);print([(k,v['minLowerBoundMm'],len(v['nonseparatedBounds'])) for k,v in out.items()])

