"""Recover the real tire cross-section on the speaker's powered grip shaft."""
from pathlib import Path
import numpy as np,json,math,hashlib
p=Path(__file__).resolve().parent;fld=Path('C:/Users/ehart/Documents/FRC Native/2024/six-speaker-20261118/field-source/blue-speaker')
cfg=json.loads((fld/'panels.json').read_bytes());pack=np.load(fld/cfg['mesh'])
part=next(v for v in cfg['parts'] if v['key']=='p75')
center=np.mean(part['blueFieldBoundsM'],axis=0);v=pack['p75_v'];tri=v[pack['p75_t']]
out=[]
for offsetMm in [-8,0,8]:
    y=center[1]+offsetMm/1000+1e-7
    tt=tri[(tri[:,:,1].min(1)<y)&(tri[:,:,1].max(1)>y)]
    segments=[]
    for row in tt:
        xy=[]
        for a,b in zip(row,np.roll(row,-1,axis=0)):
            if (a[1]-y)*(b[1]-y)<0:
                pt=a+(b-a)*(y-a[1])/(b[1]-a[1]);xy.append((pt-center)[[0,2]])
        if len(xy)==2:segments.append(xy)
    A,B=np.array(segments).transpose(1,0,2);D=B-A;radii=[]
    cross=lambda s,t:s[...,0]*t[...,1]-s[...,1]*t[...,0]
    for theta in np.arange(720)*math.pi/360:
        ray=np.array([math.cos(theta),math.sin(theta)])
        den=cross(ray,D);u=np.divide(cross(A,ray),den,out=np.full(len(den),-1.),where=abs(den)>1e-14)
        r=np.divide(cross(A,D),den,out=np.full(len(den),-1.),where=abs(den)>1e-14)
        radii.append(1000*r[(u>=0)&(u<=1)&(r>0)].max())
    out.append(dict(offsetMm=offsetMm,rangeMm=[min(radii),max(radii)],radiiMm=radii))
(p/'wheel-section.json').write_text(json.dumps(dict(source=cfg['meshSha256'],grip='p75',slices=out)));print([v['rangeMm'] for v in out])

