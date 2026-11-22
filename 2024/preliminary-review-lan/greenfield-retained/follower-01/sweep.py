"""Conservative swept separation of follower hardware from stationary and slide-mounted neighbors."""
from probe import *
import math
from stroke import scheduled
f=json.loads((p/'fit.json').read_bytes());r=json.loads((p/'response.json').read_bytes())
d=json.loads((p.parent/'catch-layout-03/review-data.json').read_bytes())
rig=next(c['rig'] for c in d['cases'] if c['key']=='intake')
q0=r['nominal']['final'][0];ii=np.linalg.inv(park);msh=json.loads((p/'mesh.json').read_bytes())
vs=np.vstack([v['v'] for n,v in msh.items()]);vs=vs@ii[:3,:3].T+ii[:3,3]
others=[(n,np.array(bounds(sh)),rig.get(n,'frame')) for n,sh in names.items() if n not in f['modified'] and rig.get(n,'frame') not in ('arm','flap','piece')]
minimum=1e6;maximum=-1e6;worst=None;maxfront=-1e6
# Linear interpolation in joint angle. Lipschitz bound 1.42 m/rad for pivot + farthest moving vertex.
Q=np.linspace(q0,165,2151);dq=math.radians(Q[1]-Q[0]);buffer=1420*dq/2
for q in Q:
    mix=np.clip((q-q0)/(125-q0),0,1)
    X=scheduled(q)+(1-mix)*(169.3625892825837-scheduled(q0));pose=mat(q,X,0,501)
    world=vs@pose[:3,:3].T+pose[:3,3];maxfront=max(maxfront,world[:,0].max())
    # Use each cheek independently; a combined bounding box would fill the whole mouth.
    for sign in [-1,1]:
        vv=world[(vs[:,1]*sign)>0];low=vv.min(axis=0);high=vv.max(axis=0)
        for n,bb,role in others:
            dx=(X-185) if role in ('slide','pin') else 0
            a=bb[:3]+[dx,0,0];b=bb[3:]+[dx,0,0]
            sep=np.linalg.norm(np.maximum(0,np.maximum(a-high,low-b)))
            if sep<minimum:minimum=sep;worst=[n,sign,q]
print(minimum,worst,maxfront,buffer)
res=dict(minimumStationaryBoxGapMm=minimum,nearest=worst,betweenPoseAllowanceMm=buffer,frontSampleMm=maxfront,frontCertifiedMm=maxfront+buffer,limitMm=644.8,rangeDeg=[q0,165],qualifier='Frame and slide stock; the NOTE and the permitted carpet contact are checked separately.')
(p/'sweep.json').write_text(json.dumps(res,indent=2))


