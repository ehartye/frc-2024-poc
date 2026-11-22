"""Replay the 24 s fallback with current stock, peer occupancy and registered field fixtures."""
from pathlib import Path
import numpy as np,json,sys,heapq,itertools,math,hashlib
from heapq import heappop,heappush
from math import hypot
from route import fielddata,ignore,design,scheduled,mat
p=Path(__file__).resolve().parent;root=p.parent;env=globals()
def dump(k,d):k.write_text(json.dumps(d,indent=2))
def build_bounds():
    cs=next(c for c in design['cases'] if c['key']=='pickup-to-shot');zero=mat(0,scheduled(0),501);park=mat(125,185,501);inv=np.linalg.inv(park)
    shapes=[];names=[]
    for n,r in cs['rig'].items():
        if n in cs['removed'] or r=='piece':continue
        mtx=zero@inv if r in ('arm','flap') else mat(0,scheduled(0)-185) if r=='slide' else np.eye(4)
        v=np.array(design['bodies'][n]['v'])@mtx[:3,:3].T+mtx[:3,3]
        shapes.append([*v.min(axis=0),*v.max(axis=0)]);names.append(n)
    return np.array(shapes),names
ab,roster=build_bounds();assert len(roster)>=150
src=(root/'park_rehearsal.py').read_text()
src=src[src.index('frames=d['):];i=src.index('mx=mat(');j=src.index('mins=ab',i);src=src[:i]+src[j:]
src=src[:src.index("dump(HERE/")]
d=json.loads((root/'park-inputs.json').read_bytes());exec(compile(src,'park route','exec'),env)
obs3=[]
for part in fielddata['parts']:
    if part['path'][1] in ignore:continue
    for aa,bb in part.get('tightBounds',[part['bounds']]):
        if aa[2] < maxs[2]/1000+.002 and bb[2]>1e-4:obs3.append([np.array(aa)*1000,np.array(bb)*1000,'/'.join(part['path'])])
conflicts=[];pairs=0
for ii,(aa,bb) in enumerate(zip(pts[:-1],pts[1:])):
    a=np.r_[aa*25.4,0];b=np.r_[bb*25.4,0];vel=b-a
    for low,high,n in obs3:
        if np.any(low>np.maximum(a,b)+maxs+.25) or np.any(high<np.minimum(a,b)+mins-.25):continue
        for j,row in enumerate(ab):
            entry=low-row[3:]-.25;leave=high-row[:3]+.25
            mn=0.;mx=1.;pairs+=1
            for k in range(3):
                if abs(vel[k])<1e-8:
                    if a[k]<entry[k] or a[k]>leave[k]:mn=2;break
                else:
                    t1=(entry[k]-a[k])/vel[k];t2=(leave[k]-a[k])/vel[k]
                    mn=max(mn,min(t1,t2));mx=min(mx,max(t1,t2))
            if mn<=mx:conflicts.append([ii,roster[j],n,mn,mx])
out.update(parent=design['step'],stockCount=len(ab),registeredBroadPairs=pairs,registeredCandidates=conflicts,scope='Fixed heading empty and horizontal throughout. Tested continuous peer envelopes and swept stock bounds against registered field leaf boxes. Any remaining box overlap requires a narrower measurement.')
dump(Path(__file__).resolve().parent/'park-result.json',out);print(total,len(roster),maxs,len(conflicts),conflicts[:3])



