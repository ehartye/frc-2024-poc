"""Sweep the updated hoist stock from the scored finish to PARK and to the chain-capture handoff."""
from pathlib import Path
import json,sys,numpy as np
p=Path(__file__).resolve().parent
sys.path.insert(0,str(p.parent/'endgame-route-01'))
from access import items,ids,layout
mesh=json.loads((p/'inboard-mesh.json').read_bytes())
verts={n:np.array(v['v']) for n,v in mesh.items()}
def boxes(frame,xy):
    m=np.asarray(frame).reshape(3,4)
    w=[v@m[:,:3].T+m[:,3]+np.r_[xy,0] for v in verts.values()]
    return np.array([a.min(axis=0) for a in w]),np.array([a.max(axis=0) for a in w])
out={};names=list(verts)
for key,end in [('field-climb',13.98678472),('field-park',24)]:
    c=next(x for x in layout['cases'] if x['key']==key);near={};prev=None;worst=0.
    for i,t in enumerate(c['time']):
        if t>end+.000001:break
        mat=np.array(c['series']['arm'][i]);xy=c['positions'][i];lo,hi=boxes(mat,xy)
        if prev:
            a,b,R=prev;now=mat.reshape(3,4)[:,:3];angle=np.arccos(np.clip((np.trace(now@R.T)-1)/2,-1,1))
            # Bound bow away from a chord for a 1.5 m rotation radius during one linear-translation keyframe interval.
            pad=.3+1500*(1-np.cos(angle/2));worst=max(worst,pad)
            low=np.minimum(lo,a)-pad;high=np.maximum(hi,b)+pad
            broad=np.flatnonzero(np.all(items[:,0]<=high.max(axis=0),axis=1)&np.all(items[:,1]>=low.min(axis=0),axis=1))
            ii,jj=np.where(np.all(items[broad,0]<=high[:,None],axis=2)&np.all(items[broad,1]>=low[:,None],axis=2))
            for j,k in zip(ii,jj):
                sig=names[j]+' / '+ids[broad[k]];rec=near.setdefault(sig,[t,t,0]);rec[1]=t;rec[2]+=1
        prev=lo,hi,mat.reshape(3,4)[:,:3];last=i
    out[key]=dict(knots=last+1,lastTime=c['time'][last],largestGuardMm=worst,candidates=near,scope='Enclosing interval boxes; interpolated rigid motion, fixed route peer and field envelopes. Chain-contact continuation uses the separate articulated-chain test.')
(p/'inboard-approach.json').write_text(json.dumps(out,indent=2));print([(k,len(v['candidates']),v['knots']) for k,v in out.items()])
