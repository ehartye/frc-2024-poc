"""Resolve the two channel hits in the coarse approach boxes in the local channel frame."""
from pathlib import Path
import json,numpy as np,sys
p=Path(__file__).resolve().parent;root=p.parent
sys.path.insert(0,str(root/'endgame-route-01'))
from access import layout,park,ip,tri
read=lambda f:json.loads(f.read_bytes())
case=next(c for c in layout['cases'] if c['key']=='field-climb')
src=read(root/'endgame-route-01/original-climb.json');anchor=np.r_[src['positions'][30],0.]
mesh=read(p/'inboard-mesh.json');out={}
for n,body in mesh.items():
    if not n.startswith('Hook guide channel'):continue
    v=np.asarray(body['v'])@ip[:3,:3].T+ip[:3,3];lo=v.min(0);hi=v.max(0);ends=[];mins=[]
    for i,t in enumerate(case['time']):
        if t<13.39 or t>13.987:continue
        h=np.eye(4);h[:3]=np.array(case['series']['arm'][i]).reshape(3,4);h[:2,3]+=case['positions'][i];h=h@park;H=h[:3,:3];O=h[:3,3]
        chain=np.array(case['chain'][i])+anchor
        q=(tri-O)@H;f=(chain-O)@H
        def separation(low,high):
            return np.linalg.norm(np.maximum(0,np.maximum(lo-high,low-hi)),axis=1).min()
        static=separation(q.min(1),q.max(1))
        active=separation(np.minimum(f[:-1],f[1:]),np.maximum(f[:-1],f[1:]))-13.335
        w=v@H.T+O
        if ends:
            before,priorChain,priorHoist,priorStatic,priorLive=ends[-1]
            motion=np.linalg.norm(w-priorHoist,axis=1).max()
            motionChain=np.linalg.norm(priorChain-chain,axis=1).max()
            mins.append([before,t,min(priorStatic,static)-motion-.6,min(priorLive,active)-motion-motionChain-.6])
        ends.append((t,chain,w,static,active))
    out[n]=dict(minOriginalMeshMm=min(a[2] for a in mins),minLiveChainCapsuleMm=min(a[3] for a in mins),intervals=len(mins))
(p/'inboard-approach-chain.json').write_text(json.dumps(out,indent=2));print(out)
