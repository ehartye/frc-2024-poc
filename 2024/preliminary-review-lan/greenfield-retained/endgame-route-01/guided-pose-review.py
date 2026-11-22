"""Compare the solved chassis pitch and hook slide to the registered field/peer envelopes at all saved poses."""
from pathlib import Path
import numpy as np,sys,json
from access import items,ids,sha
p=Path(__file__).resolve().parent
scope={'__file__':str(p/'compare-hangs.py')}
exec(compile((p/'compare-hangs.py').read_text().split('def trial')[0],'original field and robot members','exec'),scope)
read=scope['read'];moving=set(read(p/'guided-hooks.json')['hookGroup'])
nms=scope['bodyNames'];base=scope['base'];orient=scope['orient']
series={}
for label in sys.argv[1:]:
    path=p/(label+'.json');data=read(path);candidates={};mins=[];leaf=scope['scene']['rig']
    for r in data['trace']:
        m=scope['transforms'](dict(**r,strokeMm=0.),r['gate']*1000,0)
        a=np.deg2rad(r['angle']);delta=orient(r['q'])@np.array([np.cos(a),0,np.sin(a)])*r['draw']*1000
        tr=np.array([m[leaf[n]] for n in nms]);tr[[n in moving for n in nms],:3,3]-=delta
        k=tuple(np.array([np.min(v,axis=0)-.30,np.max(v,axis=0)+.30]) for n,t in zip(nms,tr) for v in [scope['verts'][n]@t[:3,:3].T+t[:3,3]])
        b=np.array(k);window=np.all(np.max(b[:,1],axis=0)>=items[:,0],axis=1)&np.all(np.min(b[:,0],axis=0)<=items[:,1],axis=1);idx=np.flatnonzero(window)
        ii,jj=np.where(np.all(b[:,None,1]>=items[idx,0],axis=2)&np.all(b[:,None,0]<=items[idx,1],axis=2))
        for i,j in zip(ii,jj):
            key=nms[i]+' / '+ids[idx[j]];x=candidates.setdefault(key,[r['t'],r['t'],0]);x[1]=r['t'];x[2]+=1
        if r['t']>5.15:mins.append(float(b[:,0,2].min()))
    series[label]=dict(inputs=sha(path),samples=len(data['trace']),minDisabledFloorMm=min(mins),fieldCandidates=candidates)
(p/'guided-pose-review.json').write_text(json.dumps(series,indent=2));print({k:(v['minDisabledFloorMm'],list(v['fieldCandidates'].keys())) for k,v in series.items()})

