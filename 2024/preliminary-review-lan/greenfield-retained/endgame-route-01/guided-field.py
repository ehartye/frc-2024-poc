"""Conservative whole-field translation sweep of the closed gate plus chain and peer candidates."""
from pathlib import Path
import numpy as np,json
p=Path(__file__).resolve().parent
from access import items,ids,fieldpath,sha
scope={'__file__':str(p/'compare-hangs.py')}
exec(compile((p/'compare-hangs.py').read_text().split('def trial')[0],'native stock poses','exec'),scope)
read=scope['read'];r=read(p.parent/'chain-contact-02/front48.json')['trace'][0]
m=scope['transforms'](r,0,0)
old=read(p/'guided-hooks.json');selected=set(old['hookGroup']);nir=scope['bodyNames']
axis=np.array([np.cos(np.deg2rad(r['angle'])),0,np.sin(np.deg2rad(r['angle']))])
candidates={}
for n in nir:
    matrix=m[scope['scene']['rig'][n]]
    v=scope['verts'][n]@matrix[:3,:3].T+matrix[:3,3]
    d=0 if n in selected else axis*100
    lo=np.minimum(v.min(axis=0),v.min(axis=0)+d)-.30
    hi=np.maximum(v.max(axis=0),v.max(axis=0)+d)+.30
    for i in np.flatnonzero(np.all(items[:,0]<=hi,axis=1)&np.all(items[:,1]>=lo,axis=1)):
        candidates.setdefault(ids[i],set()).add(n)
out=dict(input=sha(fieldpath),robot=old['source'],stroke=100.,travel=axis.tolist(),bodies=len(nir),fieldPeerBounds=len(items),candidates={i:sorted(b) for i,b in candidates.items()},scope='Full fixed-height-band field and archived peers, conservative translated swept bounds, constant pitch only. Working chain uses its initial slices. A candidate needs geometry or dynamic closure.')
(p/'guided-field.json').write_text(json.dumps(out,indent=2));print({i:len(b) for i,b in candidates.items()})

