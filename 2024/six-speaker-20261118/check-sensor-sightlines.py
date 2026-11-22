"""Check possible unobstructed vertical sensing paths against every installed field leaf."""
import json
import sys
from pathlib import Path

import numpy as np

from speaker_surfaces import SpeakerSurfaces,digest

folder=Path(sys.argv[1])
s=SpeakerSurfaces(folder)
r=json.loads((folder/'opening-and-counter.json').read_bytes())
cases={p['key']:np.array(p['caseEndCornersM']) for p in r['sensorCaseEnds']}
output={}
for a,b in r['beamPairs']:
    ends=np.stack([cases[a],cases[b]])
    lo=ends.min(axis=(0,1));hi=ends.max(axis=(0,1))
    xy=np.array([[x,y] for x in np.linspace(.05,.95,9) for y in np.linspace(.05,.95,9)])
    top=lo+ (hi-lo)*np.column_stack((xy,np.ones(len(xy))))
    top[:,2]-=.00001
    length=hi[2]-lo[2]-.00002
    direction=np.tile([0.,0.,-1.],(len(xy),1))
    near=np.flatnonzero(((hi>=s.bounds[:,0])&(lo<=s.bounds[:,1])).all(axis=1))
    blocked=[set() for _ in top]
    for i in near:
        part,pkt,rot,translation=s.parts[i]
        s.nearest(i,top[:1])
        tree,vertices,faces=s.trees[pkt,part.get('mesh',part['key'])]
        hits=tree.intersect_ray(vertices,faces,(top-translation)@rot,direction@rot)
        for hit_set,blocked_by in zip(hits,blocked):
            if any(0<h[1]<length for h in hit_set):
                blocked_by.add(part['key'])
    output[a+'-'+b]=dict(openRays=sum(not x for x in blocked),rays=len(xy),centerBlockers=sorted(blocked[40]),
                         firstOpenFraction=next((uv.tolist() for uv,block in zip(xy,blocked) if not block),None))
s.close()
out=Path(sys.argv[2])
with out.open('x') as log:json.dump(dict(field=s.pins,pairs=output),log,indent=2)
print(output)
