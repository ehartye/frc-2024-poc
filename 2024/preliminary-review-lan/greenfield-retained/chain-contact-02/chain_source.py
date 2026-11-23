from pathlib import Path
import sys,json,hashlib
p=Path(__file__).resolve().parent;sys.path.insert(0,str(p.parent/'chain-entry-01'))
from sections import slab,fieldY
import numpy as np
out={}
for n in (24,48):
    nodes=[]
    for y in np.linspace(-1210.569,1210.569,n+1):
        y=np.clip(y,-1204.0,1204.0);pts=slab(fieldY+y,.02);assert len(pts)
        nodes.append(((pts.min(axis=0)+pts.max(axis=0))/2).tolist())
    out[str(n)]=nodes
(p/'rest-chain-centers.json').write_text(json.dumps(out))

