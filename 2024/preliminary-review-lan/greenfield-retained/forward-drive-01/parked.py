from pathlib import Path
import sys,json,numpy as np
p=Path(__file__).resolve().parent;sys.path.insert(0,str(p.parent/'assembly-recheck-01'))
from sweep import load_solids,name,bounds,common
G={name(n):s for n,s in load_solids(p/'forward-drive.step')[0].items()};changed=json.loads((p/'result.json').read_bytes())['modified'];B={n:np.array(bounds(s)) for n,s in G.items()}
hits=[]
for i,n in enumerate(changed):
    for m,s in G.items():
        if m in changed[:i+1] or m=='Chambered NOTE':continue
        if (np.minimum(B[m][3:],B[n][3:])-np.maximum(B[m][:3],B[n][:3])>1e-5).all():
            val=common(s,G[n])
            if val>.001:hits.append([n,m,val])
print(hits)

