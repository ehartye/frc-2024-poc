from shared_arm import boxes,endstocks,xf,zf,reaction,HERE,dump
import itertools,numpy as np
from layout import viewmesh
centers=np.array(list(itertools.product(np.arange(-40,40.01,4),np.arange(-25,25.01,2.5),np.arange(220,350.01,5))))
bb=np.array(list(boxes.values()));p=np.array([xf,0,zf]);dirs=centers-p;dirs/=np.linalg.norm(dirs,axis=1)[:,None]
lower=[];rows=list(boxes)
for d in np.array_split(dirs,35):
    v=np.where(d[:,None,:]>=0,bb[None,:,3:],bb[None,:,:3]);lower.extend(reaction-np.max(np.sum((v-p)*d[:,None,:],axis=2),axis=1))
idx=int(np.argmin(lower));d=dirs[idx]
# Use vertices on actual stock when the bounding box alone predicts contact.
worst=float('inf');body=''
for name,sh in endstocks.items():
    vv=np.array(viewmesh(sh)['v']);height=reaction-np.max((vv-p)@d)
    if height<worst:worst=float(height);body=name
out=dict(samples=len(centers),worstBoxLowerMm=float(lower[idx]),centerMm=centers[idx].tolist(),lowestStockVertexMm=worst,body=body,scope='Static single-point suspension. Assumed CG range and chain height. Surface-vertex witness at one interior CG; no chain dynamics or build credit.')
dump(HERE/'loaded-pose.json',out);print(out)

