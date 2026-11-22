"""Search in three coordinates for an outward-hanging entry. All moves are replayed before any credit."""
from access import *
from scipy.ndimage import label, distance_transform_edt
from collections import deque
out=read(p/'outward.json');goal=np.array(out['outward']);theta=read(p/'access.json')['angle']
xs=goal[0]+np.arange(-55,76)*20;ys=goal[1]+np.arange(-79,80)*20;angles=np.unique(np.r_[np.arange(-49,166,2.),theta])
near=np.all((items[:,0,:2]<[xs.max()+1100,ys.max()+1100])&(items[:,1,:2]>[xs.min()-1100,ys.min()-1100]),axis=1)
ob=items[near];blocked=np.zeros((len(angles),len(xs),len(ys)),bool)
for i,a in enumerate(angles):
    sh=boxes(a,44);low=np.c_[-sh[:,1,:2],sh[:,0,2]];high=np.c_[-sh[:,0,:2],sh[:,1,2]]
    si,sj=np.where((ob[None,:,0,2]<=high[:,None,2]+.25)&(ob[None,:,1,2]>=low[:,None,2]-.25))
    lo=ob[sj,0,:2]-high[si,:2]-.25;hi=ob[sj,1,:2]-low[si,:2]+.25
    a1,b1=np.searchsorted(xs,lo[:,0],'left'),np.searchsorted(xs,hi[:,0],'right')
    a2,b2=np.searchsorted(ys,lo[:,1],'left'),np.searchsorted(ys,hi[:,1],'right')
    diff=np.zeros((len(xs)+1,len(ys)+1),np.int32)
    for x,y,sign in [(a1,a2,1),(b1,a2,-1),(a1,b2,-1),(b1,b2,1)]:np.add.at(diff,(x,y),sign)
    blocked[i]=np.cumsum(np.cumsum(diff,axis=0),axis=1)[:-1,:-1]>0
a0=np.abs(angles-125).argmin();start=(a0,np.abs(xs-6900).argmin(),79);end=(list(angles).index(theta),55,79)
lab,N=label(~blocked);ok=lab[start] and lab[start]==lab[end]
route=[];data=dict(connected=bool(ok),components=N,originComponent=int(lab[start]),goalComponent=int(lab[end]),dimensions=[len(angles),len(xs),len(ys)],source=layout['step'])
if ok:
    # Diagonal heuristic in metric units, with each translation or angle edge charged positive cost.
    weights=np.array([13.,20.,20.]);heap=[(0,0,start)];prev={};dist={start:0}
    while heap:
        _,cost,el=heapq.heappop(heap)
        if el==end:break
        if dist[el]!=cost:continue
        for axis in (0,1,2):
            for delta in (-1,1):
                e=list(el);e[axis]+=delta;e=tuple(e)
                if min(e)<0 or any(k>=n for k,n in zip(e,lab.shape)) or lab[e]!=lab[end]:continue
                new=cost+weights[axis]
                if new>=dist.get(e,1.e20):continue
                dist[e]=new;prev[e]=el
                h=np.dot(weights,np.abs(np.subtract(e,end)));heapq.heappush(heap,(new+h,new,e))
    node=end
    while True:
        route.append([float(angles[node[0]]),float(xs[node[1]]),float(ys[node[2]])])
        if node==start:break
        node=prev[node]
    route.reverse()
data.update(route=route,scope='Nominal lattice on three independent coordinates. Uses the two hook apertures open, two-degree pitch steps, 20 mm XY steps and conservative placed fixture boxes. Disconnection is a failed coarse screen, not a proof of impossibility.')
(p/'side-entry.json').write_text(json.dumps(data,indent=2));print(ok,len(route),data['goalComponent'])

