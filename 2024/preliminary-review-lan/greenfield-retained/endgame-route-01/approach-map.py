"""Can the reverse-facing, open gripper reach the pre-capture pose from outside the stage?"""
from access import *
from collections import deque
o=read(p/'outward.json');terminal=np.array(o['outward']);theta=read(p/'access.json')['angle'];cy=terminal[1]
xs=terminal[0]+np.arange(-70,151)*10;angles=np.unique(np.r_[np.arange(-18,166,1.),theta])
near=(items[:,1,1]>cy-650)&(items[:,0,1]<cy+650)&(items[:,0,0]<xs.max()+1200)&(items[:,1,0]>xs.min()-1200)
obs=items[near];sources=np.array(ids)[near];closed=np.zeros((len(angles),len(xs)),bool);limits={}
for i,th in enumerate(angles):
    arr=boxes(th,44)
    lows=np.c_[-arr[:,1,:2],arr[:,0,2]];highs=np.c_[-arr[:,0,:2],arr[:,1,2]]
    sel=(obs[None,:,0,1]<cy+highs[:,None,1]+.25)&(obs[None,:,1,1]>cy+lows[:,None,1]-.25)&(obs[None,:,0,2]<highs[:,None,2]+.25)&(obs[None,:,1,2]>lows[:,None,2]-.25)
    j,k=np.where(sel);xmin=obs[k,0,0]-highs[j,0]-.25;xmax=obs[k,1,0]-lows[j,0]+.25
    z=np.zeros(len(xs)+1,int);np.add.at(z,np.searchsorted(xs,xmin,'left'),1);np.add.at(z,np.searchsorted(xs,xmax,'right'),-1)
    closed[i]=np.cumsum(z)[:-1]>0
start=(list(angles).index(125),np.abs(xs-6900).argmin());goal=(list(angles).index(theta),70)
parents={start:None};todo=deque([start])
assert not closed[start]
while todo:
    a=todo.popleft()
    if a==goal:break
    for da,dx in [(0,-1),(-1,0),(1,0),(0,1)]:
        k=a[0]+da,a[1]+dx
        if k in parents or not 0<=k[0]<len(angles) or not 0<=k[1]<len(xs) or closed[k]:continue
        parents[k]=a;todo.append(k)
path=[];g=goal
if g in parents:
    while g is not None:
        path.append([float(angles[g[0]]),float(xs[g[1]])]);g=parents[g]
    path.reverse()
out=dict(source=layout['step'],route=path,sampledGrid=[len(angles),len(xs)],blocked=int(closed.sum()),reachable=len(parents),goalBlocked=bool(closed[goal]),note='This is a reachability screen with fixed Y and fully open gate. Grid connectivity alone does not verify continuous motion or chain mating in the reverse orientation.')
(p/'reverse-approach.json').write_text(json.dumps(out,indent=2));print(bool(path),len(path),len(parents),closed[goal])

