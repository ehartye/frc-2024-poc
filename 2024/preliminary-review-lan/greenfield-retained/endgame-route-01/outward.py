"""Compare field-facing orientations on the same boom motion, including the actual inner rail."""
from access import *
old=read(p/'original-climb.json');source=read(root/'chain-contact-02/front48.json')
xy0=np.array(old['positions'][30]);axis=xy0+np.array(source['trace'][0]['chain'])[24,:2]*1000
rot=np.diag([-1.,-1.,1.]);I=np.eye(3)
def currentBounds(i):
    mtr={k:np.array(v[i+30]).reshape(3,4) for k,v in old['series'].items()}
    arr=[]
    for n,v in zip(names,local):
        m=mtr[old['rig'][n]];w=v@m[:,:3].T+m[:,3];arr.append([w.min(axis=0),w.max(axis=0)])
    return np.array(arr)
mask=(np.array(ids)!='working chain')&np.all((items[:,0,:2]<axis+2300)&(items[:,1,:2]>axis-2300),axis=1)
targets=items[mask];ob=np.array(ids)[mask]
def hitsArray(arr):
    m,n=np.where(np.all((arr[:,None,1]>=targets[None,:,0]-.50)&(arr[:,None,0]<=targets[None,:,1]+.50),axis=2))
    return [[names[i],ob[j]] for i,j in zip(m,n)]
def transfer(arr,angle,position):
    if angle:arr=np.stack([np.c_[-arr[:,1,:2],arr[:,0,2]],np.c_[-arr[:,0,:2],arr[:,1,2]]],axis=1)
    return arr+np.r_[position,0]
acc={str(s):{} for s in (0,180)}
for i,r in enumerate(source['trace']):
    b=currentBounds(i)
    for s,xy in [(0,xy0),(180,2*axis-xy0)]:
        for n,k in hitsArray(transfer(b,s,xy)):acc[str(s)].setdefault(n+' / '+k,[]).append(i)
entry=read(root/'keeper-drive-02/root-entry.json')['poses']
pre=2*axis-np.array([entry['0']['x'],axis[1]])
# Inspect rotated transit endpoints on both sides of the straight working chain.
def query(a,g,x,y):
    arr=transfer(boxes(a,g),180,[x,y]);hit=hitsArray(arr)
    live=items[np.array(ids)=='working chain'];ii,jj=np.where(np.all((arr[:,None,1]>=live[None,:,0]-.50)&(arr[:,None,0]<=live[None,:,1]+.50),axis=2));return hit,list({names[j] for j in ii})
profiles={}
for x in [pre[0]-900,pre[0]-450,pre[0]-200,pre[0],pre[0]+250,pre[0]+550]:
    phases=[]
    for a in [-18,40,65,80,entry['0']['th'],100,125]:
        h,ch=query(a,44,x,axis[1]);phases.append(dict(angle=a,field=len(h),chain=ch,first=h[:1]))
    profiles[str(x)]=phases
out=dict(axis=axis.tolist(),existing=xy0.tolist(),outward=pre.tolist(),source=layout['step'],staticOverlaps=acc,prepose=profiles)
(p/'outward.json').write_text(json.dumps(out,separators=(',',':')));print(len(acc['0']),len(acc['180']),[(x,[(r['angle'],r['field'],len(r['chain'])) for r in v]) for x,v in profiles.items()])

