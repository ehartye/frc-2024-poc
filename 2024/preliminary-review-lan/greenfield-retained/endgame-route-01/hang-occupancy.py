"""Check full robot ground and field clearance along the computed hanging trajectory."""
from access import *
climb=read(p/'original-climb.json')
source=read(root/'chain-contact-02/front48.json');assert len(source['trace'])==len(climb['time'])-30
# Actual actuated keeper and two hook surfaces intentionally react chain contact.
fieldmask=(np.linalg.norm(np.maximum(0,np.maximum(items[:,0,:2]-[5753,4104],[5753,4104]-items[:,1,:2])),axis=1)<2000)&(np.array(ids)!='working chain')
ob=items[fieldmask];on=np.array(ids)[fieldmask];positives={};floor={};chainCandidates={};lowAfterCut=(1e5,None);worst={};worstSamples={}
from scipy.spatial.transform import Rotation
for i,r in enumerate(source['trace']):
    j=i+30;xy=np.array([*climb['positions'][j],0]);mats={n:np.array(v[j]).reshape(3,4) for n,v in climb['series'].items()}
    corners=[]
    for n,v in zip(names,local):
        mx=mats[climb['rig'][n]];w=v@mx[:,:3].T+mx[:,3];lo=w.min(axis=0);hi=w.max(axis=0)
        if r['t']>=8 and lo[2]<lowAfterCut[0]:lowAfterCut=(float(lo[2]),[n,r['t']])
        if lo[2]<-.01:
            if n not in floor or lo[2]<floor[n][0]:floor[n]=[float(lo[2]),r['t']]
        corners.append([lo,hi])
    arr=np.array(corners)+xy
    ii,jj=np.where(np.all((arr[:,None,0]<=ob[None,:,1])&(arr[:,None,1]>=ob[None,:,0]),axis=2))
    for ix,fx in zip(ii,jj):positives.setdefault(names[ix]+' / '+on[fx],r['t'])
    # Expand each moving chain segment by its represented radius. Refine survivors on stock below.
    nodes=np.array(r['chain'])*1000
    lower=np.minimum(nodes[1:],nodes[:-1])-13.335
    upper=np.maximum(nodes[1:],nodes[:-1])+13.335
    arr-=xy
    ii,ss=np.where(np.all((arr[:,None,0]<=upper)&(arr[:,None,1]>=lower),axis=2))
    for ix,seg in zip(ii,ss):
        name=names[ix];chainCandidates.setdefault(name,[]).append([i,int(seg)])
out=dict(source=sha(root/'chain-contact-02/front48.json'),layout=layout['step'],steps=len(source['trace']),solids=len(names),floor=floor,smallestAllStockAfterCutMm=lowAfterCut,fieldBoxes=positives,chainCandidates=chainCandidates)
(p/'hang-occupancy.json').write_text(json.dumps(out,separators=(',',':')))
print(floor,lowAfterCut,len(positives),{n:len(v) for n,v in chainCandidates.items()})


