"""Conservative separation of the five idle floor pieces on the registered first pickup."""
from pathlib import Path
import hashlib,json
import numpy as np
root=Path(__file__).parent
pick=root.parent/'turn-lift-20261118/end-to-end-fine-left'
archive=root.parent/'track-tread-candidate-20261118/route-relieved/scan.json'
with archive.open('rb') as f: scan=json.load(f)
rows=[]
pins={}
def pin(path):
    with path.open('rb') as f:pins[path.name+'-'+path.parent.name]=hashlib.file_digest(f,'sha256').hexdigest()
def read(path):pin(path);return json.loads(path.read_bytes())
stop=read(pick/'stop/receipt.json')
stopat=stop['trace'][0]['timeS']
ground=read(pick/'ground/receipt.json')
def append(t,q,n):rows.append([t,*q,*n])
append(0.,[0.,0.,0.,0.,0.],[.658,0.,.0254])
for sample in [r for r in ground['trace'] if r['timeS'] < stopat-1e-9]+stop['trace']:
    c=sample['chassis']
    append(sample['timeS'],[c['x'],c['y'],c['heading'],0.,0.],[*sample['note']['p'],.0254])
for file in [pick/'lift/trace.jsonl',root/'result-refined/trace.jsonl']:
    pin(file)
    with file.open() as f:
        for l in f:
            r=json.loads(l);q=r['q'];append(r.get('clock',r.get('time')),q[:5],q[-7:-4])
rows=np.array(rows)
assert np.all(np.diff(rows[:,0])>0) and max(np.diff(rows[:,0]))<.001
pivot=np.array([.178,0,.0254])
q=rows[:,:6];dp=np.diff(q,axis=0)
parts={}
for group in ('fixed','carriage','cradle'):
    part=[p for p in scan['inventory'] if p['motionGroup']==group]
    parts[group]=np.array([p['boundsMm'] for p in part]).reshape(-1,2,3)/1000,[p['key'] for p in part]
results={}
for label,fieldxy in [('wing-middle',(114,218.625)),('wing-outer',(114,275.625)),('center-middle',(325.625,161.625)),
                      ('center-amp-one',(325.625,227.625)),('center-amp-two',(325.625,293.625))]:
    mark=np.array([.658+(fieldxy[1]-161.625)*.0254,(114-fieldxy[0])*.0254,.0254])
    delta=mark[:2]-q[:,1:3];c=np.cos(q[:,3]);s=np.sin(q[:,3])
    fix=np.column_stack((c*delta[:,0]+s*delta[:,1],-s*delta[:,0]+c*delta[:,1],np.full(len(q),.0254)))
    car=fix.copy();car[:,2]-=q[:,4]
    cp=np.cos(q[:,5]);sp=np.sin(q[:,5]);rel=car-pivot
    cradle=pivot+np.column_stack((cp*rel[:,0]+sp*rel[:,2],rel[:,1],-sp*rel[:,0]+cp*rel[:,2]))
    radial=np.linalg.norm(delta,axis=1)
    pad=np.linalg.norm(dp[:,1:3],axis=1)+np.maximum(radial[:-1],radial[1:])*abs(dp[:,3])+1e-6
    cpad=pad+abs(dp[:,4])
    rpad=cpad+(np.maximum(np.linalg.norm(rel[:-1],axis=1),np.linalg.norm(rel[1:],axis=1))+cpad+.178)*abs(dp[:,5])
    flat=np.broadcast_to([.1778,.1778,.0254],fix.shape)
    tilted=.0254+.1524*np.column_stack((abs(cp),np.ones(len(cp)),abs(sp)))
    group_bounds={}
    for key,center,extent,travel in [('fixed',fix,flat,pad),('carriage',car,flat,cpad),('cradle',cradle,tilted,rpad)]:
        bounds,ids=parts[key]
        best=np.inf;ambiguous=set();tmin=None
        for i in range(0,len(q)-1,128):
            n=min(128,len(q)-1-i);sl=slice(i,i+n)
            ex=extent[sl]+travel[sl,None]
            lower=center[sl]-ex;upper=center[sl]+ex
            # A positive gap in any coordinate excludes overlap. Only refine candidate parts.
            max_upper=upper.max(axis=0);min_lower=lower.min(axis=0)
            broad=np.maximum(bounds[:,0]-max_upper,min_lower-bounds[:,1]).max(axis=1)
            besthere=broad.min()
            eligible=np.flatnonzero(broad<=max(0.,min(best,besthere+.15)))
            if len(eligible)==0:continue
            bl=bounds[eligible]
            gap=np.maximum(bl[None,:,0]-upper[:,None,:],lower[:,None,:]-bl[None,:,1]).max(axis=2)
            g=gap.min()
            if g<best:
                a,b=np.unravel_index(np.argmin(gap),gap.shape)
                best=g;tmin=q[i+a,0]
            ambiguous.update(ids[eligible[j]] for j in np.where(gap<=0)[1])
            # Omitted parts have broad lower bounds; keep conservative minimum.
            if len(eligible)<len(bounds):
                masked=broad.copy();masked[eligible]=np.inf
                best=min(best,float(masked.min()))
        group_bounds[key]=dict(minimumAxisGapM=best,nearS=tmin,ambiguous=sorted(ambiguous))
    sep=np.linalg.norm(rows[:-1,6:]-mark,axis=1)-.3556-np.linalg.norm(np.diff(rows[:,6:],axis=0),axis=1)
    results[label]=dict(bodyGroups=group_bounds,otherPieceGapM=float(sep.min()),chassisCenterNearestM=float(radial.min()))
out=dict(intervals=len(rows)-1,beginS=rows[0,0],endS=rows[-1,0],sourceNative=scan['sourceSha256'],sources=pins,results=results)
(root/'unused-floor-note-prefix.json').write_text(json.dumps(out,indent=2))
print(json.dumps(results))
