import sys, json
from pathlib import Path
import numpy as np
p=Path(__file__).parent
sys.path.insert(0,str(Path.cwd()/'sim/pressure'))
from turning_sweep import NativeTurnPath
from run_axial_shear_lift import load
case='outer-pair-aligned'; shot=p/case; rec=load(shot/'result.json');native=load(p/'outer-pair-aligned-summary.json')
inv=load(p.parent/'track-tread-candidate-20261118/route-relieved/scan.json')['inventory']
part=next(b for b in inv if b['key']=='body-702-1')
bounds=np.array(part['boundsMm']).reshape(2,3)
half=np.diff(bounds,axis=0).ravel()/2
center=bounds.mean(axis=0)
with (shot/'trace.jsonl').open('rb') as f: q=np.asarray([rec['initialQ'],*[json.loads(l)['q'] for l in f]])
path=NativeTurnPath.from_poses(np.c_[q[:,:5],q[:,-7:]],rec['dt'])
cs,ns=path.frames['cradle']
angles=2*np.pi*np.arange(8192)/8192
cb=np.cos(angles)[:,None];sb=np.sin(angles)[:,None]
best_box=best_capsule=999
for lo,hi,ids in native['unresolved']:
    if part['key'] not in ids:continue
    c,n=cs[lo],ns[lo]
    a=np.cross(n,[0.,1.,0.]);a/=np.linalg.norm(a)
    delta=c-center+152.4*(cb*a+sb*np.cross(n,a))
    best_box=min(best_box,np.linalg.norm(np.maximum(0.,np.abs(delta)-half),axis=1).min()-25.4)
    distance=np.hypot(np.maximum(0,abs(delta[:,0])-(half[0]-half[1])),delta[:,1])
    radial=np.maximum(0,np.maximum(distance-half[1],half[1]-2-distance))
    gap=np.hypot(radial,np.maximum(0,abs(delta[:,2])-half[2])).min()-25.4
    if gap<best_capsule:best_capsule=gap; witness=lo
print(best_box,best_capsule,witness,part['volumeMm3'])
