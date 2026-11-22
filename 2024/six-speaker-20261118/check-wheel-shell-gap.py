import numpy as np,json,sys,math
from pathlib import Path
p=Path(__file__).parent
sys.path.insert(0,str(Path(sys.argv[1])/'sim/pressure'))
from turning_sweep import NativeTurnPath
for case in sys.argv[2:]:
 fold=p/case;rc=json.loads((fold/'result.json').read_bytes());start=rc['start']
 trace=[json.loads(l)['q'] for l in (fold/'trace.jsonl').open('rb')]
 rows=np.array([rc['initialQ'],*trace]);q=np.column_stack((rows[:,:5],rows[:,-7:]))
 path=NativeTurnPath.from_poses(q,rc['dt']);c,n=path.frames['fixed']
 raw=json.loads((p.parent/'track-tread-candidate-20261118/route-relieved/scan.json').read_bytes());wheels=[b for b in raw['inventory'] if 'original wheel shell representation' in b['name']]
 result=[]
 for wheel in wheels:
  b=np.array(wheel['boundsMm']).reshape(2,3);wc=b.mean(axis=0);half=(b[1]-b[0])/2
  assert abs(half[0]-46.99)<.0001 and abs(half[2]-46.99)<.0001
  near=np.flatnonzero(np.linalg.norm(c-wc,axis=1)<230);best=999.;index=None;min_bound=999.
  angles=2*np.pi*np.arange(1024)/1024
  for first in range(0,len(near),32):
   ix=near[first:first+32];a=np.cross(n[ix],[0.,1.,0.]);a/=np.linalg.norm(a,axis=1)[:,None];e=np.cross(n[ix],a)
   pts=c[ix,None,:]-wc+152.4*(a[:,None,:]*np.cos(angles)[None,:,None]+e[:,None,:]*np.sin(angles)[None,:,None])
   r=np.hypot(pts[:,:,0],pts[:,:,2]);rad=np.maximum(np.maximum(42.2275-r,r-46.99),0)
   axial=np.maximum(abs(pts[:,:,1])-half[1],0.);g=np.hypot(rad,axial).min(axis=1)-25.4
   i=np.argmin(g)
   if g[i]<best:best=g[i];index=int(ix[i])
  result.append(dict(wheel=wheel['key'],nearestSampleGapMm=best,time=start+index*rc['dt'] if index is not None else None))
 (fold/'wheel-shell-gap.json').write_text(json.dumps(result,indent=2))
 print(case,result,flush=True)
