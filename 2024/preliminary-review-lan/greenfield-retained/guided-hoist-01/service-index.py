"""Check candidate service index angles using the original occupied assembly, battery and new hoist."""
from pathlib import Path
import numpy as np
p=Path(__file__).resolve().parent
exec(compile((p/'service.py').read_text().split('out={}')[0],'installed stock','exec'))
excluded={'Battery and terminal case','Pack lead','Distribution lead','Electronics deck','Chambered NOTE'}
parking=read(p/'inboard-mesh.json')
d=read(p.parent.parent/'www/data.json');variant=d['concepts']['field'];mesh={**d['bodies'],**variant['add'],**parking}
from sweep import place as posegeom
roleOrig=active['rig']
def scan(angle):
    mx=mat(angle,scheduled(angle),0,501);m=mx@ip;sl=offset(scheduled(angle)-185);hinge=offset(457.5,0,96)
    flap=mx@hinge@mat(float(90*np.clip((angle-102)/18,0,1)),0,0,0)@np.linalg.inv(hinge)@ip
    gaps={};hits={};pairs=0
    for n in original.keys()|parts.keys():
        if n in excluded or n in active['removed'] or n not in mesh:continue
        v=np.array(mesh[n]['v']);rig='arm' if n in parts else roleOrig.get(n,'frame')
        h=m if rig in ('arm','keeper') else sl if rig=='slide' else flap if rig=='flap' else np.eye(4)
        w=v@h[:3,:3].T+h[:3,3];a=np.r_[w.min(0),w.max(0)];b=np.array(bounds(sweep));lb=np.linalg.norm(np.maximum(0,np.maximum(a[:3]-b[3:],b[:3]-a[3:])))
        if lb>20:continue
        s=posegeom(parts[n],mx) if n in parts else posegeom(original[n],h);dist=distance(s,sweep);gaps[n]=dist;pairs+=1
        if dist<.001:
            volx=common(s,sweep)
            if volx>.01:hits[n]=volx
    return dict(worstGapMm=min(gaps.values(),default=20),hits=hits,examined=pairs)
out={}
for a in [75.386421,65,50,35,20,0,-20,-40,-49.74,90,100]:
    out[a]=scan(a);save(p/'service-index.json',out)
    print(a,len(out[a]['hits']),out[a]['worstGapMm'],flush=True)

