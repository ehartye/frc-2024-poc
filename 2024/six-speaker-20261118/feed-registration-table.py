import numpy as np, json, sys
from pathlib import Path
sys.path.insert(0,str(Path.cwd()/'sim/pressure'))
from turning_sweep import NativeTurnPath
base=Path(__file__).parent
for name in sys.argv[1:]:
    folder=base/name
    rc=json.loads((folder/'result.json').read_bytes())
    rows=[]
    hits=[('start',rc['start']),('feed',rc['feedStart']),('wheel',rc['firstLauncherContact'])]
    with (folder/'trace.jsonl').open('rb') as f:
        for ln in f:
            r=json.loads(ln)
            if hits and hits[0][1] is not None and r['time']>=hits[0][1]:
                rows.append((hits.pop(0)[0],r))
    print(name,flush=True)
    for tag,r in rows:
        q=np.array(r['q'])
        k=NativeTurnPath.from_poses(np.array([np.r_[q[:5],q[-7:]]]*2),rc['dt'])
        c,n=[x[0] for x in k.frames['fixed']]
        axis=np.array([.5,0,np.sqrt(.75)]);normal=np.array([-np.sqrt(.75),0,.5])
        nip=np.array([288,0,665.35898384862])
        tilt=np.rad2deg(np.arctan2(n[0],n[2]))
        if tilt>0:tilt-=180
        print(tag,'angle',round(tilt+60,3),'offset',round((c-nip)@normal,3),'lateral',round(c[1],3),'along',round((c-nip)@axis,2),'rotY',round(np.rad2deg(np.arcsin(n[1])),3))
