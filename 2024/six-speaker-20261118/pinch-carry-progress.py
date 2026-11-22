import json, sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path.cwd()/'sim/pressure'))
from turning_sweep import NativeTurnPath
base=Path(__file__).parent
for case in sys.argv[1:]:
    fold=base/case
    rc=json.loads((fold/'result.json').read_bytes())
    print(case)
    times=np.arange(0,.85,.1)+rc['start']
    with (fold/'trace.jsonl').open('rb') as file:
        for ln in file:
            r=json.loads(ln)
            if len(times) and r['time']>=times[0]:
                q=np.array(r['q'])
                m=NativeTurnPath.from_poses(np.array([np.r_[q[:5],q[-7:]]]*2),rc['dt'])
                c,n=[i[0] for i in m.frames['cradle']]
                print(round(r['time']-rc['start'],3),np.round(c,2),round(r['beltNormalN'],2))
                times=times[1:]
