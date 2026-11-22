import json,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path.cwd()/'sim/pressure'))
from turning_sweep import NativeTurnPath
base=Path(__file__).parent
for name in sys.argv[1:]:
    folder=base/name
    with (folder/'trace.jsonl').open('rb') as f:
        f.seek(-8192,2);lines=f.readlines()
        r=json.loads(lines[-1] if lines[-1].endswith(b'\n') else lines[-2])
    q=np.array(r['q'])
    path=NativeTurnPath.from_poses(np.array([np.r_[q[:5],q[-7:]]]*2),.00005)
    pos=path.frames['cradle'][0][0]
    print(name,round(r['time'],4),np.round(pos,2),round(r['beltNormalN'],2),round(r['wheelNormalN'],2))
