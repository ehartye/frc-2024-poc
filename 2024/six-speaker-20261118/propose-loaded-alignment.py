import json, math, sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path.cwd()/'sim/pressure'))
from turning_sweep import NativeTurnPath
base=Path(__file__).parent
bound=json.loads((Path.cwd()/'sim/mujoco/evidence/native-launcher-binding-20261118.json').read_bytes())
for case in sys.argv[1:]:
    file=base/case/'trace.jsonl'
    with file.open('rb') as f:
        f.seek(-8192,2);lines=f.readlines();row=json.loads(lines[-1] if lines[-1].endswith(b'\n') else lines[-2])
    q=np.array(row['q']);m=NativeTurnPath.from_poses(np.array([np.r_[q[:5],q[-7:]]]*2),.0005)
    c,n=[v[0] for v in m.frames['fixed']]
    pitch=bound['pitchRad'];axis=np.array([np.cos(pitch),0,np.sin(pitch)])
    normal=np.array([-np.sin(pitch),0,np.cos(pitch)])
    error=np.rad2deg(pitch+math.atan2(n[0],n[2]))
    offset=(c-1000*np.array(bound['stationM']))@normal
    # Offline hypothesis derived from the first two measured feed paths. The
    # contact starts roughly 140 mm downstream from the seated ring center.
    trim=error-.045
    height=(-.25-offset-140*np.sin(np.deg2rad(trim)))/math.cos(pitch)
    print(case,round(row['t'],3),'pitchErrorDeg',round(error,3),'normalOffsetMm',round(offset,3),'trimDeg',round(trim,3),'heightMm',round(height,3))
