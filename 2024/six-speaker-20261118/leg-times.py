import json, sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path.cwd()/'sim/pressure'))
from wing_field_frame import WingFrame
base=Path(__file__).parent
for case in sys.argv[1:]:
    folder=base/case;r=json.loads((folder/'result.json').read_bytes())
    frame=WingFrame(r['firstMark'],r['fieldYawDeg'])
    t=r['start']
    print(case)
    for line in (folder/'trace.jsonl').open('rb'):
        row=json.loads(line)
        if row['t']<t:continue
        t+=1
        q=row['q'];xy=frame.to_field([*q[:2],0])[:2]; heading=(np.rad2deg(q[2])+frame.yaw_degrees+180)%360-180
        print(f"{row['t']:.2f} {xy[0]:.2f} {xy[1]:.2f} {heading:.0f}")
