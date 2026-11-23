import json,sys
from pathlib import Path
import numpy as np
base=Path(__file__).parent
for old,body in [('middle245-passive-collection','0'),('outer-aligned-collection-to-66','1')]:
    def paths(fold):
        points={}
        for line in (base/fold/'trace.jsonl').open('rb'):
            if not line.endswith(b'\n'):break
            row=json.loads(line)
            if row['t']>42.61:break
            if row['t']>=40.38739:points[round(row['t'],4)]=row
        return points
    a=paths(old);b=paths('middle-outer-coupled-to-66/'+body)
    keys=set(a)&set(b); assert len(keys)>22000
    xyz=max(max(abs(np.array(a[t]['q'][:3])-b[t]['q'][:3])) for t in keys)
    speed=max(max(abs(np.array(a[t]['v'])-b[t]['v'])) for t in keys)
    print(old,len(keys),xyz,speed)
