import sys,json,math
from pathlib import Path
sys.path.insert(0,str(Path.cwd()/'sim/pressure'))
from wing_field_frame import WingFrame
frame=WingFrame('middle',245.)
for case in sys.argv[1:]:
    fold=Path(__file__).parent/case
    next_time=0.;print(case)
    for line in (fold/'trace.jsonl').open('rb'):
        if not line.endswith(b'\n'):break
        r=json.loads(line);t=r['t']
        if t<next_time:continue
        next_time=t+1.989
        q=r['q'];xy=frame.to_field([*q[:2],0]);print(round(t,2),*map(lambda x:round(x,2),xy[:2]),round((math.degrees(q[2])+425)%360-180))
