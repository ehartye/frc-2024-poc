import hashlib,json,sys
from pathlib import Path
root=Path(__file__).resolve().parent
sys.path.insert(0,str(Path(sys.argv[1])/'scripts'))
from mechanical_motion import evaluate
plan=json.loads((root/'supported-paths.json').read_bytes())
from copy import deepcopy
control=deepcopy(next(c for c in plan['cases'] if c['id']=='over-front-cover'))
control['id']='displaced-front-cover'
control.setdefault('initialPose',{}).setdefault('Complete blue bumper envelope',[]).append(dict(kind='translation',travelMm=[2,0,0]))
plan['cases'].append(control)
# The same stationary interfaces recur across cases. Memoize only identical
# immutable shapes, orientations and exact placements within this invocation.
import mechanical_motion as motion
from functools import lru_cache
for attr in ('common','distance','bounds'):
    setattr(motion,attr,lru_cache(maxsize=60000)(getattr(motion,attr)))
receipt=evaluate(root,plan)
assert receipt['stepSha256']==plan['step']['sha256']
(root/'supported-checked.json').write_text(json.dumps(receipt,separators=(',',':')))
print([(c['case'],c['status']) for c in receipt['cases']],flush=True)

