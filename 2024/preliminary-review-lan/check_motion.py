import hashlib,json,sys
from pathlib import Path
root=Path(__file__).resolve().parent
sys.path.insert(0,str(Path(sys.argv[1])/'scripts'))
from mechanical_motion import evaluate
plan=json.loads((root/'nominal-motion.json').read_bytes())
from copy import deepcopy
control=deepcopy(next(c for c in plan['cases'] if c['id']=='over-front-cover'))
control['id']='displaced-front-cover'
control.setdefault('initialPose',{}).setdefault('Complete blue bumper envelope',[]).append(dict(kind='translation',travelMm=[2,0,0]))
plan['cases'].append(control)
receipt=evaluate(root,plan)
assert receipt['stepSha256']==plan['step']['sha256']
(root/'nominal-checked.json').write_text(json.dumps(receipt,separators=(',',':')))
print([(c['case'],c['status']) for c in receipt['cases']],flush=True)

