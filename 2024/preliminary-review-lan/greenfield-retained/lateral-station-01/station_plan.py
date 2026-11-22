"""Change only the launch endpoint on the existing route and verify the affected occupied paths."""
from pathlib import Path
import json,copy,hashlib
p=Path(__file__).resolve().parent;root=p.parent
previous=json.loads((root/'split-field-02/plan.json').read_bytes());updated=copy.deepcopy(previous)
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
originalRunner=(p/'route.py').read_text();exec(originalRunner[:originalRunner.index('exec(compile(source,')])
exec(compile(source[:source.index("if __name__=='__main__':")],'unchanged shared layout and field search','exec'))
checks=[];idle=order.copy()
for i,leg in enumerate(updated['plan']):
    k=leg['id'];idle.remove(k);q=leg['yaw']
    if i:
        leg['out'][0]=station.tolist()
        pair=np.array(leg['out'][:2]);leg['outT'][0]=clock(np.linalg.norm(pair[1]-pair[0]))
        coarse=Grid(idle+[k],q,low=True)
        checks.append([k,'out',coarse.clear(*pair)])
    leg['loaded'][-1]=station.tolist();pair=np.array(leg['loaded'][-2:])
    leg['loadedT'][-1]=clock(np.linalg.norm(pair[1]-pair[0]))
    coarse=Grid(idle,0);checks.append([k,'return',coarse.clear(*pair)])
updated.update(placedStep=design['step'],parent=sha(root/'split-field-02/plan.json'),envelope=[lo.tolist(),hi.tolist()],stationM=station.tolist(),angleDeg=122,changedChecks=checks,unchangedFirst=sha(root/'guided-hoist-01/inboard-first-dev.json'))
(p/'station-checks.json').write_text(json.dumps(updated,indent=2))
assert all(r[-1] for r in checks),checks
(p/'plan.json').write_text(json.dumps(updated,indent=2));print(updated['placedStep'],checks)

