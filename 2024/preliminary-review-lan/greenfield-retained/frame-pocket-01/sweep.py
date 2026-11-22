"""Check the new split package and pocketed chassis on the same sampled operating and service poses."""
from pathlib import Path
p=Path(__file__).resolve().parent;root=p.parent
src=(root/'assembly-recheck-01/sweep.py').read_text()
a=src.index('src=root/');b=src.index('names=[n for n',a)
src=src[:a]+"""src=p/'frame-and-feeder.step';receipt=json.loads((p/'result.json').read_bytes())
assert hashlib.sha256(src.read_bytes()).hexdigest()==receipt['step']
d=json.loads((root/'follower-01/review-data.json').read_bytes());pkg=json.loads((root/'split-cycle-01/package.json').read_bytes())
raw,_=load_solids(src)
def name(n):
    while chr(194)+chr(183) in n:n=n.replace(chr(194)+chr(183),chr(183))
    return n
shapes={name(n):s for n,s in raw.items()}
roles=next(c for c in d['cases'] if c['key']=='pickup-to-shot')['rig'].copy()
for n in pkg['removed']:roles.pop(n,None)
roles.update({n:'arm' for n in pkg['added']});assert set(shapes)-set(roles)<={'Chambered NOTE'}
d['step']=receipt['step'];park=mat(125,185,0,501);ip=np.linalg.inv(park)
"""+src[b:]
exec(compile(src,'revised package sweep','exec'))
