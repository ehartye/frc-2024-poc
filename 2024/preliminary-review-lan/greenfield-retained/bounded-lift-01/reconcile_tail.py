"""Charge the same autonomous actions on the two observed collection-floor continuations."""
from pathlib import Path
import json, hashlib
p=Path(__file__).resolve().parent
def read(f):return json.loads(f.read_bytes())
def digest(f):return hashlib.sha256(f.read_bytes()).hexdigest()
results=[]
for n in (48,96):
    src=p/f'run-dev-{n}.json'
    for mu,z in [('.25','0'),('.75','.05')]:
        dep=0.;rows=[]
        for i,q in enumerate(read(src)['parts']):
            shot=dep+(q['shotStart']-q['depart'])
            end=shot+q['both']
            a=p/f'tail-receipt_{mu}_{z}-dev-{q["id"]}-{n}.json'
            overlap=q['resetParallelEligible']
            if a.exists():
                r=read(a);assert r['original']==q['shot'] and all(r['same'].values())
                overlap=r['passScreen']
            old=p/f'split-outerlane-dev-{q["id"]}-{n}-6.88.json';assert digest(old)==q['shot']
            nxt=max(end,shot+read(old)['feedTime']+1+4.8) if overlap else end+4.8
            rows.append(dict(id=q['id'],elapsed=end,overlap=overlap,nextDeparture=nxt,receipt=digest(a) if a.exists() else None))
            dep=nxt
        results.append(dict(resolution=n,friction=mu,height=z,source=digest(src),parts=rows,count=6,elapsed=end,within120=end<=120))
(p/'tail-timing.json').write_text(json.dumps(results,indent=2))
for x in results:print(x['resolution'],x['friction'],round(x['elapsed'],3),x['within120'])

