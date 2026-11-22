"""Reassess the measured field transport on the unchanged six-piece missions."""
from pathlib import Path
import json, hashlib, subprocess, sys
from concurrent.futures import ThreadPoolExecutor
p=Path(__file__).resolve().parent
base=p.parent/'split-field-02'
def read(path): return json.loads(path.read_bytes())
def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()
def save(path, data):
    tmp=path.with_suffix('.tmp')
    tmp.write_text(json.dumps(data,indent=2)); tmp.replace(path)
def verify(field, prior):
    a,b=read(field),read(prior)
    keys=['sourceCarry','robotX','targetSpeed','initialState','initialVelocity','rotorsAtFeed','release','exit','opening']
    for k in keys:
        if a[k]!=b[k]: raise ValueError((field.name,k))
    return {k:a[k] for k in ['both','beams','opening','passShot','minBusV']}
def mission(case):
    name,n=case;old=read(base/f'attempt-{name}-{n}.json')
    legs=[];old_time=0;clock=0
    for row in old['parts']:
        tag=f"{name}-{row['id']}"; key=f'{tag}-{n}';setting='6.88'
        before=base/f'split-ready-{key}-{setting}.json'
        after=p/f'split-paired_.75_.75-{key}-{setting}.json'
        if not before.exists(): break
        if not after.exists():
            with after.with_suffix('.log').open('w') as log:
                subprocess.run([sys.executable,'fired.py',tag,str(n),setting,'.75','.75'],cwd=p,stdout=log,stderr=log,check=True)
        s=verify(after,before)
        clock+=row['endCarryClock']-old_time
        clock+=s['both'] if s['passShot'] else read(after)['trace'][-1]['t']
        old_time=row['clock']
        legs.append(dict(piece=row['id'],clock=clock,oldBoth=read(before)['both'],**s,baseline=sha(before),fieldRun=sha(after)))
        save(p/f'cycle-{name}-{n}.json',dict(case=name,n=n,legs=legs,score=sum(x['passShot'] for x in legs),passMission=len(legs)==6 and all(x['passShot'] for x in legs) and clock<=120))
        if not s['passShot'] or clock>120:break
    return name,n,len(legs),sum(x['passShot'] for x in legs),clock
if __name__=='__main__':
    cases=[('dev',48),('dev',96),('R01',96)]
    config=dict(cases=cases,duty=[.75,.75],robotMps=6.88,setupSource=sha(base/'freeze.json'),geometry=sha(p/'transmission.json'),surfaceCode=sha(p/'counter.py'),exitMatch='source, robot, initial position and velocity, rotors, exit, release and full opening match legacy before counting',deadlineS=120)
    freeze=p/'cycle-freeze.json'
    if freeze.exists():assert read(freeze)==json.loads(json.dumps(config))
    else:save(freeze,config)
    with ThreadPoolExecutor(3) as pool:
        for result in pool.map(mission,cases):print(result,flush=True)

