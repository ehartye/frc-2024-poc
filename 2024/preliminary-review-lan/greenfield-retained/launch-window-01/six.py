"""Apply the one selected setting to every piece on both nominal clocks and the first retained failed route."""
from pathlib import Path
import json,hashlib,shutil,subprocess,sys,concurrent.futures
p=Path(__file__).resolve().parent;base=p.parent/'split-field-02'
read=lambda f:json.loads(f.read_bytes())
def sha(f):return hashlib.sha256(f.read_bytes()).hexdigest()
setting=6.74
records=[('dev',96),('dev',48),('R01',96)]
config=dict(rows=records,settingMps=setting,windowInput=sha(p/'results.json'),missionInput=sha(base/'freeze.json'),why='Both candidate commands recovered the known miss. Select the lower-speed setting for its larger opening margin on that miss and shorter nominal counter dwell. Apply it to all six pieces.',criterion='Six unique scored pieces by 120 seconds on both nominal discretizations and R01. Stop at first failed leg within each trial; retain each failure. This is a development replay.')
f=p/'six-freeze.json'
if not f.exists():f.write_text(json.dumps(config,indent=2))
else:assert read(f)==json.loads(json.dumps(config))
def replay(case):
    start,n=case;prevOld=0.;now=0.;rows=[]
    for r in read(base/f'attempt-{start}-{n}.json')['parts']:
        leg=r['id'];key=f'{start}-{leg}-{n}';carry=p/(key+'.json')
        if not carry.exists():shutil.copyfile(base/carry.name,carry)
        assert sha(carry)==r['carry']
        field=p/f'split-ready-{key}-{setting}.json'
        if not field.exists():
            with field.with_suffix('.log').open('w') as out:subprocess.run([sys.executable,'shot.py',f'{start}-{leg}',str(n),'ready',str(setting)],cwd=p,stdout=out,stderr=out,check=True)
        s=read(field);assert s['sourceCarry']==r['carry'] and s['targetSpeed']==setting
        now+=r['endCarryClock']-prevOld;now+=s['both'] if s['passShot'] else s['trace'][-1]['t'];prevOld=r['clock']
        rows.append(dict(id=leg,passShot=s['passShot'],opening=s['opening'],both=s['both'],arrival=now,fieldSha=sha(field),minBusV=s['minBusV']))
        if not s['passShot'] or now>120:break
    verdict=dict(start=start,n=n,rows=rows,score=sum(a['passShot'] for a in rows),passMission=len(rows)==6 and all(a['passShot'] for a in rows) and now<=120)
    (p/f'six-{start}-{n}.json').write_text(json.dumps(verdict,indent=2));return verdict
with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:out=list(pool.map(replay,records))
print([(r['start'],r['n'],r['score'],r['passMission']) for r in out])
