"""Bind the pickup comparison to the existing private review and update its evidence text."""
from pathlib import Path
import json,hashlib
p=Path(__file__).resolve().parent
lip=p.parent/'pickup-lip'
live=p.parent.parent/'www/data.json'
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def read(path):return json.loads(path.read_bytes())
d=read(live)
assert d['step']==sha(lip/'shared-cassette-lip.step')
files=sorted(p.glob('*.json'))
rows=[]
for f in files:
    t=read(f)
    if 'trajectory' in t:
        rows.append({**{k:v for k,v in t.items() if k not in ('trajectory',)},'source':f.name,'sha256':sha(f)})
observed={r['label']:r for r in rows}
assert all(observed[k]['seated']==v for k,v in [('isolated-bare45',False),('isolated-lip45',True),('isolated-bare75',True),('isolated-lip75',True),('drum',False),('center',True),('off',False)])
record=p/'comparison.json'
record.write_text(json.dumps(dict(step=d['step'],mode='MuJoCo free rigid NOTE and passive spring flap; imposed chassis and belt speeds',sources={f.name:sha(f) for f in [lip/'pickup_trial.py',p.parent/'pickup_physics.py',p/'run.py',p/'approach.py',lip/'layout.json']},trials=rows),indent=2))
c=next(c for c in d['cases'] if c['key']=='intake')
text='The NOTE and passive flap respond to simulated contact and gravity. Chassis approach and belt speeds are prescribed. The viewer replays the free NOTE pose. Ring deformation, measured tread friction, motor speed sag and carpet pile remain unverified.'
c['basis']=text+' This centered replay uses the short pickup lip, a 360 mm approach at 0.25 m/s and a 0.25 ms solver step.'
for r in c['results']:
    if r['pair']==['Replaceable pickup lip','Nose pickup roller']:
        r['witness']['reason']='Matched centered pickup at assumed nose friction 0.75 seated with and without the lip. At 0.45 the bare nose lifted the NOTE 9.6 mm and stalled. With the lip the NOTE rose 346 mm and seated. The carpet and side-belt parameters were identical.'
d['bodies']['Nose pickup roller']['purpose']='Drive the wrapped center belt and present rolling surfaces on either side for an offset NOTE.'
d['bodies']['Center intake belt']['purpose']='Catch the centered NOTE on the moving wrap around the nose and draw it toward the side belts.'
d['bodies']['Replaceable pickup lip']['rationale']='The short lip helped the NOTE reach the moving belt in the lower-grip simulation. Its 3 mm carpet clearance still needs a physical test.'
d['receipt']=sha(record)
out=p/'review-evidence-data.json';out.write_text(json.dumps(d,separators=(',',':')),encoding='utf8')
previous=read(live)
assert previous['step']==d['step'] and next(c for c in previous['cases'] if c['key']=='intake')['series']==c['series']
temp=live.with_suffix('.next');temp.write_bytes(out.read_bytes());temp.replace(live)
print(len(rows),'trials; registered review evidence',d['receipt'])

