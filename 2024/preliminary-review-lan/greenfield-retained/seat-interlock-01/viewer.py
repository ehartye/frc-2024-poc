"""Display the same third floor piece on the frozen and beam-confirmed controllers."""
from pathlib import Path
import json,csv
p=Path(__file__).resolve().parent;root=p.parent
sf=root/'split-field-02'
web=root.parent/'www/data.json'
renderer=root/'tread-underlay-01/scene.py'
src=renderer.read_text(encoding='utf8');options=src[src.index('template='):src.index('cases=[]')];env=dict(root=root)
exec(options,env);base=env['template'];read=lambda path:json.loads(path.read_bytes())
doc=read(web);layout=doc['concepts']['field'];example=next(c for c in layout['cases'] if c['key']=='field-r01')
observed=next(x for x in csv.DictReader(Path('R:/sim/pressure/evidence/speaker-repeatability-starts.csv').open()) if x['attempt']=='R08')
expected=read(sf/'attempt-R08-96.json')
for recover in (False,True):
    patch=base.replace("r=read(p/(tag+'.json'));s=read(p/('split-ready-'+tag+'-6.88.json'))","folder=local if name=='inner' and successor else p;r=read(folder/(tag+'.json'));shot=folder/('split-ready-'+tag+'-6.88.json');s=read(shot) if shot.exists() else None")
    patch=patch.replace("if t>2.82:","if t>2.82+r.get('seatDelay',0):").replace("moment(loaded,leg['loadedT'],t-2.82)","moment(loaded,leg['loadedT'],t-2.82-r.get('seatDelay',0))")
    patch=patch.replace("clock+=r['finish'];shotStart=clock;",'''clock+=r['finish']
    if not s:
        c['measure'][-1]=f'{prefix} · failed to seat · lift started on timer · {count}/6 scored'
        c['results'].append(dict(pair=[prefix,'Rear seat'],status='fail',witness=dict(reason='Lift started while the third NOTE remained short of the stop. No launch authorized.')))
        break
    shotStart=clock;''')
    patch=patch.replace('for f in [p/(tag+\'.json\'),p/(\'split-ready-\'+tag+\'-6.88.json\')]','for f in [folder/(tag+\'.json\'),shot]')
    inputs=dict(__file__=str(sf/'publish.py'),local=p,attempt='R08',nn=96,expected=expected,obs=observed,successor=recover)
    exec(compile(patch,'matched R08 replay','exec'),inputs)
    case=inputs['c'];case['rig']=example['rig'].copy()
    case.update(key='field-seat-recovery' if recover else 'field-r08',id='Seat-confirmed pickup · R08 third NOTE' if recover else 'Repeatability · R08 missed pickup',basis='Same frozen start. Two common scores precede the third pickup. '+('A modeled rear beam waits for 40 ms of continuous detection, delaying the lift 113 ms. The field replay inherits the actual incoming ring and rotor state. The trial ends at this third score.' if recover else 'The clocked lift begins before the third NOTE seats. Replay ends with its uncredited carry.'))
    if not recover:assert inputs['count']==2 and abs(inputs['clock']-expected['elapsed'])<.001
    else:assert inputs['count']==3
    layout['cases']=[c for c in layout['cases'] if c['key']!=case['key']]+[case]
    print(case['key'],inputs['clock'],case['measure'][-1])
layout['validationText']='The original inward climb intersects the official fixed stage rail. PARK clears the tested approach. Six of eight frozen scoring starts passed; two failures ended the 19-of-20 campaign. Seat recovery is a separate bounded trial.'
tmp=web.with_suffix('.seat-next');tmp.write_text(json.dumps(doc,separators=(',',':')),encoding='utf8');tmp.replace(web)

