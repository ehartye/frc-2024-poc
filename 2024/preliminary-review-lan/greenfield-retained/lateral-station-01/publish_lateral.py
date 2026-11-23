"""Present the measured lateral-station nominal mission alongside its actuation limit."""
from pathlib import Path
import copy,json,hashlib,numpy as np,math
p=Path(__file__).resolve().parent;root=p.parent
src=(root/'six-piece-02/publish.py').read_text(encoding='utf8')
exec(compile(src.split("plan=read(p/'plan.json')")[0],'shared presentation coordinate frames','exec'))
www=root.parent/'www/data.json';d=read(www);g=d['concepts']['guided'];f=d['concepts']['field'];plan=read(p/'plan.json')
assert plan['placedStep']==g['step']
receipt=read(p/'mission-receipt-dev-48.json');assert receipt['terminal'] and len(receipt['parts'])==6 and receipt['passMission']
c=copy.deepcopy(next(c for c in f['cases'] if c['key']=='field-48'))
c.update(key='lateral-nominal-48',id='Six NOTES · lateral shooting station',time=[],positions=[],measure=[],series={k:[] for k in ['frame','piece','arm','slide','flap']},results=[],frames={},stage='Six-piece scoring trial',status='sampled',basis='One nominal run at 0.5 ms. The free NOTE and both field counter signals are simulated. The actual pickup and release states carry through each return. The mechanism follows the prescribed 3.4 s lift. Its slide command exceeds the provisional actuation envelope, so this view supports scoring geometry only.')
c['rig']={**dict.fromkeys(g['add'],'arm'),**c['rig']}
functions=src[src.index('def append('):src.index('clock=0;count=0;')]
exec(compile(functions,'timing, transforms and field names','exec'))
loop=src[src.index('clock=0;count=0;'):src.index('res=dict(planHash')]
loop=loop.replace("tag='dev-'+name+'-96'","tag='dev-'+name+'-48'").replace("'indexed-'+tag+'.json'","'split-outerlane-'+tag+'-6.88.json'").replace("s=read(p/('split-outerlane-'+tag+'-6.88.json'))","s=read(p/('split-outerlane-'+tag+'-6.88.json'));legReceipt=receipt['parts'][idx];assert sha(p/(tag+'.json'))==legReceipt['carry'] and s['sourceCarry']==legReceipt['carry'] and s['pivot']==[1.575,5.871,.501] and sha(p/('split-outerlane-'+tag+'-6.88.json'))==legReceipt['shot']")
loop=loop.replace('a=125+(deployed-125)','a=122+(deployed-122)').replace('a=deployed+(125-deployed)','a=deployed+(122-deployed)').replace('scoreXY,125,P,185','scoreXY,122,P,185')
loop=loop.replace('if t>2.82:',"if t>2.82+r['seatDelay']:").replace("leg['loadedT'],t-2.82)","leg['loadedT'],t-2.82-r['seatDelay'])")
loop=loop.replace('clock+=stop;count+=int(s[\'passShot\'])',"clock+=stop;count+=int(s['passShot']);assert abs(clock-legReceipt['clock'])<1e-5")
exec(compile(loop,'measured six legs and counters','exec'))
assert count==6 and abs(c['time'][-1]-clock)<1e-4
c['results'].append(dict(pair=['Loaded slide command','Available actuation'],status='fail',witness=dict(reason='The piece solver imposes cassette motion. At the 185 mm slide limit, the velocity changes abruptly. The original motion and its score remain evidence for this case. A bounded 4.8 s motor and clearance trial is separate.')))
g['cases']=[a for a in g['cases'] if a['key']!=c['key']]+[c]
g['defaultCase']=c['key']
g['status']='Preliminary design held · scoring repeatability and loaded actuation'
g['focus']='The lateral shooting station scored six in the nominal coarse model. Open the lateral shooting trial for each floor pickup and field count. Loaded motor demand and independent starts are under review.'
title='Loaded lift and drive · 4.8 s candidate'
rr=read(root/'loaded-actuation-01/purchased-power.json')
group=[r for r in rr['cases'] if r['liftS']==4.8 and r['auxA']==25];minv=min(r['weakBusMinV'] for r in group)
case=dict(case=title,stage='Loaded return',batteryCondition='weak',openCircuitV=11.8,resistance=.027,requiredMinV=8,loadedBusV=[minv,11.125],batterySideA=[25,(11.8-minv)/.027],marginV=minv-8,status='provisional',lines=[dict(name='Accessory allowance',amps=[25,25],members=[]),dict(name='Four traction motors, pivot and slide combined',amps=[0,(11.8-minv)/.027-25],members=['Pivot motor','Slide drive motor'])],basis='Conservative upper mass and 2 kg of reserve at the cassette. NEO 550 datasheet and assumed 8 mm screw lead. Signed negative pivot-current allowance requires a separate loaded-mode setting. Physical motor, screw and feedback qualification open.')
g['power']=[x for x in g['power'] if x['case']!=title]+[case]
tmp=www.with_suffix('.lateral-next');tmp.write_text(json.dumps(d,separators=(',',':')),encoding='utf8');tmp.replace(www)
(p/'nominal-view.json').write_text(json.dumps(dict(case=c['key'],source=sha(p/'mission-receipt-dev-48.json'),step=g['step'],legs=report,timeS=clock,frames=len(c['time']),score=count,power=sha(root/'loaded-actuation-01/purchased-power.json')),indent=2))
print(c['key'],len(c['time']),clock,count)

