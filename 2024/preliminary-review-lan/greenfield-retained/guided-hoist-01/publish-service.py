"""Expose the selected replacement service index and the measured straight battery withdrawal."""
from pathlib import Path
import json,math,copy,numpy as np,sys
p=Path(__file__).resolve().parent;root=p.parent;web=root.parent/'www/data.json'
read=lambda f:json.loads(f.read_bytes())
doc=read(web);g=doc['concepts']['guided'];field=doc['concepts']['field']
sys.path.insert(0,str(root/'follower-01'));from stroke import scheduled
def M(a=0,x=0,y=0,z=0):
    t=math.radians(a);c,s=math.cos(t),math.sin(t);return np.array([[c,0,-s,x],[0,1,0,y],[s,0,c,z],[0,0,0,1.]])
def flat(m):return m[:3].round(8).ravel().tolist()
pin=read(p/'service-pin.json');g['step']=pin['whole']
new={**doc['bodies']['Service index disk'],**read(p/'service-index-mesh.json')}
new.update(purpose='Index the empty cassette during battery removal.',rationale='The second bore accepts the truck pin at −40°. The 80° service index traps the pack behind the new hoist rail.',basis='Registered 6.35 mm pin and two measured disk bores. Manual slide restraint and loaded lock verification remain open.')
g['add']['Service index disk']=new
for c in g['cases']:
    for r in c['results']:
        if r['pair'][0]=='Battery withdrawal at 80°':r['witness']['reason']='The original 80° pose obstructs extraction. Use the separate −40° service trial to inspect the replacement disk and battery pull.'
case=copy.deepcopy(next(c for c in field['cases'] if c['key']=='field-climb'))
case.update(id='Service · indexed battery withdrawal',key='guided-service',fixture='',stage='battery service',basis='Empty cassette at −40°. Main leads disconnected and omitted. The manual pin inserts 12 mm, overlapping the new disk by 10 mm. The battery then lifts vertically 370 mm. Modeling assumes the translating truck is separately restrained.',frames={},series={},rig=dict(case['rig']),results=[],time=[],positions=[],measure=[])
case.pop('chain',None);case.pop('floorCheckMm',None)
case['removed']=list(set(case['removed'])|{'Pack lead','Distribution lead'})
rig=case['rig']
for n in g['add']:rig[n]='arm'
rig.update({'Battery service pin':'servicePin','Battery and terminal case':'withdrawal','Catch dog':'slide'})
for group in set(rig.values()):case['series'][group]=[]
ip=np.linalg.inv(M(125,185,0,501));truck=M(0,scheduled(-40)-185);arm=M(-40,scheduled(-40),0,501)@ip
for i in range(91):
    t=i/30;pinMm=min(12,t*18);liftMm=370*np.clip((t-1)/2,0,1)
    m=dict(frame=np.eye(4),arm=arm,slide=truck,pin=truck,servicePin=truck@M(0,0,-pinMm,0),withdrawal=M(0,0,0,liftMm),piece=np.eye(4),keeper=arm,flap=arm)
    for name in case['series']:case['series'][name].append(flat(m[name]))
    case['time'].append(t);case['positions'].append([2300,5550]);case['measure'].append(f'Pinned service pose −40° · pin insertion {pinMm:.1f} mm · pack withdrawal {liftMm:.0f} / 370 mm')
case['results']=[dict(pair=['Battery and terminal case','Guided hoist and chassis'],status='pass',witness=dict(reason='Continuous 370 mm extraction envelope clears surrounding stock by at least 5.0 mm, excluding battery support and disconnected cables.')),dict(pair=['Battery service pin','Service index disk'],status='pass',witness=dict(reason='Positive 10 mm pin entry, 0.175 mm radial gap. Four-degree indexing errors on either side produce 83.58 mm³ interference.')),dict(pair=['Translating truck','Frame'],status='open',witness=dict(reason='Provide a positively retained restraint for the slide before hands-on service. Harness, handling and proof load remain open.'))]
g['cases']=[c for c in g['cases'] if c['key']!=case['key']]+[case]
g['validationText']=g['validationText'].replace('Physical splice','A new −40° service index clears the disconnected battery pull by 5 mm. Physical splice')
g['status']='Preliminary design held · six-NOTE repeatability'
tmp=web.with_suffix('.service-next');tmp.write_text(json.dumps(doc,separators=(',',':')),encoding='utf8');tmp.replace(web)
