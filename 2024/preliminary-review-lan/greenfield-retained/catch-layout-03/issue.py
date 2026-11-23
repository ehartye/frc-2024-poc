"""Register the preliminary catch geometry and replay the controlled proof on its parent layout."""
from pathlib import Path
import json,math,numpy as np,copy,hashlib
from scipy.spatial import ConvexHull
p=Path(__file__).resolve().parent;root=p.parent;dest=root.parent/'www/data.json'
def read(f):return json.loads(f.read_bytes())
def sha(f):return hashlib.sha256(f.read_bytes()).hexdigest()
def mat(a,x=0,y=0,z=0):
    co=math.cos(math.radians(a));si=math.sin(math.radians(a))
    return np.array([[co,0,-si,x],[0,1,0,y],[si,0,co,z],[0,0,0,1]])
def flat(a):return a[:3].round(9).flatten().tolist()
from scipy.spatial.transform import Rotation
d=read(p/'parent-review.json');r=read(p/'layout-results.json');g=read(p/'meshes.json');mass=read(p/'mass-allowance.json')
assert d['step']==r['parent'] and len(d['bodies'])+len(r['attr'])==r['n']==146
for name,mesh in g.items():
    if name not in r['attr']:d['bodies'][name].update(mesh);continue
    a=r['attr'][name]
    why=('Three spaced attachments react the torque at the removable cheek sector.' if a['role']=='arm' else 'The truck moves with the cassette pivot, keeping the catch guide aligned through slide travel.')
    if name=='Catch dog':why='A 10.8 mm axial stroke seats a 12 mm steel pin behind the sector. The pivot is lowered onto the pin before motor power is removed.'
    d['bodies'][name]=dict(**mesh,category='electrical' if 'actuator' in name else 'mechanism',group='Climb catch',material=a['material'],purpose=a['purpose'],rationale=why,basis='Preliminary envelope; purchased actuator, sensor package and strength remain open.')
    d['support'][name]='Cassette cheek to sector, sector to dog, receiver to translating bearing truck, truck to both guide and frame.'
for c in d['cases']:
    for name,a in r['attr'].items():
        ref='Cassette cheek · left' if a['role']=='arm' else 'Translating bearing block · left'
        if 'rig' in c:c['rig'][name]=c['rig'].get(ref,'frame')
        elif ref in c['frames']:c['frames'][name]=c['frames'][ref]
    if c['key'].startswith('trial-catch') or c['key']=='trial-brake':
        c['basis']+=' Archived test of the preceding assembly; the mounted catch is hidden in this historical control.'
        c['removed']+=list(r['attr'])
old=next(c for c in d['cases'] if c['key']=='climb');xy=old['positions'][0];park=mat(125,185,0,501);hinge=mat(0,457.5,0,96)
def lift(n,key,label):
    tr=read(p/(n+'.json'));s=copy.deepcopy(old);s.update(id=label,key=key,fixture='climb',positions=[],time=[],measure=[],chain=[],series={},basis='Model of the installed catch at the upper running mass. The hoist lowers onto the 158° stop under position control and transfers load before motor cutoff. The hooks close ideally; sensor response, purchased actuator and physical stock compliance remain unmeasured.',results=[])
    s['rig']['Catch dog']='pin'
    groups={role:np.vstack([d['bodies'][name]['v'] for name,v in s['rig'].items() if name not in s['removed'] and v==role]) for role in ('frame','arm','flap','pin')}
    groups={k:v[ConvexHull(v).vertices] for k,v in groups.items()}
    zmin=1e5
    for row in tr['trace']:
        t=row['t'];angle=row['angle'];q=row['q'];m=np.eye(4);m[:3,:3]=Rotation.from_quat([*q[1:],q[0]]).as_matrix();m[:3,3]=np.array(row['p'])*1000
        arm=m@mat(angle,185,0,501);poses=dict(frame=m,arm=arm@np.linalg.inv(park),pin=m@mat(0,0,-row['strokeMm'],0),flap=arm@hinge@mat(90*np.clip((angle-102)/18,0,1))@np.linalg.inv(hinge)@np.linalg.inv(park),piece=np.eye(4))
        for k,v in poses.items():s['series'].setdefault(k,[]).append(flat(v))
        if t>=tr['cutTime']:zmin=min(zmin,min(np.min(v@poses[k][2,:3]+poses[k][2,3]) for k,v in groups.items()))
        s['positions'].append(xy);s['time'].append(t);s['chain'].append(np.round(np.array(row['chain'])*1000,3).tolist())
        stage='Disabled' if t>=tr['cutTime'] else 'Load confirmed' if row['confirmed'] else 'Pin inserted' if row['armed'] else 'Hoisting'
        s['measure'].append(f"{stage} · stroke {row['strokeMm']:.1f} mm · catch {row['catchNm']:.0f} N·m · motor {row['torqueNm']:.0f} N·m · floor {row['floorN']:.0f} N")
    s['floorCheckMm']=zmin;s['results']=[dict(pair=['Catch dog','Catch sector'],status='sampled' if tr['passed'] else 'fail',witness=dict(reason=f"Peak reaction {tr['peakCatchNm']:.1f} N·m. Motor cutoff {tr['cutTime']:.2f} s. Minimum full-stock height {zmin:.1f} mm at 25 ms snapshots; max floor reaction {tr['maxFloorN']:.1f} N after cutoff."))]
    return s
lnew=lift('controlled-front','climb','Climb · load-confirmed catch');lmiss=lift('controlled-open','catch-held-open','Trial · installed catch held open')
d['cases'][d['cases'].index(old)]=lnew;d['cases'].append(lmiss)
for e in d['power']:
    if e['stage']==old['id']:e['stage']=lnew['id'];e['case']=lnew['id']+' / '+e['batteryCondition']
    e['lines'].append(dict(name='Catch actuator and sensing allowance',amps=[0,2],members=['Catch actuator and spring']))
    e['batterySideA'][1]+=2;e['loadedBusV'][0]-=2*e['resistance'];e['marginV']=e['loadedBusV'][0]-e['requiredMinV'];e['status']='pass' if e['marginV']>=0 else 'fail'
d['mass'].append(dict(id='Climb catch, actuator and sensing allowance',kg=mass['increaseKg'],members=list(r['attr'])))
for k in ('inspection','running'):d[k]=[v+w for v,w in zip(d[k],mass['increaseKg'])]
d['required'].append(['Mounted climb catch',list(r['attr'])]);d['step']=r['step'];d['issue']='Shared cassette · mounted climb catch';d['status']='Preliminary design · integration under review'
d['focus']='Controlled lowering transfers the load to the mounted catch. Pickup height control and the repeated scoring cycle are the next whole-robot checks.'
d['narrativeReview']['objectives']=[dict(objective=t,disposition=v,basis=b) for t,v,b in [
('Acquire and retain a loose floor NOTE','Provisional','Profile A is the default. Fixed-height high-side pickup misses; a floor-referenced intake is under study.'),
('Score six distinct floor NOTES in 120 seconds on 19 of 20 frozen starts','Open','One held-pose weak-pack shot and both detectors verified. The six-piece route on this revision has yet to pass the frozen campaign.'),
('Turn and raise while carrying; recover or reset for the next pickup','Open','The current lift pose review covers geometry. A free-piece loaded return and pose handoff on the campaign clock remain to verify.'),
('Solo ONSTAGE after disable','Conditional','Mounted catch and free chassis hold for 5 s after torque cutoff in four cases. The hooks start aligned to the chain. Approach capture, load sensing and full truss clearance remain open.'),
('PARK fallback at the 24-second decision','Preliminary route','Earlier parked layout rehearsed 10.53 s of travel. Rebind route to the current outer envelope and a usable terminal pose.'),
('Shared chain and overhead receive','Optional, unclaimed','Solo score and solo hang are selected. A partner placement and overhead entry pose would require separate evidence.')]]
d['notes'].append('Catch stock, fasteners and actuator reserve add %.3f–%.3f kg. A 2 A peak allowance is included in the electrical ledger. Rigid dog contact was verified against the modeled normal face; a 1° overrotation enters it. Earlier angular-spring trials are retained as controls.'%tuple(mass['increaseKg']))
d['receipt']=sha(p/'layout-results.json');d['defaultPickupBlade']['assembly']=d['step']
for key in ['held-open','nominal'] :pass
out=p/'review-data.json';out.write_text(json.dumps(d,separators=(',',':')),encoding='utf8')
assert sha(dest)==sha(p/'parent-review.json') or read(dest)['step']==d['step']
tmp=dest.with_suffix('.next');tmp.write_bytes(out.read_bytes());tmp.replace(dest)
print(d['step'],d['inspection'],lnew['floorCheckMm'])

