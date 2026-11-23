"""Collect measured outcomes and add replayable experiment cases to the current review."""
from pathlib import Path
import hashlib,json,copy,math,numpy as np
from scipy.spatial.transform import Rotation
p=Path(__file__).resolve().parent;root=p.parent;www=root.parent/'www'
def read(f):return json.loads(f.read_bytes())
def sha(f):return hashlib.sha256(f.read_bytes()).hexdigest()
def data(name):return read(p/(name+'.json'))
def mat(angle,x=0,y=0,z=0):
    v=np.eye(4);v[:3,:3]=Rotation.from_euler('y',-angle,degrees=True).as_matrix();v[:3,3]=[x,y,z];return v
def world(q,xyz):
    v=np.eye(4);v[:3,:3]=Rotation.from_quat([*q[1:],q[0]]).as_matrix();v[:3,3]=xyz;return v
def flat(v):return v[:3].round(9).flatten().tolist()
d=read(root/'swap-blade/review-data.json');live=www/'data.json'
assert d['step']==read(live)['step']==sha(root/'swap-blade/shared-cassette-swappable.step')
assert d['defaultPickupBlade']['default']=='A'
baseline={c['key']:c for c in d['cases']}
catalog={}
for group,names in dict(pickup='p-original p-baseline p-default p-low p-high p-fine p-weak p-off',climb='b-center b-forward b-fine b-rear b-low b-delay b-none',shot='s-armed s-fine s-bypass s-stuck').items():
    for nm in names.split():
        f=p/(nm+'.json');r=read(f);row={k:v for k,v in r.items() if k not in ('trajectory','motorTrace','trace','held')}
        if group=='pickup' and row.get('minimumContactSpeedMps',0)==10: row['minimumContactSpeedMps']=None
        catalog[nm]=dict(group=group,file=f.name,sha256=sha(f),result=row)
sourcefiles=[p/k for k in ('PLAN.md','pickup.py','brake.py','power.py','report.py')]+[root/v for v in ('pickup_physics.py','pickup-lip/pickup_trial.py','pickup-lip/shot_replay.py','pickup-lip/gap.py','pickup-lip/layout.json','mission-physics/climb.py','mission-physics/shot.py','swap-blade/blade-selection.json','swap-blade/review-data.json')]
receipt=p/'results.json'
receipt.write_text(json.dumps(dict(assembly=d['step'],profile='A',sources={str(k.relative_to(root)):sha(k) for k in sourcefiles},cases=catalog),indent=2))
park=mat(125,185,0,501);hinge=mat(0,457.5,0,96);seat=park@mat(0,215,0,50.4);I=np.eye(4)
def trial(parent,key,title,basis):
    c=copy.deepcopy(baseline[parent]);c.update(key=key,fixture=parent,id=title,basis=basis,stage='Bounded experiment',status='review',time=[],measure=[],positions=[],results=[],series={g:[] for g in c['series']},frames={},narrow=0,gap='scoped')
    if 'chain' in c:c['chain']=[]
    if 'beltSpeed' in c:c['beltSpeed']=[]
    return c
def frame(c,t,xy,text,poses):
    c['time'].append(t);c['positions'].append(xy);c['measure'].append(text)
    for k in c['series']:c['series'][k].append(flat(poses.get(k,I)))
def evidence(c,pair,status,reason):
    c['results'].append(dict(pair=pair,status=status,witness=dict(reason=reason)))
nose='Replaceable pickup lip';note='Chambered NOTE'
for code,key,title in [('p-default','trial-pickup','Trial · pickup stop raised 0.75°'),('p-fine','trial-pickup-miss','Trial · raised nose misses the NOTE')]:
    r=data(code);c=trial('intake',key,title,'Profile A. Free NOTE, finite-inertia nose with solved contact load, 8 A motor-current screen. Side belts retain prescribed speed and capped traction. Chassis motion is prescribed. No physical carpet or motor calibration.')
    xyz=read(root/'pickup-lip/layout.json')['pickPivot'].copy();xyz[2]+=r['addedHeightMm'];pose=mat(r['pitchDegrees'],*xyz);arm=pose@np.linalg.inv(park)
    for row,motor in zip(r['trajectory'],r['motorTrace'],strict=True):
        t=row[0];adv=1000*min(.25*t,.44);deg=math.degrees(row[-1]);frame(c,t,[965.2+adv,5547.36],f"Nose {motor[1]:.2f} m/s · {motor[3]:.1f} A · tip height {r['minimumBladeHeightMm']:.2f} mm · {row[-2] or 'no recorded contact'}",dict(arm=arm,slide=mat(0,xyz[0]-185,0,xyz[2]-501),flap=pose@hinge@mat(deg)@np.linalg.inv(hinge)@np.linalg.inv(park),piece=world(row[4:8],np.array(row[1:4])*1000)@np.linalg.inv(seat)))
    # Height perturbation is an intake-datum sensitivity, not a new driven Z joint.
    evidence(c,[nose,note],'sampled' if r['seated'] else 'fail',f"Seat residence {r['seatResidenceS']:.3f} s; first contacts {r['firstContactS']}. Relative intake height offset {r['addedHeightMm']:.1f} mm.")
    d['cases'].append(c)
r=data('b-fine');c=trial('climb','trial-brake','Trial · power-loss brake slips','Two engaged ideal hook captures. 180 N·m modeled pivot brake starts 30 ms after loss of motor torque, then ramps over 30 ms. Chassis CG is 40 mm forward. Ground reaction comes from tires and bumper. Hardware response and rating require measurement.')
xy=baseline['climb']['positions'][0];r0=baseline['climb'];groups={}
from scipy.spatial import ConvexHull
for k in ['frame','arm','flap']:
    pts=np.vstack([v['v'] for n,v in d['bodies'].items() if n not in c['removed'] and c['rig'].get(n,'frame')==k]);groups[k]=pts[ConvexHull(pts).vertices]
minZ=1000.
for row in r['trace']:
    t=row['t'];a=row['angle'];base=world(row['q'],1000*np.array(row['p']));rot=base@mat(a,185,0,501);cam=90*np.clip((a-102)/18,0,1)
    poses=dict(frame=base,arm=rot@np.linalg.inv(park),flap=rot@hinge@mat(cam)@np.linalg.inv(hinge)@np.linalg.inv(park))
    z=min((pts@poses[k][2,:3]+poses[k][2,3]).min() for k,pts in groups.items())
    if t>=4:minZ=min(minZ,z)
    frame(c,t,xy,f"{'Motor disabled' if t>=4 else 'Hoist'} · pivot {a:.1f}° · {row['floorN']:.0f} N floor · {z:.0f} mm lowest stock",poses);c['chain'].append(np.round(np.array(row['chain'])*1000,3).tolist())
c['floorCheckMm']=minZ;evidence(c,['Chassis','Carpet'],'fail',f"Motor torque zero for 5 s. Brake reached {r['peakDisabledBrakeNm']:.0f} N·m; pivot slipped {r['driftDeg']:.1f}°. Ground support returned.");d['cases'].append(c)
for code,key,title in [('s-armed','trial-shot','Trial · weak-pack shot interlock'),('s-bypass','trial-shot-miss','Trial · weak-pack interlock bypassed')]:
    r=data(code);c=trial('shot',key,title,'Same held aim, free NOTE and registered field detectors. An electrical acceleration demand is applied at fixed field pose. Permission needs 100 ms at forecast ≥8.2 V and the acceleration request released; it suppresses renewed acceleration through release. Bypass omits this arbitration.')
    xy=baseline['shot']['positions'][0]
    # The simulation begins chambered with the flap cam fully open at the target angle.
    for row in r['trace']:
        t=row['t'];q=row['q'];xyz=np.array(q[:3])*1000;xyz[:2]-=xy;v=sum(row['beltMps'])/2
        stage='Waiting for power' if t<r['enableTime'] else 'Both detectors registered' if r['both'] and t>=r['both'] else 'Free flight' if r['exit'] and t>=r['exit']['t'] else 'Spinning up'
        frame(c,t,xy,f"{stage} · {row['busV']:.2f} V · {v:.2f} m/s belt",dict(flap=park@hinge@mat(90)@np.linalg.inv(hinge)@np.linalg.inv(park),piece=world(q[3:],xyz)@np.linalg.inv(seat)));c['beltSpeed'].append(v)
    evidence(c,[note,'Blue speaker'],'sampled' if r['passShot'] else 'fail',f"Minimum {r['minBusV']:.2f} V. Both detectors: {r['both'] if r['both'] else 'miss'}. Drive/belt overlap timesteps: {r['poweredOverlapSteps']}.");d['cases'].append(c)
    # The highest pack draw occurs during spin-up before release.
    other=25 if code=='s-armed' else 118
    d['power'].append(dict(case=title+' / weak',stage=title,batteryCondition='weak',openCircuitV=11.8,resistance=.027,requiredMinV=8,loadedBusV=[r['minBusV'],11.8],batterySideA=[0,r['peakPackA']],marginV=r['minBusV']-8,status='pass' if r['minBusV']>=8 else 'fail',lines=[dict(name='Other loads at peak',amps=[other,other],members=[]),dict(name='Scoring belt motors at peak',amps=[0,r['peakPackA']-other],members=[n for n in d['bodies'] if n.startswith('Belt motor ·')])]))
d['notes'].append('Bounded risk experiment 01. The staged shot scored on both numerical refinements. The raised stop traded reduced digging for a missed high-side pickup. The finite friction brake slipped at all three CG positions. The five Trial replays use the same assembled robot.')
d['boundedRiskReceipt']=sha(receipt);d['focus']='Profile A remains the default. Trial replays show pickup sensitivity, brake slip and weak-pack shot sequencing.'
for c in d['cases']:
    assert len(c['measure'])==len(c['time']) if 'time' in c else True
path=p/'review-data.json';path.write_text(json.dumps(d,separators=(',',':')),encoding='utf8')
if __name__=='__main__' and '--live' in __import__('sys').argv:
    current=read(live);assert current.get('boundedRiskReceipt') or current['receipt']==d['receipt'] and len(current['cases'])==len(baseline)
    temp=live.with_suffix('.next');temp.write_bytes(path.read_bytes());temp.replace(live)
print('replays',len(d['cases']),'full-stock climb minimum mm',minZ, 'receipt',sha(receipt))

