from pathlib import Path
import json,base64,math,hashlib,shutil
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
p=Path(__file__).resolve().parent;www=p.parent.parent/'www'
def get(s):return json.loads((p/(s+'.json')).read_bytes())
d=get('p3-data');a=d.pop('concepts')['shared'];oldCases=d['cases'];b={k:v for k,v in d['bodies'].items() if k not in a['remove']};b.update(a['add']);extra=get('added-stock');b.update(extra['bodies']);d.update({k:v for k,v in a.items() if k not in ('add','remove')});d['bodies']=b
d['step']=hashlib.sha256((p/'selected-cassette.step').read_bytes()).hexdigest()
d['issue']='Shared cassette · 2024 preliminary design';d['status']='Physics review · engineering holds'
d['focus']='Two mission replays share one layout. Hook capture, the brake and flap cam are idealized. Hardware loads and service remain under review.'
d['fixtureMeshes']=get('display-field');d['fixtureSource']=get('fixture-lineage')
def mat(angle,x=0,y=0,z=0):
    m=np.eye(4);m[:3,:3]=Rotation.from_euler('y',-angle,degrees=True).as_matrix();m[:3,3]=[x,y,z];return m
def world(q,p):
    m=np.eye(4);m[:3,:3]=Rotation.from_quat([*q[1:],q[0]]).as_matrix();m[:3,3]=p;return m
def arr(m):return m[:3].round(8).flatten().tolist()
park=mat(125,185,0,501);seat=park@mat(0,215,0,50.4);hinge=mat(0,399.5,0,84)
armnames=set(a['cases'][2]['frames'])|set(extra['bodies'])
flap='Mouth retention flap';note='Chambered NOTE';armnames.discard(flap)
groups={k:'arm' if k in armnames else 'frame' for k in b};groups[flap]='flap';groups[note]='piece'
def poses(angle,frame,flapOpen):
    arm=frame@mat(angle,185,0,501)
    return dict(frame=arr(frame),arm=arr(arm@np.linalg.inv(park)),flap=arr(arm@hinge@mat(flapOpen)@np.linalg.inv(hinge)@np.linalg.inv(park)))
def cam(angle):return 90*max(0,min(1,(angle-102)/18))
def case(id,slug,xy):return dict(id=id,key=slug,positions=[],time=[],measure=[],rig=groups,series={k:[] for k in ['frame','arm','flap','piece']},basis='',stage='physics playback',gap='scoped',status='review',results=[],frames={},removed=[],narrow=0)
def add(c,t,xy,poses,label):
    c['time'].append(round(t,5));c['positions'].append(xy);c['measure'].append(label)
    for k in c['series']:c['series'][k].append(poses.get(k,arr(np.eye(4))))
def finding(c,parts,status,s):c['results'].append(dict(pair=parts,status=status,witness=dict(reason=s)))
s=get('real1');f=get('production-fine');off=get('production-inhibit');xy=[s['robotX']*1000,s['pivot'][1]*1000];shot=case('Shoot · reverse the cassette belts','shot',xy)
for i in range(40):
    t=i*.025;u=t*t*(3-2*t);ang=102+23*u;ps=poses(ang,np.eye(4),cam(ang));ps['piece']=ps['arm']
    add(shot,t,xy,ps,'Cassette rises · passive cam opens flap %.0f° · belts stopped'%cam(ang))
for row in s['trace']:
    t=row['t']
    if s['both'] is not None and t>s['both']+.35:break
    ps=poses(125,np.eye(4),90);q=row['q'];xyz=np.array(q[:3])*1000;xyz[:2]-=xy;ps['piece']=arr(world(q[3:],xyz)@np.linalg.inv(seat))
    phase='Both detector pairs shadowed' if s['both'] is not None and t>=s['both'] else 'Through the opening' if t>=s['opening']['fullyThroughByS'] else 'Free flight' if t>=s['exit']['t'] else 'Belts accelerate toward the mouth'
    add(shot,1+t,xy,ps,'%s · %.1f m/s belt · %.2f V'%(phase,sum(row['beltMps'])/2,row['busV']))
shot['basis']='Cam rise is kinematic. From belt enable, a free torus responds to compliant tread contact, motor current, gravity and field surfaces. The held cassette starts both belts from rest. Air drag, actual friction and finished inertia remain unmeasured.'
finding(shot,['Chambered NOTE','Blue speaker'],'sampled','Both counters at %.3f s after enable on the primary run. %.1f mm finite-ring opening margin.'%(s['both'],1000*s['opening']['minimumMarginM']))
finding(shot,['Belt motors','NOTE'],'open','Half-step / 96-segment replay: '+('both detectors at %.3f s'%f['both'] if f['both'] else 'no simultaneous paired detection')+'. The inhibited drive has no release. Another firing location at X1.55 m cleared the opening and missed paired detection.')
cl=get('hooked');xy2=[5760.46-cl['chainX']*1000,4104.32];lift=case('Climb · take chain load and disable','climb',xy2);lift['removed']=[note];lift['chain']=[]
hulls={}
for g in ['frame','arm','flap']:
    points=np.vstack([b[name]['v'] for name,role in groups.items() if role==g]);hulls[g]=points[ConvexHull(points).vertices]
def minheight(ps):return min(float((pts@np.array(ps[g])[8:11]+ps[g][11]).min()) for g,pts in hulls.items())
heldmins=[]
for row in cl['trace']:
    t=row['t'];ang=row['angle'];ps=poses(ang,world(row['q'],np.array(row['p'])*1000),cam(ang))
    z=minheight(ps)
    if t>=4.1:heldmins.append(z)
    label='Drive disabled · brake engaged' if row['hold'] else 'Hoisting' if t>.55 else 'Two gates seated' if t>=.25 else 'Align chain in throats'
    add(lift,t,xy2,ps,'%s · %.0f N carpet · %.0f N·m drive · %.0f mm lowest stock'%(label,row['floorN'],row['torqueNm'],z))
    lift['chain'].append(np.round(np.array(row['chain'])*1000,3).tolist())
lift['basis']='Free chassis on unilateral tire and bumper contacts. Twenty-four articulated chain links connect two field anchors. Two ideal gate constraints engage inside a 12 mm capture window. The pivot has a 215 N·m limit. Motor torque goes to zero at 4 s. The brake holds for the remaining 5 s.'
finding(lift,['Chassis','Carpet'],'sampled','Nominal disabled hold: %.1f mm minimum rendered stock clearance, zero carpet load. Peak pivot torque %.1f N·m.'%(min(heldmins),cl['peakNm']))
finding(lift,['Hook keeper','Shared lift crosspiece'],'open','Missing either brake function or hook capture returns load to the floor. At +40 mm chassis CG the modeled brake misses its engagement window. Throat entry, link twist, hardware rating and purchased drive remain open.')
lift['floorCheckMm']=min(heldmins)
d['cases']=[shot,lift]
other=a['cases'][0].copy();other.update(id='Inspect parked cassette',key='robot',basis='Closed spring flap, NOTE held in the compressed tread.');d['cases'].append(other)
d['static']+=extra['positive'];d['notes']=[shot['basis'],lift['basis'],'Field meshes are reduced from FE-2024.STEP in its registered coordinates. The physics uses separate original speaker surfaces and a lumped chain. The colored opening outline marks the actual scoring plane.','Nominal foam compression is represented by the two rigid tread/NOTE overlaps. Six post reliefs clear the outer tread return. Baseline and failing trials are retained locally.']
d['mass'].append(dict(id='Replacement traction tread and hardware allowance',kg=[.06,.22],members=[n for n in b if n.startswith('Compliant belt tread')]))
for k in ['inspection','running']:d[k]=[round(v+w,4) for v,w in zip(d[k],[.06,.22])]
for row in d['mass']:
    if row['id']=='Shared cassette lift':row['members']+=[n for n in b if n.startswith(('Hook keeper','Chain hook'))]
d['required'].append(['Gated yoke and reversing tread',[n for n in b if n.startswith(('Hook keeper','Chain hook','Compliant belt'))]])
def electrical(c,amps,base,members):
    total=base+amps;vlo=11.8-.027*total
    return dict(case=c['id']+' / weak',stage=c['id'],batteryCondition='weak',openCircuitV=11.8,resistance=.027,requiredMinV=8,loadedBusV=[vlo,11.8-.027*base],batterySideA=[base,total],marginV=vlo-8,status='pass' if vlo>=8 else 'hold',lines=[dict(name='Other simultaneous loads (assumed)',amps=[base,base],members=[]),dict(name='Mission drive',amps=[0,amps],members=members)])
d['power']=[electrical(shot,s['estimatedPeakDriveA'],25,[n for n in b if n.startswith('Belt motor ·')]),electrical(lift,cl['peakNm']/(.0198*250*.72),25,['Pivot motor','Pivot reducer'])]+d['power']
d['narrativeReview']['objectives']=[dict(objective='Deliver the finite NOTE into the speaker',disposition='Simulated',basis=shot['results'][0]['witness']['reason']),dict(objective='Hang after power is disabled',disposition='Conditional',basis=lift['results'][1]['witness']['reason'])]
d['receipt']=hashlib.sha256((p/'real1.json').read_bytes()+(p/'hooked.json').read_bytes()).hexdigest()
(p/'mission-data.json').write_text(json.dumps(d,separators=(',',':')),encoding='utf8')
print(len(b),[len(c.get('time',[])) for c in d['cases']],min(heldmins),f['both'])

