# Add shared-layout operating and service views and give each moving belt an orientation witness.
from pathlib import Path
import json, math, numpy as np
from scipy.spatial.transform import Rotation
p=Path(__file__).resolve().parent;d=json.loads((p/'mission-data.json').read_bytes())
src=json.loads((p/'p3-data.json').read_bytes())
shot,lift,parked=d['cases'];b=d['bodies'];flap='Mouth retention flap'
def mat(deg,x=0,y=0,z=0):
    m=np.eye(4);m[:3,:3]=Rotation.from_euler('y',-deg,degrees=True).as_matrix();m[:3,3]=[x,y,z];return m
def flat(m):return m[:3].flatten().round(8).tolist()
def matrix(f):return np.vstack((np.asarray(f).reshape(3,4),[0,0,0,1]))
base=mat(125,185,0,501);hinge=mat(0,399.5,0,84);identity=flat(np.eye(4))
parked['frames'][flap]=[flat(base@hinge@mat(90)@np.linalg.inv(hinge)@np.linalg.inv(base))]*25
parked['basis']='Aim stop with cam holding the mouth open. Foam preload is represented by rigid tread/NOTE intersection.'
for j in (1,2,3,4,5):
    old=src['cases'][j];c={k:v for k,v in old.items() if k not in ('id','frames','results','status','basis')}
    c.update(id=old['id'],key=['deploy','intake','raise','service','supported-service'][j-1],status='review',stage='operating envelope',basis='Preliminary motion envelopes. The finite piece follows the registered intake path. Passive flap motion on admission is an ideal follower. Clearance to compliant stock and service support remain under review.',frames={},results=[])
    for i in range(25):
        def get(n):return old['frames'].get(n,[identity]*25)[i]
        arm=matrix(get('Rear piece stop'));world=arm@base;deg=math.degrees(math.atan2(world[2,0],world[0,0]))
        openCam=90*np.clip((deg-102)/18,0,1)
        # During the floor feed the entering piece can push the flap against its return spring.
        passiveOpen=90*math.sin(math.pi*i/24) if j==2 else 0
        flapPose=flat(world@hinge@mat(openCam if openCam else -passiveOpen)@np.linalg.inv(hinge)@np.linalg.inv(base))
        for n in b:
            if n in c['removed']:continue
            f=get(n) if n in old['frames'] else flat(arm) if shot['rig'][n]=='arm' else identity
            c['frames'].setdefault(n,[]).append(flapPose if n==flap else f)
    c['frames']={n:fs for n,fs in c['frames'].items() if not np.allclose(fs,identity)}
    c['results']=[r for r in old['results'] if all(n in b for n in r['pair']) and r['status'] in ('fail','open')]
    c['results'].append(dict(pair=['Compliant belt tread','Chambered NOTE'],status='open',witness=dict(reason='Inherited path. Pickup force, follower contact and moving clearances need a matched trial on this tread revision.')))
    d['cases'].append(c)
b[flap]['purpose']='The incoming NOTE can push the spring flap inward. At the firing angle a cassette-driven cam releases the outward stop and folds the flap clear.'
b[flap]['rationale']='Pickup, retention and reverse discharge share one mouth. The spring returns the flap. The stop release, follower and clearance under load need hardware verification.'
for n in b:
    if n.startswith('Opposed traction belt'):
        b[n]['purpose']='The inboard belt run pulls toward the stop for intake, then reverses toward the mouth for shooting.'
        b[n]['rationale']='Both facing runs accelerate the NOTE in the same direction. The mirrored drive pulleys turn in opposite senses.'
read=json.loads((p/'real1.json').read_bytes());shot['beltSpeed']=[0.]*40+[sum(r['beltMps'])/2 for r in read['trace']][:len(shot['time'])-40]
d['notes'].append('Drive screen uses 50 A motor limit per side, direct pulleys, 0.20 kg effective moving mass per belt and assumed grip. At the sensor-registered placement, the 0.18 and 0.22 kg cases also register. Moving the robot 120 mm downfield misses registration in the tested coarse run.')
d['notes'].append('The ideal hook constraint starts from a chain within 12 mm of each throat. Gate capture from a driving approach, clearance to every truss member, air drag and drive hardware are open. The suspended chassis continues swinging during the disabled interval.')
(p/'mission-data.json').write_text(json.dumps(d,separators=(',',':')),encoding='utf8')

