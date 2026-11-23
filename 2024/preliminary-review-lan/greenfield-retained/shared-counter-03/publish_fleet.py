"""Build the six visible NOTE instances from the verified carry and shared-field sources."""
from pathlib import Path
import json
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from make_generator import p,old,read,sha
from six_field import publish
plan=read(old/'plan.json')
web=p.parent.parent/'www'
data=read(web/'data.json')
guided=data['concepts']['guided']
angle=np.radians(125)
rot=np.array([[np.cos(angle),0,-np.sin(angle)],[0,1,0],[np.sin(angle),0,np.cos(angle)]])
park=np.eye(4);park[:3,:3]=rot;park[:3,3]=rot@np.array([215,0,50.4])+[185,0,501]
unseat=np.linalg.inv(park)
response=[]
for n in (48,96):
    case=next(c for c in guided['cases'] if c['key']==f'continuous-cycle-{n}')
    source=p/f'six-shared-dev-{n}.json';model=read(source);assert model['terminal'] and model['clock']>=119.99
    trfile=source.with_suffix('.trajectory.json');assert sha(trfile)==model['trajectory']
    raw=read(trfile);run=read(old/f'run-dev-{n}.json');assert model['inputRun']==sha(old/f'run-dev-{n}.json')
    clock=np.asarray(case['time']);xy=np.asarray(case['positions']);pieces={}
    for i,(leg,part) in enumerate(zip(plan['plan'],run['parts'])):
        k=leg['id']
        name=('Blue wing '+k if i<3 else 'Center line '+k.split('-')[-1])+' NOTE'
        acquired=(clock>=part['depart']-1.e-6)
        if i:acquired&=clock>part['depart']+1.e-6 # existing boundary frame still displays previous NOTE
        shotClock=part['shotStart'];simulation=clock>=shotClock-1.e-6
        # Near the mouth use the dense independent shot poses up to the first possible coupled contact.
        inputShot=old/f'split-tail_.25_0-dev-{k}-{n}-6.88.json'
        assert sha(inputShot)==part['shot']
        s=read(inputShot);src=s['trace']
        firstContact=min((v['firstS'] for pair,v in model['contacts'].items() if k in pair.split('/')),default=float('inf'))
        incoming=[(v['t']+shotClock,v['q']) for v in src if v['t']+shotClock<firstContact]
        observed=[(t,states[i]) for t,states in raw if len(states)>i and t>incoming[-1][0]+1.e-5]
        path=incoming+observed
        t=np.array([x[0] for x in path]);q=np.array([x[1] for x in path])
        matrices=np.tile(np.eye(4),(len(clock),1,1))
        matrices[:,:3,3]=np.r_[np.array(leg['mark'])*1000,25.4]-np.c_[xy,np.zeros(len(clock))]
        matrices=matrices@unseat
        active=acquired&~simulation
        matrices[active,:3]=np.asarray(case['series']['piece'])[active].reshape(-1,3,4)
        T=np.clip(clock[simulation],t[0],t[-1])
        quats=Slerp(t,Rotation.from_quat(q[:,[4,5,6,3]]))(T)
        flight=np.tile(np.eye(4),(len(T),1,1));flight[:,:3,:3]=quats.as_matrix()
        flight[:,:3,3]=1000*np.column_stack([np.interp(T,t,q[:,j]) for j in range(3)])-np.c_[xy[simulation],np.zeros(len(T))]
        matrices[simulation]=flight@unseat
        assert np.max(abs(np.linalg.det(matrices[:,:3,:3])-1))<.00001
        pieces[name]=matrices[:,:3].reshape(-1,12).round(6).tolist()
    assert len(pieces)==6
    if 'individualMeasure' not in case:case['individualMeasure']=case['measure']
    case['measure']=[s.split(' · ')[0]+' · '+(' · '.join(s.split(' · ')[1:-1]))+f" · {sum(v<=t for v in model['counts'].values())}/6 shared-counter events" for t,s in zip(clock,case['individualMeasure'])]
    case['fieldNotes']=pieces
    case['basis']='Each of the six floor NOTES keeps its identity. Select a NOTE to follow its position. The speaker replay retains earlier NOTES, their contacts and the time both beams clear before the next event. Field motion between saved physics samples is interpolated for display.'
    case['results']=[r for r in case['results'] if r['pair']!=['Shared field counter','Repeat deliveries']]
    case['results'].append(dict(pair=['Shared field counter','Repeat deliveries'],status='sampled' if len(model['counts'])==6 else 'open',witness=dict(reason=f"{len(model['counts'])} distinct events by 120 s in the shared speaker on this nominal start. Six original releases, with all earlier NOTES remaining in the simulation.")))
    response.append(dict(resolution=n,trace=sha(trfile),model=sha(source),counts=model['counts'],instances=list(pieces),times=len(clock)))
publish(web/'data.json',data)
publish(p/'six-visible-pieces.json',response)

