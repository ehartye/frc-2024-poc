"""Trace the empty robot from the sixth observed score to the selected guided-hoist approach."""
from pathlib import Path
import sys,json,math,hashlib
import numpy as np
here=Path(__file__).resolve().parent;root=here.parent
sys.path.insert(0,str(root/'endgame-route-01'))
import access as a
doc=a.d;guided=doc['concepts']['guided'];slide=doc['slideService']
score=next(c for c in guided['cases'] if c['key']=='continuous-cycle-48')
front=next(c for c in guided['cases'] if c['key']=='guided-front')
stock={**doc['bodies'],**doc['concepts']['field']['add'],**guided['add'],**slide['add']}
omit=set(guided.get('fixtureOnlyNames',[]))|set(front['removed'])
names=sorted((score['rig'].keys()-omit)|slide['add'].keys())
assert all(n in stock for n in names)
def tr(v):return np.array([*v,0,0,0,1]).reshape(4,4)
def transform(m,v):return v@m[:3,:3].T+m[:3,3]
verts={n:np.array(stock[n]['v']) for n in names}
anglePos=lambda t:a.mat(t,185.,501.)
def bounds(deg):
    M={k:tr(v[-1]) for k,v in score['series'].items()}
    delta=anglePos(deg)@np.linalg.inv(anglePos(122))
    for group in ['arm','flap']:M[group]=delta@M[group]
    out=[]
    for n,v in verts.items():
        group='frame' if n in slide['add'] else score['rig'][n]
        points=transform(M[group],v);out.append([points.min(axis=0),points.max(axis=0)])
    return np.array(out)
safe=a.read(root/'endgame-route-01/access.json')
a.names=names
a.boxes=lambda deg,gate=0:bounds(deg)
start=np.array(score['positions'][-1]);wait=np.array(safe['waiting'])
assert np.allclose(start,[1390,5871])
high=a.Field(122);path=a.search(high,start,wait)
assert path
hits=[high.hits(s,e,True) for s,e in zip(path,path[1:])]
slew=a.transition(wait,122,125)
assert not any(hits) and slew['pass5mm']
branch=a.read(root/'endgame-route-01/branch.json')['branches']['terminal snapshot']
original=np.array(front['positions'])
arrival=[i for i,xy in enumerate(original) if np.linalg.norm(xy-wait)<1e-6][0]
oldApproach=front['time'][arrival]
drive=sum(a.times(path))
cut=next(t for t,m in zip(front['time'],front['measure']) if m.startswith('Power cut'))
handoff=drive+1.
w=dict(step=guided['step'],serviceParts=slide['step'],robotBodies=len(names),obstacleAndPeerSlabs=len(a.ids),waiting=wait.tolist(),start=start.tolist(),path=[v.tolist() for v in path],legsS=a.times(path),driveS=drive,plannedPoseChangeS=1.,slew=slew,segmentHits=hits,blockedShortcut=high.hits(start,wait,True)[:5],earlierRouteArrivalAtWaitS=oldApproach,earlierPowerCutS=cut,powerCutFromSixthS=handoff+cut-oldApproach,remainingIn24S=24-(handoff+cut-oldApproach),holdToEndS=handoff+front['time'][-1]-oldApproach,scope='Continuous field slabs for travel with robot at 122 degrees, including the new motor package. At the waiting point the unloaded cassette moves to the prior 125-degree approach. Subsequent 230 mm guided climb uses the existing forward-CG physics trial and an archived five-peer field envelope. This composes geometry and timing. Service overlays, changed mass, live traffic and field availability require their own checks during the loaded climb.')
(here/'endgame-transfer.json').write_text(json.dumps(w,indent=2))
print(drive, w['powerCutFromSixthS'],w['remainingIn24S'],len(names),len(a.ids),slew)

