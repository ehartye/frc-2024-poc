from pathlib import Path
import json,sys,hashlib,math,copy,numpy as np
p=Path(__file__).resolve().parent;sys.path.insert(0,str(p.parent/'follower-01'));from stroke import scheduled,mat
r=json.loads((p/'park-recovery.json').read_bytes());www=p.parent.parent/'www/data.json';design=json.loads(www.read_bytes());assert design['step']==r['parent']
def m4(a,x=0,y=0,z=0):
    v=mat(a,x,z);v[1,3]=y;return v
P=m4(125,185,0,501);ip=np.linalg.inv(P);H=m4(0,457.5,0,96)
source=next(c for c in design['cases'] if c['key']=='pickup-to-shot')
c=copy.deepcopy(source);c.update(key='park-route',id='PARK · loaded-cycle fallback',fixture='climb',positions=[],time=[],series={k:[] for k in c['series']},measure=[],removed=list(set(c['removed']+['Chambered NOTE'])),stage='Terminal decision',basis='Empty robot at the 24-second decision. The cassette lowers to −18° before it drives. Continuous bounds cover current rigid stock against the registered field and five peers. The left bumper point ends inside the upright triangle.',results=[])
c.pop('chain',None)
points=np.array(r['waypointsIn'])*25.4;dur=np.array(r['timesS']);total=sum(dur);dt=2.9
start=points[0];goal=points[-1]
# Check transition stock against nearby sourced field and the full peer boxes before publishing.
from route import fielddata,ignore
sh=[(n,np.array(design['bodies'][n]['v']),g) for n,g in c['rig'].items() if n not in c['removed']]
assert len(sh)==r['placedBodies']
field=[]
for part in fielddata['parts']:
    if part['path'][1] in ignore:continue
    for a,b in part.get('tightBounds',[part['bounds']]):
        aa=np.asarray(a)*1000;bb=np.asarray(b)*1000
        if bb[2]>.001 and aa[2]<1300 and np.all(aa[:2]<start+800) and np.all(bb[:2]>start-800):field.append([aa,bb])
raw=json.loads((p.parent/'park-inputs.json').read_bytes());actors=raw['frames'][0]['actors']
for j,actor in enumerate(actors[1:],1):
    loc=np.array([v['actors'][j]['p'] for v in raw['frames']])*25.4;half=25.4*(np.array([actor['length'],actor['width']])/2+actor['bumper'])
    field.append([np.r_[loc.min(axis=0)-half,-100],np.r_[loc.max(axis=0)+half,2000]])
bounds=np.array(field);minsep=1e8
def transforms(q):
    pose=m4(q,scheduled(q),0,501);return dict(frame=np.eye(4),slide=m4(0,scheduled(q)-185),arm=pose@ip,flap=pose@H@m4(90*np.clip((q-102)/18,0,1))@np.linalg.inv(H)@ip,piece=np.eye(4))
for q in np.linspace(-18,125,573):
    poses=transforms(q)
    for n,v,g in sh:
        m=poses[g];w=v@m[:3,:3].T+m[:3,3]+[*start,0]
        sep=np.maximum(bounds[:,0]-w.max(axis=0),w.min(axis=0)-bounds[:,1]).max(axis=1)
        minsep=min(minsep,float(min(sep)))
assert minsep>5,minsep
def at(s):
    for i,T in enumerate(dur):
        if s<=T:
            a=1.4;v=min(1.2,a*T/2);rise=v/a;L=np.linalg.norm(points[i+1]-points[i])/1000
            x=.5*a*s*s if s<rise else L-.5*a*(T-s)**2 if s>T-rise else v*(s-rise/2)
            return points[i]+(points[i+1]-points[i])*x/L
        s-=T
    return points[-1]
for t in np.unique(np.r_[np.arange(0,dt+total,.05),dt+total,24.]):
    u=np.clip(t/dt,0,1);q=125-143*u*u*(3-2*u);xy=at(max(0,t-dt));c['positions'].append(xy.round(3).tolist());c['time'].append(t)
    for k,v in transforms(q).items():c['series'][k].append(v[:3].round(8).ravel().tolist())
    c['measure'].append(f'{"Lowering for stage clearance" if t<dt else "In PARK" if t>=dt+total else "Driving to PARK"} · cassette {q:.0f}° · {24-t:.1f} s to disable')
r.update(postureChangeS=dt,transitionMinSlabMm=minsep,transitionInterSampleBoundMm=5.,timeFromScoringPostureS=dt+total,reserveFromScoringPostureS=24-dt-total)
(p/'park-issued.json').write_text(json.dumps(r,indent=2))
c['fixture']='park';c['results'].append(dict(pair=['Blue bumper set','PARK'],status='sampled',witness=dict(reason=f'Bumper witness inside triangle at {dt+total:.2f} s. Previous horizontal straight-route trial has 63 unresolved stock/field pairs.')))
design['cases']=[k for k in design['cases'] if k['key']!=c['key']]+[c]
for o in design['narrativeReview']['objectives']:
    if 'PARK' in o['objective']:o.update(disposition='Modeled on current layout',basis=f'From the projected decision pose the empty cassette lowers to −18°. Current rigid stock and five peers clear for {dt+total:.2f} s to PARK with {24-dt-total:.2f} s spare.')
tmp=www.with_suffix('.next');tmp.write_text(json.dumps(design,separators=(',',':')),encoding='utf8');tmp.replace(www);print(r['timeFromScoringPostureS'],minsep)

