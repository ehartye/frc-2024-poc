"""Publish the development route on a single field clock, using the physical counter for each ID."""
from pathlib import Path
import numpy as np,json,math,hashlib,copy,sys
from scipy.spatial.transform import Rotation
p=Path(__file__).resolve().parent;parent=p.parent/'follower-01';sys.path.insert(0,str(parent));from stroke import scheduled
def read(f):return json.loads(f.read_bytes())
def sha(f):return hashlib.sha256(f.read_bytes()).hexdigest()
def pose(a,x=0,y=0,z=0):
    v=np.eye(4);v[:3,:3]=Rotation.from_euler('y',-a,degrees=True).as_matrix();v[:3,3]=[x,y,z];return v
def yaw(q):
    v=np.eye(4);v[:3,:3]=Rotation.from_euler('z',q).as_matrix();return v
def ring(quat,r):
    v=np.eye(4);v[:3,:3]=Rotation.from_quat([*quat[1:],quat[0]]).as_matrix();v[:3,3]=r;return v
def flat(v):return v[:3].round(7).flatten().tolist()
park=pose(125,185,0,501);invPark=np.linalg.inv(park);iseat=np.linalg.inv(park@pose(0,215,0,50.4));axis=pose(0,457.5,0,96)
plan=read(p/'plan.json');www=p.parent.parent/'www/data.json';d=read(www)
assert d['step']==plan['placedStep'] and d['defaultPickupBlade']['default']=='A'
old=next(c for c in d['cases'] if c['key']=='pickup-to-shot');c=copy.deepcopy(old)
c.update(key='six-note-indexed',id='Six-NOTE field rehearsal · indexed, fine',fixture='shot',positions=[],time=[],measure=[],series={r:[] for r in c['series']},results=[],stage='Mission rehearsal',basis='Six nominal floor marks on a common clock. The viewer follows the active NOTE. Robot acceleration and yaw act on the free NOTE. Scoring requires whole opening passage and both modeled counters. Empty legs and pivot motion are commanded. Field bounds and dormant NOTE envelopes were screened during planning. Each pickup starts with the intake at rest; electrical recovery, spin-down and feedback are provisional.')
def append(t,xy,angle,P,px,q,flap,caption):
    if c['time'] and t<=c['time'][-1]+1e-7:return
    turn=yaw(q);arm=pose(angle,px,0,501)
    poses=dict(frame=turn,piece=P@iseat,arm=turn@arm@invPark,slide=turn@pose(0,px-185,0,0),flap=turn@arm@axis@pose(flap)@np.linalg.inv(axis)@invPark)
    c['time'].append(round(t,6));c['measure'].append(caption);c['positions'].append((np.array(xy)*1000).round(4).tolist())
    for k in c['series']:c['series'][k].append(flat(poses[k]))
def moment(path,durations,t):
    for i,T in enumerate(durations):
        if t<T:
            u=t/T;return path[i]+(path[i+1]-path[i])*u*u*(3-2*u),i,u
        t-=T
    return path[-1],len(durations)-1,1.
def clean(n):return 'Blue wing '+n if n in ['inner','middle','outer'] else n.replace('center-','Center line ')
clock=0;count=0;report=[]
for idx,leg in enumerate(plan['plan']):
    name=leg['id'];prefix=clean(name);tag='dev-'+name+'-96';r=read(p/(tag+'.json'));s=read(p/('indexed-'+tag+'.json'))
    mark=np.array(leg['mark']);q=leg['yaw'];deployed=r['pitchDegrees'];startX=169.3625892825837;S=scheduled(deployed)
    origin=np.array(leg['out'][0]);out=np.array(leg['out']);loaded=np.array(leg['loaded']);scoreXY=loaded[-1];td=sum(leg['outT'])
    pre=1.15 if idx==0 else 3.4
    for t in np.arange(0,pre+td,.08):
        if idx==0:
            xy,_,_=moment(out,leg['outT'],t);u=np.clip((t-td)/pre,0,1);a=deployed*u*u*(3-2*u)
        else:
            xy,_,_=moment(out,leg['outT'],max(0,t-pre));u=np.clip(t/pre,0,1);a=125+(deployed-125)*u*u*(3-2*u)
        P=pose(0,*((mark-xy)*1000),25.4);append(clock+t,xy,a,P,scheduled(a),q,90*np.clip((a-102)/18,0,1),f'{prefix} · empty approach · {count}/6 scored')
    clock+=pre+td;start=clock;hYaw=yaw(q)
    for j,row in enumerate(r['trajectory'][::5]):
        t=row[0];adv=min(leg['advance'],t*.25);xy=out[-1]+hYaw[:2,0]*adv;y=q
        if t>2.82:
            xy,i,part=moment(loaded,leg['loadedT'],t-2.82)
            if idx==0:y=q*(1-part*part*(3-2*part)) if i==1 else (0 if i>1 else q)
        u=np.clip((t-r['startLift'])/3.4,0,1);bl=u*u*(3-2*u);a=deployed+(125-deployed)*bl;px=scheduled(a)+(1-bl)*(startX-S)
        P=yaw(y)@ring(row[4:8],np.array(row[1:4])*1000)
        append(clock+t,xy,a,P,px,y,math.degrees(row[-1]),f'{prefix} · {"intake" if t<2.82 else "loaded return"} · {count}/6 scored')
    clock+=r['finish'];shotStart=clock;result='registered' if s['passShot'] else 'miss'
    stop=s['both'] if s['passShot'] else s['trace'][-1]['t']
    for row in s['trace'][::3]+[min(s['trace'],key=lambda v:abs(v['t']-stop))]:
        t=min(stop,row['t']);P=ring(row['q'][3:],(np.array(row['q'][:3])-[*scoreXY,0])*1000)
        append(clock+t,scoreXY,125,P,185,0,90,f'{prefix} · {result if t>=stop else "scoring attempt"} · {count+(s["passShot"] and t>=stop)}/6 scored')
        if t>=stop:break
    clock+=stop;count+=int(s['passShot'])
    report.append(dict(id=name,floorStartS=start,scoreRequestS=shotStart,endS=clock,seat=r['seated'],held=r['held'],opening=s['opening'],counterS=clock if s['passShot'] else None,release=s['release']['t'] if s['release'] else None,busV=s['minBusV'],files={f.name:sha(f) for f in [p/(tag+'.json'),p/('indexed-'+tag+'.json')]}))
    c['results'].append(dict(pair=[prefix,'Speaker'],status='sampled' if s['passShot'] else 'fail',witness=dict(reason=f'{result} at {clock:.2f} s on the development clock.')))
res=dict(planHash=sha(p/'plan.json'),placedStep=d['step'],rows=report,scoreCount=count,timeS=clock,target=[6,120],frozenRosterAttempts=0,calibration='Development, nominal marks, 96 segments. Supply is finite. Holding and field scoring are simulated; route tracking and empty arm resetting are commanded. Positive NOTE custody through return and a score are separate evidence.')
(p/'mission.json').write_text(json.dumps(res,indent=2))
d['cases']=[k for k in d['cases'] if k['key']!=c['key']]+[c];d['narrativeReview']['objectives'][1]['basis']=f'Six nominal marks rehearsed in {clock:.2f} s. {count}/6 registered on the development route. Qualification runs on the frozen roster remain open.'
d['focus']='One nominal 96-segment run scored six. The 48-segment check scored four. Frozen setup variation is under evaluation.'
target=www.with_suffix('.next');target.write_text(json.dumps(d,separators=(',',':')),encoding='utf8');target.replace(www)
print(count,clock,c['measure'][-1],len(c['time']))

