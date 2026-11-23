"""Publish the timed endgame branches and preserve the previous scope."""
from access import *
import copy
web=root.parent/'www/data.json';b=read(p/'branch.json')
if not (p/'original-climb.json').exists():
    for key in ('climb','park'):
        (p/('original-'+key+'.json')).write_text(json.dumps(next(c for c in layout['cases'] if c['key']=='field-'+key),separators=(',',':')))
old=read(p/'original-climb.json')
trace=read(root/'chain-contact-02/front48.json')
r=b['branches']['terminal snapshot'];way=np.array(r['path']);duration=np.array(r['times']);wait=np.array(way[-1]);mouth=np.array(read(p/'access.json')['mouth']);yaw=b['park']['riseS'];a=read(p/'access.json')['angle'];entryApproach=read(root/'keeper-drive-02/root-entry.json')['poses']
pin=mat(0,457.5,96);rest=old['chain'][0]
def motion(t):
    for tseg,L,R in zip(duration,way,way[1:]):
        if t<tseg:return interp(t,L,R,tseg)
        t-=tseg
    return wait
def interp(t,L,R,T):
    delta=R-L;length=np.linalg.norm(delta)
    if length<1e-8:return R
    up=min(1.2,1.4*T/2)/1.4
    s=.5*1.4*t*t if t<up else length/1000-.5*1.4*(T-t)**2 if t>T-up else 1.4*up*(t-up/2)
    return L+delta*np.clip(s*1000/length,0,1)
def build(parked):
    c=copy.deepcopy(old);c.update(key='field-park' if parked else 'field-climb',id='PARK from the 24-second decision' if parked else 'ONSTAGE approach and loaded contact',time=[],positions=[],chain=[],measure=[],series={s:[] for s in old['series']},results=[])
    c['basis']='The archived terminal snapshot fixes the robot and five peers. Drive legs use a 1.2 m/s speed cap with stops at corners. '+('The same approach stops outside the chain with the keeper closed. A vertex of the actual bumper lies 43.3 mm inside the upright triangle.' if parked else 'The inward climb shown is rejected. The bumper passes through a fixed stage rail in the official field CAD, and the chain strikes the battery and slide motor. Two lower catch angles still hit field steel.')
    if parked:c['fixture']='park'
    def add(t,xy,angle,gate,txt):
        mx=pose(angle,gate);mapping={**mx,'piece':np.eye(4),'pin':mx['frame']}
        c['time'].append(t);c['positions'].append(np.asarray(xy).tolist());c['measure'].append(txt);c['chain'].append(rest)
        for s in c['series']:c['series'][s].append(mapping[s][:3].round(8).ravel().tolist())
    dt=duration.sum();end=np.array(b['park']['position']) if parked else mouth
    cruise=times([wait,end])[0];last=dt+yaw+cruise
    for t in np.unique(np.r_[np.arange(0,last,.05),dt,dt+yaw,last]):
        u=np.clip((t-dt)/yaw,0,1);smooth=u*u*(3-2*u)
        heading=125+(a-125)*smooth;gate=0 if parked else smooth*44
        xy=motion(t) if t<=dt else wait if t<=dt+yaw else interp(t-dt-yaw,wait,end,cruise)
        msg='Drive around stage and peers' if t<dt else 'Set approach angle' if t<dt+yaw else 'Approach PARK' if parked else 'Approach open chain throat'
        add(float(t),xy,heading,gate,f'{msg} · {24-t:.1f} s to match cutoff')
    if parked:
        add(last+.001,end,a,0,f'PARK · bumper {b["park"]["depthInsideFieldTriangleMm"]:.1f} mm inside · {24-last:.2f} s spare')
        add(24,end,a,0,f'PARK at cutoff · bumper {b["park"]["depthInsideFieldTriangleMm"]:.1f} mm inside')
        c['results'].append(dict(pair=['Blue bumper set','PARK region'],status='sampled',witness=dict(reason=f'Actual bumper vertex inside at {last:.2f} s; no bounding-box conflict on this path. The earlier lowered-arm route has field candidates with the keeper hardware.')))
    else:
        for j,t in enumerate(old['time']):
            if j==0:continue
            c['time'].append(float(last+t));c['positions'].append(old['positions'][j]);c['chain'].append(old['chain'][j]);c['measure'].append(old['measure'][j]+(' · rejected inward climb · fixed stage rail and chain interference' if t>=1.2+2.8 else ''))
            for s in c['series']:c['series'][s].append(old['series'][s][j])
        c['results'].append(dict(pair=['Blue bumper set','Fixed stage rail GE-24222'],status='fail',witness=dict(reason='Actual solids intersect by 611,052 mm³ on the recorded inward swing. The lower catch angles also intersect a fixed rail.')))
        c['results'].append(dict(pair=['Battery and terminal case','Working chain'],status='fail',witness=dict(reason='During the loaded swing the capsule centerline enters the actual battery envelope. The first 100 ms-sampled contact is at 2.80 s after the throat reaches the chain.')))
        c['results'].append(dict(pair=['Slide drive motor','Working chain'],status='fail',witness=dict(reason='The motor and swept chain overlap by up to 2.55 mm radially in the original hanging simulation.')))
    return c
for item in [build(False),build(True)]:
    idx=next(i for i,k in enumerate(layout['cases']) if k['key']==item['key']);layout['cases'][idx]=item
for obj in layout['narrativeReview']['objectives']:
    if 'PARK' in obj['objective']:obj.update(disposition='Route checked',basis='The revised east-side route takes 12.61 s and places the bumper 43.3 mm inside. The five terminal-snapshot peers were included.')
    elif 'ONSTAGE' in obj['objective']:obj.update(disposition='Rejected inward climb',basis='The swept bumper cuts the fixed stage rail. The chain also enters the battery and slide motor. PARK is the supported fallback.')
layout['focus']='Profile A pickup, six-NOTE repeatability and full-robot climb contact. PARK has a common east-side approach.'
layout['status']='Preliminary design held · six-NOTE repeatability and ONSTAGE'
for obj in layout['narrativeReview']['objectives']:
    if 'six' in obj['objective'].lower() or 'six-note' in obj['objective'].lower():obj.update(disposition='Repeatability target missed',basis='Two failed setups in the frozen twenty-start roster. R01 missed the paired counters; R08 never seated the third NOTE. The maximum is 18 of 20 against a target of 19.')
tmp=web.with_suffix('.endgame-next');tmp.write_text(json.dumps(d,separators=(',',':')),encoding='utf8');tmp.replace(web)
print([(k['key'],len(k['time']),k['time'][-1]) for k in layout['cases'] if k['key'] in ('field-park','field-climb')])
