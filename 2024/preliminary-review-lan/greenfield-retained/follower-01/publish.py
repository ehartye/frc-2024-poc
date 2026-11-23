"""Register the passive followers and show the solved single-NOTE station rehearsal."""
from pathlib import Path
import json,copy,math,numpy as np,hashlib
from scipy.spatial.transform import Rotation
from stroke import scheduled
p=Path(__file__).resolve().parent;www=p.parent.parent/'www'
def read(f):return json.loads(f.read_bytes())
def sha(f):return hashlib.sha256(f.read_bytes()).hexdigest()
def matrix(a,x=0,y=0,z=0):
    f=np.eye(4);f[:3,:3]=Rotation.from_euler('y',-a,degrees=True).as_matrix();f[:3,3]=[x,y,z];return f
def piece(q,v):
    f=np.eye(4);f[:3,:3]=Rotation.from_quat([*q[1:],q[0]]).as_matrix();f[:3,3]=v;return f
def flat(f):return f[:3].round(8).ravel().tolist()
d=read(p.parent/'catch-layout-03/review-data.json');f=read(p/'fit.json');response=read(p/'response.json');mesh=read(p/'mesh.json')
assert d['step']==f['source'] and not f['overlaps'] and f['step']==sha(p/'paired-follower.step')
descs={'Carpet follower':('Roll on the carpet to set pickup height.','Two outboard wheels let the lip follow the floor while leaving the ring inlet open.'),'Follower shoulder axle':('Locate the wheel on the drilled cassette side extension.','The axle shoulder leaves the wheel free when the locknut is tightened.'),'Follower thrust washer':('Separate the rotating wheel from its mounting plate.','A replaceable washer takes lateral rubbing.'),'Follower locknut':('Retain the follower axle.','The locknut bears on the axle shoulder.')}
for n,g in mesh.items():
    if n in d['bodies']:d['bodies'][n].update(g);continue
    ty=n.split(' · ')[0];pu,ra=descs[ty]
    d['bodies'][n]=dict(**g,group='Carpet followers',category='mechanism',material=f['added'][n],purpose=pu,rationale=ra,basis='Preliminary passive follower. Effective contact stiffness and wear remain estimates.')
    d['support'][n]='Wheel to shoulder axle, cassette side extension and pivoting side beam.'
for c in d['cases']:
    ref='Cassette side beam extension · left'
    for n in f['added']:
        if 'rig' in c:c['rig'][n]=c['rig'].get(ref,'arm')
        elif ref in c['frames']:c['frames'][n]=c['frames'][ref]
    if c['key'].startswith('trial-') or c['key']=='intake':c['removed']+=list(f['added'])
d['mass'].append(dict(id='Carpet followers, axles and added cheek material',kg=[f['installedAddedKg'],.05],members=list(f['added'])))
for k in ('inspection','running'):d[k]=[d[k][0]+f['installedAddedKg'],d[k][1]+.05]
d['step']=d['defaultPickupBlade']['assembly']=f['step'];d['issue']='Shared cassette · floor-following pickup';d['focus']='Test the six-piece field route with the same free-piece handoff used in the station rehearsal.'
park=matrix(125,185,0,501);inv=np.linalg.inv(park);hinge=matrix(0,457.5,0,96);iseat=np.linalg.inv(park@matrix(0,215,0,50.4))
out=[]
for nn in [48,96]:
    tr=read(p/f'cam{nn}.json');sh=read(p/f'fire{nn}.json');dep=tr['pitchDegrees'];base=169.3625892825837
    expect=np.array(tr['endPieceQpos']);expect[:3]+=[.99,5.55,0]
    assert np.allclose(expect,sh['initialState'],rtol=0,atol=1e-12) and np.array_equal(tr['endPieceQvel'],sh['initialVelocity'])
    c=copy.deepcopy(next(c for c in d['cases'] if c['key']=='intake'))
    c.update(key='pickup-to-shot' if nn==96 else 'pickup-to-shot-48',fixture='shot',id='Floor pickup to speaker · '+str(nn)+' segments',time=[],measure=[],positions=[],series={k:[] for k in c['series']},frames={},results=[],stage='Station rehearsal',basis='Same free NOTE through intake and elevation. Its solved pose and velocity start the powered field replay at the same station. The chassis approach and arm motion are prescribed. The contact solver changes at the firing station. This floor placement is a test fixture.')
    c['removed']=[n for n in c['removed'] if n not in f['added']];c.pop('beltSpeed',None)
    def add(t,adv,x,angle,fp,P,text):
        pos=matrix(angle,x,0,501)
        poses=dict(arm=pos@inv,slide=matrix(0,x-185,0,0),flap=pos@hinge@matrix(fp)@np.linalg.inv(hinge)@inv,frame=np.eye(4),piece=P@iseat)
        c['time'].append(t);c['positions'].append([990+adv,5550]);c['measure'].append(text)
        for k in c['series']:c['series'][k].append(flat(poses[k]))
    for row in tr['trajectory'][::2]:
        t=row[0];u=np.clip((t-2.82)/3.4,0,1);b=u*u*(3-2*u);q=dep+(125-dep)*b;x=scheduled(q)+(1-b)*(base-scheduled(dep));adv=min(440,250*t)
        add(t,adv,x,q,math.degrees(row[-1]),piece(row[4:8],np.array(row[1:4])*1000),f"{'Floor pickup' if t<2.82 else 'Raising cassette' if t<6.22 else 'At firing angle'} · {q:.1f}° · NOTE center {row[8]*1000:.0f} mm along cassette")
    for i,row in enumerate(sh['trace']):
        if row['t']==0:continue
        t=row['t'];through=sh['opening'] and t>=sh['opening']['fullyThroughByS'];hit=sh['both'] is not None and t>=sh['both']
        stage='Both detectors' if hit else 'Through opening' if through else 'Free flight' if sh['release'] and t>=sh['release']['t'] else 'Driving belts' if t>=sh['enableTime'] else 'Waiting for voltage reserve'
        add(7+t,440,185,125,90,piece(row['q'][3:],(np.array(row['q'][:3])-[1.43,5.55,0])*1000),f'{stage} · {row["busV"]:.2f} V')
    c['results']=[dict(pair=['Chambered NOTE','Speaker'],status='sampled' if sh['passShot'] else 'fail',witness=dict(reason=f"Full opening passage: {bool(sh['opening'])}. Paired detector time after scoring request: {sh['both']}. Minimum bus {sh['minBusV']:.3f} V."))]
    d['cases'].append(c)
    out.append(dict(segments=nn,held=tr['carryResult'],endFlapDeg=math.degrees(tr['endFlapRad']),gapMm=tr['flapClearanceMm'],handoffPositionAndVelocityChecked=True,score=sh['passShot'],minBusV=sh['minBusV'],opening=sh['opening'],pairS=sh['both'],source={q:sha(p/q) for q in [f'cam{nn}.json',f'fire{nn}.json']}))
res=dict(follower=sha(p/'fit.json'),response=sha(p/'response.json'),sweep=read(p/'sweep.json'),station=out,scope='Single floor fixture 0.44 m from the firing station. Travel between field marks, actual carpet calibration, loaded moving chassis and climb approach remain open.')
(p/'station-result.json').write_text(json.dumps(res,indent=2));d['receipt']=sha(p/'station-result.json')
d['narrativeReview']['objectives'][0]['basis']='Two passive floor followers maintained the tested pickup band across 72 parameter combinations. Actual carpet, wheel wear and arm loading need calibration.'
d['narrativeReview']['objectives'][1]['basis']='One free floor NOTE was picked up, raised and handed to the field solver at one test station. The six-piece campaign has yet to be run.'
d['narrativeReview']['objectives'][2]['basis']='The floor NOTE stayed in the cassette through the commanded swing and slide at both resolutions. Traversal under chassis acceleration and yaw is open.'
dest=www/'data.json';assert read(dest)['step'] in [f['source'],f['step']]
file=p/'review-data.json';file.write_text(json.dumps(d,separators=(',',':')),encoding='utf8')
tmp=dest.with_suffix('.next');tmp.write_bytes(file.read_bytes());tmp.replace(dest)
print([(k['segments'],k['score'],k['pairS']) for k in out],d['inspection'])

