"""Assemble this private issue on the preceding review; update the LAN data atomically."""
from pathlib import Path
import json,hashlib,numpy as np
from gap import mat,quat,clearance
p=Path(__file__).resolve().parent;parent=p.parent/'mission-physics';www=p.parent.parent/'www'
def read(f):return json.loads(f.read_bytes())
d=read(parent/'mission-data.json');g=read(p/'layout.json');sweep=read(p/'sweep.json')
assert d['step']==g['parentStep']
def M(f):return np.vstack((np.asarray(f).reshape(3,4),[0,0,0,1]))
def flat(m):return m[:3].round(9).flatten().tolist()
I=np.eye(4);park=mat(125,185,0,501);seat=park@mat(0,215,0,50.4);old=mat(0,399.5,0,84);new=mat(0,*g['flapAxleLocalMm']);note='Chambered NOTE';flap='Mouth retention flap';rep='Rear piece stop'
def at(c,n,i):
    ser=c.get('series',{}).get(c.get('rig',{}).get(n));f=(ser or c['frames'].get(n))
    return M(f[i]) if f else I
def audit(c,h):
    if note in c['removed']:return []
    return [clearance(at(c,note,i)@seat,at(c,flap,i)@park@h).tolist() for i in range(len(c['positions']))]
original=next(c for c in d['cases'] if c['key']=='intake')
oldgap=np.min(audit(original,old),axis=0)
copies=['Replaceable pickup lip','Pickup lip crossbar','Cassette side beam extension · left','Cassette side beam extension · right']
tr=read(p/'trial-1-1-96-0.36-0.json');coarse=read(p/'trial-1-1-48-0.36-0.json');off=read(p/'trial-1-0-48-0.36-0.json')
assert tr['seated'] and coarse['seated'] and not off['seated']
for n,mesh in g['hingeChanges'].items():d['bodies'][n].update(mesh)
d['bodies'].update(g['parts'])
gv=np.vstack([g['parts'][n]['v'] for n in copies]);inv=np.linalg.inv(park);gv=gv@inv[:3,:3].T+inv[:3,3]
d['bodies'][flap]['purpose']='The arriving NOTE pushes the spring-loaded flap inward. Once its trailing rim clears, the flap returns behind it. The discharge cam folds it outward at the firing position.'
d['bodies'][flap]['rationale']='The moved hinge clears the incoming rim and leaves room for the full flap to close after seating.'
# Apply the slide correction to arm and truck together without moving the rest of the chassis.
import math
for c in d['cases']:
    if c['key']=='intake':continue
    if 'rig' in c:c['rig'].update({n:'arm' for n in copies})
    ff=[];mts=[];N=len(c['positions'])
    for i in range(N):
        arm=at(c,rep,i);f=at(c,flap,i);world=arm@park
        Q=np.linalg.inv(world@old)@f@park@old
        corrected=world@new@Q@np.linalg.inv(new)@np.linalg.inv(park)
        dx=0 if c['key']=='climb' else -max(0.,np.max((gv@world[:3,:3].T)[:,0])+world[0,3]-640.)
        ff.append(flat(mat(0,dx)@corrected));mts.append(flat(mat(0,dx)@arm))
        if dx:
            for n,fs in c['frames'].items():
                if n not in (flap,) and (n!=note or c['key']=='raise'):
                    fs[i]=flat(mat(0,dx)@M(fs[i]))
    if 'series' in c:c['series']['flap']=ff
    else:
        c['frames'][flap]=ff
        for n in copies:c['frames'][n]=mts
    if c['key']=='shot':
        r=read(p/'shot-96.json');assert r['both'] and not r['changedContacts'];c['basis']+=' Replayed with the relocated flap, axle and shovel in the contact model.'
    if c['key']=='climb':c['basis']+=' Inherited load case. Added stock clears the floor in the recorded hang; drive and brake capacity still require hardware checks.'
    c['results']=[r for r in c['results'] if 'Inherited path.' not in r.get('witness',{}).get('reason','')]
cs=original
cs.update(id='Floor pickup · lip, roller and passive flap',basis='Free NOTE on carpet. The chassis advances 360 mm at 0.25 m/s. The blunt lip feeds the nose; the hinged flap responds to contact, its spring and its stop. The trace uses a 0.25 ms time step and a 3 mm numerical contact margin on the flap hardware. Belt speeds are prescribed in this preliminary pickup test.',time=[],positions=[],measure=[],frames={},results=[],rig={},series={k:[] for k in ('arm','slide','piece','flap','frame')})
theta=g['pickAngle'];pickup=mat(theta,*g['pickPivot']);arm=pickup@np.linalg.inv(park)
# Identify slide parts from their commissioned world transformation on deployment.
slide={'Translating bearing block · left','Translating bearing block · right','Pivot bearing · left','Pivot bearing · right','Pivot motor','Pivot reducer','Travel nut and clevis','Service pin guide','Battery service pin'}
armMembers=d['cases'][0]['rig']
for n in d['bodies']:cs['rig'][n]='slide' if n in slide else armMembers.get(n,'frame')
cs['rig'][flap]='flap';cs['rig'][note]='piece'
for row in tr['trajectory']:
    t=row[0];pos=np.array(row[1:4])*1000;qs=row[4:8];deg=math.degrees(row[-1]);adv=min(t*.25,tr['approachM'])*1000
    cs['time'].append(t);cs['positions'].append([965.2+adv,5547.36])
    stage='NOTE seated · spring returns flap' if row[8]<.235 else 'Side belts drawing NOTE' if t>=tr['sideBeltTakeoverS'] else 'NOTE pushes flap inward' if abs(deg)>3 else 'Roller feeding NOTE' if t>=tr['firstContactS']['center belt'] else 'Approach floor NOTE'
    cs['measure'].append('%s · flap %.1f° · chassis travel %.0f mm'%(stage,abs(deg),adv))
    for k,pm in dict(arm=arm,slide=mat(0,g['pickPivot'][0]-185),frame=I,flap=pickup@new@mat(deg)@np.linalg.inv(new)@np.linalg.inv(park),piece=quat(qs,pos)@np.linalg.inv(seat)).items():cs['series'][k].append(flat(pm))
def result(c,pair,status,reason):c['results'].append(dict(pair=pair,status=status,witness=dict(reason=reason)))
result(cs,[flap,note],'sampled','Contact-driven hinge and free NOTE. Both centered refinements seated the ring. Fine trace checks 10,001 poses against the leaf, sleeve and axle independently.')
result(cs,['Replaceable pickup lip','Nose pickup roller'],'sampled','%.2f mm rigid gap to the nose roller. Matched powered runs also seat without the lip. Unpowered pickup fails.'%g['noseGapMm'])
result(cs,['Replaceable pickup lip','Carpet'],'open','3.0 mm nominal tip height. A further half degree nose-down reaches below the nominal floor. Carpet pile, wear, loaded flex and passive stop tolerances need a physical test.')
for e in d['power']:
    if e['stage']=='Intake':e['stage']=cs['id']
cs['status']='sampled';cs['narrow']=tr['checkedPoses'];cs['gap']=round(min(tr['minimumSolidClearanceMm']),2)
# Check the *shipped viewer transformations*, across all cases displaying the game piece.
auditRows={c['key']:np.min(rows,axis=0).tolist() for c in d['cases'] if (rows:=audit(c,new))}
d['issue']='Shared cassette · beam-mounted pickup lip';d['focus']='Loading uses the free NOTE and contact-driven flap. Remaining holds include carpet tolerance, blade attachment and suspended balance.'
d['step']=hashlib.sha256((p/'shared-cassette-lip.step').read_bytes()).hexdigest()
d['required'].append(['Pickup lip and load path',copies]);d['mass'].append(dict(id='Pickup lip, extended rail tips and attachments',kg=[.085,.17],members=copies))
for k in ('inspection','running'):d[k]=[round(v+w,4) for v,w in zip(d[k],[.085,.17])]
d['notes']+=[f"Shovel stock is nominally 3.0 mm above the flat carpet. Swing bumper clearance bound {g['bumperSweptLowerMm']:.1f} mm; front reach bound {g['interSampleFrontUpperMm']:.1f} mm against the 644.8 mm project envelope. Up to {g['maxAdditionalRetractionMm']:.1f} mm extra slide retraction. Complete truck support, fasteners, wear and loaded flex remain open.",'The shaft moved 58 mm toward the mouth and 12 mm above the old axis. The raised ends of both lid rails support it. The spring and stop are preliminary parameters.']
d['receipt']=hashlib.sha256((p/'layout.json').read_bytes()+(p/'trial-1-1-96-0.36-0.json').read_bytes()).hexdigest()
out=dict(previousViewerIntakeLowerMm=oldgap.tolist(),viewerLowerMm=auditRows,parent=d['step'],physics=tr['minimumSolidClearanceMm'],shape=['flap leaf','sleeve','axle'],frameCount=len(cs['time']))
(p/'viewer-check.json').write_text(json.dumps(out,indent=2));print(out,flush=True)
assert min(min(v) for v in auditRows.values())>0,'One of the shared review cases still clips the NOTE'
out=p/'review-data.json';out.write_text(json.dumps(d,separators=(',',':')),encoding='utf8')
if '--live' in __import__('sys').argv:
    import shutil
    dest=www/'data.json';before=read(dest);assert before['step'] in [g['parentStep'],d['step']]
    temp=dest.with_suffix('.next');shutil.copyfile(out,temp);temp.replace(dest)

