"""Measure the changed slide trajectory against the complete saved preliminary stock."""
from pathlib import Path
import json,hashlib,sys,math,numpy as np
p=Path(__file__).resolve().parent;root=p.parent
sys.path.insert(0,str(root/'assembly-recheck-01'));from sweep import load_solids,name,place,mat,bounds,common
trajectory=np.load(p/'trajectory.npz');times=trajectory['t']
from stroke import scheduled
def read(f):return json.loads(f.read_bytes())
def sha(f):return hashlib.sha256(f.read_bytes()).hexdigest()
doc=read(root.parent/'www/data.json');g=doc['concepts']['guided'];f=doc['concepts']['field'];service=next(c for c in g['cases'] if c['key']=='guided-service');fp=root/'guided-hoist-01/service-locked-robot.step'
assert sha(fp)==g['step']
R={name(n):s for n,s in load_solids(fp)[0].items()}
geom={**doc['bodies'],**f['add'],**g['add']}
assert len(R)==232
pose0=mat(125,185,0,501);ip=np.linalg.inv(pose0);eye=np.eye(4)
def role(n):
    if n in ['Battery and terminal case','Pack lead','Distribution lead']:return 'frame'
    if n=='Battery service pin':return 'slide'
    return service['rig'][n]
names=[n for n in R if n!='Chambered NOTE']
roles={n:role(n) for n in names};fixed=[n for n in names if roles[n]=='frame'];moving=[n for n in names if roles[n] in ('arm','slide','flap')]
assert len(fixed)+len(moving)==len(names),set(roles.values())
F=np.array([bounds(R[n]) for n in fixed]);offenders=[];errors=[];rows=[];worstForward=[0,None];seen=set();stepcount=0;originalBounds={n:bounds(R[n]) for n in names}
src=read(root/'lateral-station-01/dev-middle-48.json');a=src['pitchDegrees'];b=122;x0=read(root/'pickup-lip/layout.json')['pickPivot'][0];xt=trajectory['x'];assert a==trajectory['start'] and b==trajectory['end'] and x0==trajectory['initial']
def matrices(q,x):
    P=mat(q,x,0,501);flapAngle=90*np.clip((q-102)/18,0,1)
    return dict(arm=P@ip,slide=mat(0,x-185,0,0),flap=P@mat(0,457.5,0,96)@mat(flapAngle,0,0,0)@mat(0,-457.5,0,-96)@ip)
def bb(n,tr):
    B=np.array(originalBounds[n]);center=tr[:3,:3]@((B[:3]+B[3:])/2)+tr[:3,3];half=abs(tr[:3,:3])@((B[3:]-B[:3])/2);return np.r_[center-half,center+half]
def output(done):
    report=dict(step=g['step'],path=sha(root/'loaded-actuation-01/smooth.py'),operatingBodies=len(names),stationary=len(fixed),moving=len(moving),sampledStates=len(rows),narrow=len(seen),samples=rows,positive=offenders,unresolved=errors,finished=done,scope='All stationary-to-moving pairs at commissioned states, including flap on its intended cam path. Same-group and moving-to-moving relationships are unchanged by this chassis-X translation. Feeder contacts and freely rotating flap require dynamic replay. No interpolation guarantee for close stock between measured states.')
    (p/'clearance.json').write_text(json.dumps(report,indent=2))
for t in np.unique(np.r_[np.linspace(0,3.4,87),np.arange(1.77,2.,.0075)]):
    u=t/3.4;blend=u*u*(3-2*u);q=a+(b-a)*blend;x=float(np.interp(t,times,xt))*1000;baseX=float(scheduled(q)+(x0-scheduled(a))*(1-blend));M=matrices(q,x);M0=matrices(q,baseX)
    count=0
    for n in moving:
        tr=M[roles[n]];B=bb(n,tr);ix=np.flatnonzero(np.all((F[:,3:]+1e-5>=B[:3])&(B[3:]+1e-5>=F[:,:3]),axis=1))
        s=place(R[n],tr) if len(ix) else None
        for j in ix:
            other=fixed[j];key=(n,other,tuple(np.round(tr[:3].ravel(),8)))
            if key in seen:continue
            seen.add(key);count+=1
            try:
                v=common(s,R[other])
                if v>.01:
                    old=common(place(R[n],M0[roles[n]]),R[other]);offenders.append(dict(degrees=q,deltaMm=x-baseX,moving=n,fixed=other,commonMm3=v,originalMm3=old))
            except Exception as e:errors.append([q,n,other,str(e)])
    rows.append([float(t*4.8/3.4),q,x,x-baseX,count])
    if len(rows)%10==0:output(False)
output(True);print(len(rows),len(seen),len(offenders),len(errors))

