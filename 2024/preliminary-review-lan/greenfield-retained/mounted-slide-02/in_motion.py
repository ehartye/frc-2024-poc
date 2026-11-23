"""Test the removable slide cartridge against moving stock and the straight battery extraction volume."""
from pathlib import Path
import numpy as np,json,sys,hashlib
p=Path(__file__).resolve().parent;root=p.parent
sys.path.insert(0,str(root/'assembly-recheck-01'))
from sweep import load_solids,name,place,mat,bounds,common,distance
from layout import cyl,X,box
from stroke import scheduled
def read(f):return json.loads(f.read_bytes())
def sha(f):return hashlib.sha256(f.read_bytes()).hexdigest()
def get(f):return {name(k):s for k,s in load_solids(f)[0].items()}
fx=p/'slide-cartridge.step';R=get(root/'guided-hoist-01/service-locked-robot.step');H=get(fx);motorName=next(k for k in H if k in ('Purchased slide motor','SOLID'));motorBox=bounds(H.pop(motorName));assert 244<motorBox[0]<246 and 299<motorBox[3]<302,motorBox
H.update({'can envelope':cyl((256,275,428),X,17.5,44.5),'terminal envelope':cyl((282,275,428),X,np.hypot(25.5,25.5),18.5),'pilot':cyl((251.75,275,428),X,6.5,4.25),'stem':cyl((244.6,275,428),X,1.585,6.9)})
serviceparts=get(p/'drive-service.step')
del H['Trial coupling blank'];H.update(serviceparts)
H['rotating clamp full circumference']=cyl((233.5,275,428),X,np.hypot(6.05+2.25,4.50),18)
H['actual lead exits keepout']=cyl((259.2,275,428),X,30,3.6)
d=read(root.parent/'www/data.json')['concepts']['guided'];rig=next(c['rig'] for c in d['cases'] if c['key']=='guided-service')
moving={n:s for n,s in R.items() if rig.get(n) in ['arm','slide','flap'] and n not in ['Battery and terminal case','Distribution lead','Pack lead','Chambered NOTE']}
B={n:np.array(bounds(s)) for n,s in moving.items()};fn=list(H);F=np.array([bounds(H[n]) for n in fn])
def bounded(b,M):
    center=M[:3,:3]@((b[:3]+b[3:])/2)+M[:3,3];h=abs(M[:3,:3])@((b[3:]-b[:3])/2)
    return np.r_[center-h,center+h]
ip=np.linalg.inv(mat(125,185,0,501));poses=[(a,x,'bounded lift and reset') for _,a,x,_,_ in read(root/'bounded-lift-01/clearance.json')['samples']]
path=np.load(root/'bounded-lift-01/trajectory.npz');fitClock=np.linspace(0.,3.4,1001);u=fitClock/3.4
angles=path['start']+(path['end']-path['start'])*u*u*(3-2*u)
xs=1000*np.interp(fitClock,path['t'],path['x'])
poses=[(float(a),float(x),'bounded lift and reset') for a,x in zip(angles,xs)]
poses.extend((a,scheduled(a),mode) for mode,angles in [('service',[-40]),('endgame swing',np.arange(122,165.01,1))] for a in angles)
hits=[];tests=0;minBound=1e6; intervalMin={};prevState={};maxSlideExcursion=np.max(abs(np.diff(path["x"])/np.diff(path["t"])))*1000*.0034
for q,x,mode in poses:
    A=mat(q,x,0,501);f=np.clip((q-102)/18,0,1)*90
    M={'arm':A@ip,'slide':mat(0,x-185,0,0),'flap':A@mat(0,457.5,0,96)@mat(f,0,0,0)@mat(0,-457.5,0,-96)@ip}
    state={}
    for n,s in moving.items():
        T=M[rig[n]];b=bounded(B[n],T)
        ds=np.linalg.norm(np.maximum(0.,np.maximum(F[:,:3]-b[3:],b[:3]-F[:,3:])),axis=1)
        lb=float(min(ds));minBound=min(minBound,lb)
        state[n]=lb
        if mode in prevState:
            pq,px,dist=prevState[mode];theta=abs(np.radians(q-pq));dx=maxSlideExcursion if mode=="bounded lift and reset" else abs(x-px)
            # Every vertex is within the farthest corner radius of its parked box to the pivot.
            end=np.maximum(abs(B[n][:3]-[185,0,501]),abs(B[n][3:]-[185,0,501]))
            movement=dx if rig[n]=='slide' else dx+theta*np.hypot(end[0],end[2])
            if rig[n]=='flap':
                prevFlap=np.clip((pq-102)/18,0,1)*90
                movement+=.1*1000*abs(np.radians(f-prevFlap))+200*theta
            # Each endpoint yields a bound across the interval; use the stronger one.
            intervalMin[mode]=min(intervalMin.get(mode,1e6),max(lb,dist[n])-movement)

        for j in np.flatnonzero(ds<.001):
            tests+=1;v=common(place(s,T),H[fn[j]])
            if v>.001:hits.append([mode,q,n,fn[j],v])
    prevState[mode]=(q,x,state)
b=bounds(R['Battery and terminal case']);sweep=box(*b[:3],b[3]-b[0],b[4]-b[1],b[5]-b[2]+370)
gap=min(distance(s,sweep) for s in H.values())
r=dict(fixture=sha(fx),robot=d['step'],testedPoses=len(poses),moving=len(moving),movingNames=sorted(moving),fixed=len(H),narrow=tests,hits=hits,smallestSampledBoxGapMm=minBound,monotoneIntervalConservativeGapMm=intervalMin,batteryStraightPullCaseGapMm=gap,scope='Sampled rigid mechanisms in mounted, released-pickup, service and climb poses. Battery case and first 45 mm cartridge withdrawal use continuous envelopes. Loaded deflection, cable loops and rotating clamp hardware remain to be checked.')
(p/'motion.json').write_text(json.dumps(r,indent=2));print({k:v for k,v in r.items() if k not in ['movingNames','hits']},hits[:5])

