"""Check the changed station legs with occupied part bounds instead of a whole-robot prism."""
from pathlib import Path
import numpy as np,math,json,sys,hashlib
p=Path(__file__).resolve().parent;root=p.parent
sys.path.insert(0,str(root/'endgame-route-01'));from access import items,ids,pose
def read(f):return json.loads(f.read_bytes())
def sha(f):return hashlib.sha256(f.read_bytes()).hexdigest()
plan=read(p/'station-checks.json');d=read(root.parent/'www/data.json');f=d['concepts']['field'];g=d['concepts']['guided'];c=next(c for c in f['cases'] if c['key']=='field-r08')
assert plan['placedStep']==g['step']
bodies={**d['bodies'],**f['add'],**g['add']}
names=sorted(set(bodies)-set(f['remove'])-set(c['removed'])-set(g['fixtureOnlyNames'])-{'Chambered NOTE'})
rig={n:c['rig'].get(n,'arm') for n in names}
bb0={n:np.array(bodies[n]['v']) for n in names}
# Translation per degree from the scheduled slide is <= 12 mm and a 1.3 m radius contributes <=23 mm/deg.
# Interpolation guard exceeds the between-sample displacement for a 0.25 degree pitch interval.
worst0=np.ones((len(names),3))*1e6;worst1=-worst0.copy()
for angle in np.linspace(-51,122,693):
    tr=pose(angle)
    for i,n in enumerate(names):
        h=tr[rig[n]];v=bb0[n]@h[:3,:3].T+h[:3,3];worst0[i]=np.minimum(worst0[i],v.min(0));worst1[i]=np.maximum(worst1[i],v.max(0))
# A loose flap during pickup sweeps an 80 mm disk in its own plane.
for n in names:
    if rig[n]=='flap':
        i=names.index(n);worst0[i]-=[80,0,80];worst1[i]+=[80,0,80]
out=[];field=[(n,b) for n,b in zip(ids,items) if n.startswith('FE-') or n=='working chain']
for i,leg in enumerate(plan['plan']):
    for label,path in [('out',leg['out'][:2]),('return',leg['loaded'][-2:])]:
        if i==0 and label=='out':continue
        obstacles=field+[(q['id'],[np.r_[np.array(q['mark'])*1000-177.8,0],np.r_[np.array(q['mark'])*1000+177.8,50.8]]) for q in plan['plan'][i+(label=='return'):]]
        A,B=np.array(path)*1000;A=np.r_[A,0];B=np.r_[B,0];V=B-A;bb=np.array([b for _,b in obstacles]);matches=[]
        for j,n in enumerate(names):
            # Both route segments have heading zero. Sweep each part across the whole line inclusively.
            lo=bb[:,0]-worst1[j]-6;hi=bb[:,1]-worst0[j]+6;t0=np.zeros(len(hi));t1=t0+1
            for k in (0,1,2):
                if abs(V[k])<1e-8:t0[(A[k]<lo[:,k])|(A[k]>hi[:,k])]=2
                else:
                    a=(lo[:,k]-A[k])/V[k];b=(hi[:,k]-A[k])/V[k];t0=np.maximum(t0,np.minimum(a,b));t1=np.minimum(t1,np.maximum(a,b))
            for idx in np.flatnonzero(t0<=t1):matches.append([n,obstacles[idx][0],round(float(t0[idx]),5),round(float(t1[idx]),5)])
        out.append(dict(leg=leg['id'],phase=label,matches=matches,subjects=len(names),fieldItems=len(obstacles)))
res=dict(plan=sha(p/'station-checks.json'),data=plan['placedStep'],check=out,terminal=not any(r['matches'] for r in out),scope='6 mm between-pose allowance. Entire angle interval -51..122 degrees allowed anywhere on each changed straight segment. Fixed field structure and dormant NOTES included; time-varying peers remain separate.')
(p/'individual-route.json').write_text(json.dumps(res,indent=2))
print([(r['leg'],r['phase'],len(r['matches']),r['matches'][:2]) for r in out])
if res['terminal']:(p/'plan.json').write_text(json.dumps(plan,indent=2))

