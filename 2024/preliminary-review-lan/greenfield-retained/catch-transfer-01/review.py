"""Add two comparison replays to the existing LAN package. Geometry stays on Profile A."""
from pathlib import Path
import copy,json,math,hashlib,numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
p=Path(__file__).resolve().parent;www=p.parent.parent/'www';f=www/'data.json'
def read(f):return json.loads(f.read_bytes())
def sha(f):return hashlib.sha256(f.read_bytes()).hexdigest()
initial=sha(f);d=read(f)
assert d['defaultPickupBlade']['default']=='A' and any(c['key']=='trial-brake' for c in d['cases'])
def mat(deg,x,y,z):
    m=np.eye(4);m[:3,:3]=Rotation.from_euler('y',-deg,degrees=True).as_matrix();m[:3,3]=[x,y,z];return m
def flat(m):return np.round(m[:3].flatten(),9).tolist()
ref=next(c for c in d['cases'] if c['key']=='trial-brake');park=mat(125,185,0,501);h=mat(0,457.5,0,96)
groups={}
for group in ['frame','arm','flap']:
    pts=np.vstack([v['v'] for n,v in d['bodies'].items() if n not in ref['removed'] and ref['rig'].get(n,'frame')==group])
    groups[group]=pts[ConvexHull(pts).vertices]
cases=[];rows={}
for name in ['nominal','forward','aft','forward-refined','unseated','limited']:
    path=p/(name+'.json');r=read(path);rows[name]={k:v for k,v in r.items() if k!='trace'}
    rows[name]['sha256']=sha(path)
    if name not in ('forward-refined','unseated'):continue
    c=copy.deepcopy(ref);c.update(key='trial-catch' if name=='forward-refined' else 'trial-catch-open',
        id='Trial · catch takes the load' if name=='forward-refined' else 'Trial · catch held open',
        basis='Forward CG +40 mm. Passive catch modeled by a one-way compliant angular stop. Hook closures are ideal; catch teeth, supports and seat sensor are conceptual. Chamber empty. Same torque ramp and forced power cut for the seated and held-open cases.',
        time=[],positions=[],measure=[],chain=[],series={g:[] for g in ref['series']},results=[])
    floor=1e5
    for row in r['trace']:
        t=row['t'];q=row['q'];a=row['angle']
        base=np.eye(4);base[:3,:3]=Rotation.from_quat([*q[1:],q[0]]).as_matrix();base[:3,3]=np.array(row['p'])*1000
        rot=base@mat(a,185,0,501);cam=90*np.clip((a-102)/18,0,1)
        poses=dict(frame=base,arm=rot@np.linalg.inv(park),flap=rot@h@mat(cam,0,0,0)@np.linalg.inv(h)@np.linalg.inv(park))
        z=min((pts@poses[k][2,:3]+poses[k][2,3]).min() for k,pts in groups.items())
        if t>=4:floor=min(floor,z)
        for k in c['series']:c['series'][k].append(flat(poses.get(k,np.eye(4))))
        c['time'].append(t);c['positions'].append(ref['positions'][0]);c['chain'].append(np.round(np.array(row['chain'])*1000,3).tolist())
        stage='Motor disabled' if t>=4 else 'Seat confirmed' if row['confirmed'] else 'Transferring load' if row['catchNm']>0 else 'Hoisting'
        c['measure'].append(f"{stage} · catch {row['catchNm']:.0f} N·m · motor {row['torqueNm']:.0f} N·m · {row['floorN']:.0f} N floor")
    rows[name]['minimumRenderedStockAt25msMm']=floor
    c['floorCheckMm']=floor
    c['results'].append(dict(pair=['Pivot torque shaft','Chassis'],status='sampled' if r['passed'] else 'fail',witness=dict(reason=f"Five seconds with zero motor torque; pivot drift {r['measuredDriftDeg']:.2f}°, peak floor reaction {r['maxFloorN']:.0f} N.")))
    cases.append(c)
receipt=p/'results.json';receipt.write_text(json.dumps(dict(assembly=d['step'],trialSource=sha(p/'trial.py'),cases=rows),indent=2))
d['cases']=[c for c in d['cases'] if c['key'] not in ('trial-catch','trial-catch-open')]+cases
d['catchTrialReceipt']=sha(receipt);d['focus']='Profile A remains the default. The climb catch held in the load-transfer trial. Catch packaging, load path and reliable engagement remain open.'
assert sha(f)==initial
payload=json.dumps(d,separators=(',',':')).encode();tmp=f.with_suffix('.next');tmp.write_bytes(payload);tmp.replace(f)
print([(k,round(r['measuredDriftDeg'],3),r['confirmAt'],round(r['peakCatchNm'],2),r['passed']) for k,r in rows.items()]);print('rendered clearance:',rows['forward-refined']['minimumRenderedStockAt25msMm'])

