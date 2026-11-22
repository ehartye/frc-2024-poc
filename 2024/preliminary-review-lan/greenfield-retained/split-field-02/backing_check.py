"""Screen the swept ring against each rigid belt backing. Never credit elastic travel beyond the actual substrate."""
from pathlib import Path
import json,numpy as np,sys,math
from scipy.spatial.transform import Rotation
p=Path(__file__).resolve().parent;root=p.parent;sys.path.insert(0,str(root/'follower-01'))
from stroke import scheduled
def read(f):return json.loads(f.read_bytes())
phi=np.arange(160)*math.tau/160;circle=152.4*np.column_stack([np.cos(phi),np.sin(phi),np.zeros(len(phi))])
def checkPose(positions,quat,angle,origin):
    Q=Rotation.from_quat(quat[:,[1,2,3,0]]);inv=Rotation.from_euler('y',angle[:,None],degrees=True).inv()
    # Columns of cassette's local-to-world rotation correspond to rotation about negative Y.
    rel=Rotation.from_euler('y',angle[:,None],degrees=True)
    center=rel.apply(positions-origin)
    RM=(rel*Q).as_matrix();points=np.einsum('bij,kj->bki',RM,circle)+center[:,None,:]
    out=[]
    for side in (-1,1):
        for lo,hi in [(97,352),(407,600)]:
            x,y,z=points.transpose(2,0,1);dx=np.maximum(lo-x,np.maximum(0.,x-hi))
            dr=np.hypot(dx,y-side*195.8)-18.;dz=np.maximum(32-z,np.maximum(0.,z-64))
            gap=np.hypot(np.maximum(0.,dr),dz)-25.4
            out.append(gap.min(axis=1))
    return np.min(out,axis=0)
def leg(fn):
    d=read(fn);t=np.array([v[0] for v in d['trajectory']]);pos=np.array([v[1:4] for v in d['trajectory']])*1000;Q=np.array([v[4:8] for v in d['trajectory']])
    dep=d['pitchDegrees'];u=np.clip((t-d['startLift'])/3.4,0,1);b=u*u*(3-2*u);q=dep+(125-dep)*b
    org=np.zeros_like(pos);org[:,0]=[scheduled(a)+(1-bl)*(169.3625892825837-scheduled(dep)) for a,bl in zip(q,b)];org[:,2]=501
    g=checkPose(pos,Q,q,org);j=g.argmin()
    return dict(file=fn.name,phase='pickup and carry',minimumSampleMm=float(g[j]),atS=float(t[j]),overlapSamples=int(np.sum(g<-.01)),samples=len(t))
def fire(fn):
    d=read(fn);trace=[x for x in d['trace'] if x['t']<1.5];t=np.array([v['t'] for v in trace]);a=np.array([v['q'] for v in trace]);org=np.tile(np.array(d['pivot'])*1000,(len(a),1));g=checkPose(a[:,:3]*1000,a[:,3:],np.full(len(a),125),org);j=g.argmin()
    return dict(file=fn.name,phase='firing',minimumSampleMm=float(g[j]),atS=float(t[j]),overlapSamples=int(np.sum(g<-.01)),samples=len(t))
rows=[]
for head in ('R01','R02','dev'):
    for fn in sorted(p.glob(head+'-*-*.json')):rows.append(leg(fn))
for fn in sorted(p.glob('split-ready-*.json')):rows.append(fire(fn))
out=dict(method='Distance from the 160-node ring centerline to the finite 32 mm-high, 18 mm-radius backing envelope, less the tube radius. Negative samples are finite rigid intrusions for the modeled circular tube; positive sampled clearances need angular and time refinement. Front/rear spans use the actual local datum.',rows=rows)
(p/'backing-screen.json').write_text(json.dumps(out,indent=2));print(len(rows),sorted(rows,key=lambda r:r['minimumSampleMm'])[:3])

