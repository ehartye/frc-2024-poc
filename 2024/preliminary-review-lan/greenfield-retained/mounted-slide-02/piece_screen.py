"""Conservative free-piece interval clearance to the entire proposed stationary cartridge."""
from pathlib import Path
import json,hashlib
import numpy as np
p=Path(__file__).resolve().parent;root=p.parent;mission=root/'counted-reset-power-02'
read=lambda f:json.loads(f.read_bytes())
def sha(f):return hashlib.sha256(f.read_bytes()).hexdigest()
mesh=read(p/'display-mesh.json');names=list(mesh)
B=np.array([[np.min(mesh[n]['v'],axis=0),np.max(mesh[n]['v'],axis=0)] for n in names])/1000
# Include the full body and cable terminals of the purchased motor inside a larger box.
B[names.index('Slide drive motor')]=[[.2444,.2380,.3910],[.301,.312,.465]]
def screen(t,x):
    g=np.maximum(0.,np.maximum(B[None,:,0,:]-x[:,None,:],x[:,None,:]-B[None,:,1,:]))
    gap=np.linalg.norm(g,axis=2)-.1778
    d=np.minimum(gap[:-1],gap[1:])-.5*12.0*np.diff(t)[:,None]
    i,j=np.unravel_index(np.argmin(d),d.shape)
    return dict(minGuaranteedM=float(d[i,j]),minimumSampledM=float(gap.min()),part=names[j],time=float(t[i]),largestChordMps=float(np.max(np.linalg.norm(np.diff(x,axis=0),axis=1)/np.diff(t))))
rows=[]
for n in [48,96]:
    run=read(mission/f'run-dev-{n}.json')
    assert run['terminal'] and run['passMission']
    for leg in run['parts']:
        cc=mission/f'dev-{leg["id"]}-{n}.json';ss=mission/f'split-tail_.25_0-dev-{leg["id"]}-{n}-6.88.json'
        assert sha(cc)==leg['carry'] and sha(ss)==leg['shot']
        c=read(cc);s=read(ss);rr=c['trajectory']
        t=np.array([v[0] for v in rr]);x=np.array([v[1:4] for v in rr])
        w=screen(t,x);rr=s['trace'];f=screen(np.array([v['t'] for v in rr]),np.array([v['q'][:3] for v in rr])-[1.39,5.871,0])
        rows.append(dict(resolution=n,piece=leg['id'],acquisitionAndCarry=w,shotAndCollection=f))
report=dict(serviceMesh=sha(p/'display-mesh.json'),results=rows,minimumM=min(u[k]['minGuaranteedM'] for u in rows for k in ('acquisitionAndCarry','shotAndCollection')),assumptions='Circumsphere with 177.8 mm radius. Robot-frame centers for carry and stationary field-frame centers for the outgoing NOTE. Each interval deducts half the interval times a 12 m/s relative speed bound. Motor box includes all motor terminals. This excludes other loose floor NOTES and post-shot robot departure.')
(p/'free-piece-clearance.json').write_text(json.dumps(report,indent=2));print(report['minimumM'],max(u[k]['largestChordMps'] for u in rows for k in ('acquisitionAndCarry','shotAndCollection')))

