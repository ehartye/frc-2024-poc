"""Check the occupied space of each untouched floor NOTE until its own pickup leg starts."""
from pathlib import Path
import json
import math
import hashlib
import numpy as np

p=Path(__file__).resolve().parent
def read(f):return json.loads(f.read_bytes())
def sha(f):return hashlib.sha256(f.read_bytes()).hexdigest()
fn=p.parent.parent/'www/data.json';doc=read(fn);g=doc['concepts']['guided'];field=doc['concepts']['field'];svc=doc['slideService']
stock={k:v for k,v in doc['bodies'].items() if k not in field['remove']};stock.update(field['add']);stock.update(g['add']);stock.update(svc['add'])
planfile=p.parent/'counted-reset-power-02/plan.json';plan=read(planfile)
proof=[]
for n in [48,96]:
    runfile=planfile.parent/f'run-dev-{n}.json';run=read(runfile)
    c=next(x for x in g['cases'] if x['key']==f'continuous-cycle-{n}')
    rigs={**c['rig'],**{k:'frame' for k in svc['add']}}
    omit=set(c['removed']+g['fixtureOnlyNames']+['Chambered NOTE'])
    parts=[k for k in stock if k not in omit and k in rigs and rigs[k]!='piece']
    xy=np.array(c['positions']);tt=np.array(c['time'])
    for leg,part in zip(plan['plan'],run['parts']):
        carry=read(planfile.parent/f"dev-{leg['id']}-{n}.json")
        ends=part['shotStart']-carry['finish'];use=(tt>=part['depart']-.000001)&(tt<=ends+.000001)
        ctr=np.c_[np.array(leg['mark'])*1000-xy[use],np.full(use.sum(),25.4)]
        best=(math.inf,None,None);swept=(math.inf,None,None);intervalFindings=[]
        for group in sorted(set(rigs[k] for k in parts)):
            T=np.array(c['series'][group])[use].reshape(-1,3,4)
            local=np.einsum('ti,tij->tj',ctr-T[:,:,3],T[:,:,:3])
            R=T[:,:,:3]
            relative=np.einsum('tji,tjk->tik',R[:-1],R[1:])
            theta=np.arccos(np.clip((relative.trace(axis1=1,axis2=2)-1)/2,-1,1))
            worldOrigins=T[:,:,3]+np.c_[xy[use],np.zeros(use.sum())]
            displacement=np.linalg.norm(np.diff(worldOrigins,axis=0),axis=1)
            for name in (k for k in parts if rigs[k]==group):
                v=np.array(stock[name]['v']);lo=v.min(0);hi=v.max(0)
                margin=np.linalg.norm(np.maximum(0,np.maximum(lo-local,local-hi)),axis=1)-177.8
                i=int(margin.argmin())
                if margin[i]<best[0]:best=(float(margin[i]),name,float(tt[use][i]))
                # For the displayed linear translation + shortest rigid rotation, each point is at most
                # half an interval of travel away from its nearer endpoint. Use the farthest possible vertex.
                bound=np.minimum(margin[:-1],margin[1:])-.5*(displacement+theta*np.linalg.norm(v,axis=1).max())-.002
                i=int(bound.argmin())
                if bound[i]<swept[0]:swept=(float(bound[i]),name,float(tt[use][i]))
                if bound[i]<=0:intervalFindings.append([name,float(bound[i]),float(tt[use][i])])
        proof.append(dict(meshResolution=n,id=leg['id'],ownApproachEndsS=ends,parts=len(parts),poses=int(use.sum()),minimumSampleBoundMm=best,minimumBetweenSampleBoundMm=swept,unresolved=intervalFindings,sourceRun=sha(runfile)))
out=dict(route=sha(planfile),webPayload=sha(fn),scope='During each intended pickup approach up to initialization of the free NOTE. Circumsphere around each stationary free NOTE against each part-local bounding box. Between-frame enclosure assumes linear translation and shortest rigid rotation per displayed part. No physical calibration implied.',proof=proof)
(p/'approach-floor-pieces.json').write_text(json.dumps(out,indent=2))
print([(r['meshResolution'],r['id'], round(r['minimumBetweenSampleBoundMm'][0],2),len(r['unresolved'])) for r in proof])

