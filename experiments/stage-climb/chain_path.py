"""Broad-phase sweep of the analytical solo chain against the lowered R7 and R8."""
import math
import numpy as np


def screen_chain(first, added, screen, trajectory):
    rows=[r for r in first if r['name']!='Continuous bumper native display v2']
    # The open roof and both cheeks are the intended chain interface. All other
    # climber solids are included and translated by their own reopened joint.
    structural=[r for r in added if r['body'] not in ('Chain bearing roof','Retaining lip','Saddle spine')]
    boxes=np.array([r['bboxMm'] for r in rows]+[r['boundsMm'] for r in structural]).reshape(-1,2,3)
    titles=[r['name'] for r in rows]+[r['body'] for r in structural]
    motion=np.array(['fixed']*len(rows)+[r['component'] for r in structural])
    fit=screen['chainFit']
    half=fit['halfSpanMm']
    s=np.linspace(-half,half,161)
    solo=screen['stageChecks'][0]
    free=solo['unloadedLowMm']
    loaded=solo['loadedSeatMm']
    free_curve=free+fit['catenaryAmm']*(np.cosh(s/fit['catenaryAmm'])-1)
    loaded_curve=loaded+(fit['anchorApproxMm']-loaded)*abs(s)/half
    hits={}
    for pose in trajectory['samples']['engaged']:
        fraction=(free-pose['chainLowMm'])/(free-loaded)
        z=(1-fraction)*free_curve+fraction*loaded_curve
        line=np.column_stack((np.full_like(s,.5),s-30,z))
        x,y,z,w=pose['quaternionXYZW']
        K=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
        rot=np.eye(3)+2*w*K+2*K@K
        chain=(line-np.array(pose['originMm']))@rot
        local=boxes.copy()
        local[motion=='R8 sliding saddle',:,2]+=pose['slideMm']
        local[motion=='R8 transverse lock',:,0]+=pose['pinWithdrawnMm']
        # L-infinity 14 mm expansion covers the measured 26.67 mm wide chain.
        lo,hi=local[:,0]-14,local[:,1]+14
        # Reject disjoint body boxes before exact segment vs expanded box tests.
        ids=np.flatnonzero(np.all(hi>=chain.min(axis=0),axis=1)&np.all(lo<=chain.max(axis=0),axis=1))
        for i in ids:
            pts=chain[:-1]; delta=np.diff(chain,axis=0)
            parallel=np.abs(delta)<1e-10
            direction=np.where(parallel,1,delta)
            t0=(lo[i]-pts)/direction; t1=(hi[i]-pts)/direction
            start=np.where(parallel,-np.inf,np.minimum(t0,t1)).max(axis=1)
            end=np.where(parallel,np.inf,np.maximum(t0,t1)).min(axis=1)
            valid=(~(parallel & ((pts<lo[i])|(pts>hi[i]))).any(axis=1)) & (np.maximum(0,start)<=np.minimum(1,end))
            if np.any(valid):
                hits.setdefault(titles[i],[]).append(pose['tS'])
    return dict(sampleCount=len(trajectory['samples']['engaged']),segments=160,boxInflationMm=14,
        possibleContacts=[dict(body=k,firstS=min(v),lastS=max(v)) for k,v in hits.items()],
        scope='Sampled solo approach and engaged-lock trajectory at nominal CG. Intended saddle roof and cheeks excluded; retained R7 physical bumper constituents replace the cosmetic whole-bumper AABB. No dynamic contact or inter-sample proof.')
