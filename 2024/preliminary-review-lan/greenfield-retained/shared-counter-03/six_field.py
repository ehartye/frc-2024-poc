"""Integrate all six finite pieces in the same speaker, retaining every earlier NOTE."""
from pathlib import Path
import itertools
import math
import importlib
import sys
import json
import hashlib
from types import SimpleNamespace
import numpy as np
from make_generator import make, read, sha, old, p

sys.path[:0] = [str(p.parent/'counter-drive-01'),
    'C:/Users/ehart/Documents/FRC Native/2024/six-speaker-20261118/field-query-libs',
    'C:/Users/ehart/repos/frc-2024-poc/sim/pressure']
import run_speaker_field as field
import coupled_collection as contact
from speaker_score_observer import beam_samples, shadow_margin, opening_passage

def clear_ray(states, line):
    # A surviving ray must miss every torus simultaneously, including the space between its sample sites.
    clear = np.ones(line.shape[1], dtype=bool)
    for a in states:
        if abs(a.center[0]-line[0,0,0])>.25:
            continue
        rel = line-a.center
        z = rel@a.rotation[:,2]
        radial = np.sqrt(np.maximum(0.,np.sum(rel*rel,axis=2)-z*z))
        distance = np.hypot(radial-.1524,z)
        clear &= distance.min(axis=0)>.0254+.5*np.linalg.norm(line[1]-line[0],axis=1)+1.e-7
    return bool(clear.any())

def ray_grid(a,b):
    ends=np.array([sorted(v.tolist(),key=lambda q:(q[0],q[1])) for v in (a,b)])
    mid=ends.mean(axis=1,keepdims=True)
    face=np.concatenate((mid,.8*ends+.2*mid),axis=1)
    t=np.linspace(0,1,97)[:,None,None]
    return (1-t)*face[0]+t*face[1]

def publish(path, data):
    tmp=path.with_suffix('.tmp')
    tmp.write_text(json.dumps(data,allow_nan=False,separators=(',',':')))
    tmp.replace(path)

def run(name,n):
    dt=.024/n
    rf=old/f'run-{name}-{n}.json'
    parts=read(rf)['parts']
    assert len(parts)==6 and len(set(x['id'] for x in parts))==6
    first=parts[0]['shotStart']
    starts=[round((x['shotStart']-first)/dt) for x in parts]
    steps=math.floor((120-first)/dt)
    params=SimpleNamespace(stiffness=1.2e6,damping=450.,friction=.45)
    generators=[];sources=[];phase=[];openings={};counts={};touch={};edge=[];poseError=[0.]*6
    released=set();touched=set();states=[];tracks=[];armed=True;ambiguous=[];trace=[]
    result=p/f'six-shared-{name}-{n}.json'
    for part,j in zip(parts,starts):
        importlib.reload(field)
        gen,env,s=make(f"{name}-{part['id']}",n,(steps-j+10)*dt)
        assert part['shot']==sha(old/f"split-tail_.25_0-{name}-{part['id']}-{n}-6.88.json")
        generators.append(gen);sources.append(s);tracks.append([])
    corners={x['key']:np.array(x['caseEndCornersM']) for x in env['target']['sensorCaseEnds']}
    beams=[beam_samples(corners[a],corners[b]) for a,b in env['target']['beamPairs']]
    rays=[ray_grid(corners[a],corners[b]) for a,b in env['target']['beamPairs']]
    def report(t,done):
        return dict(start=name,resolution=n,terminal=done,clock=t,inputRun=sha(rf),
            shotInputs=[x['shot'] for x in parts],integrationStepS=dt,
            startRoundingErrorsS=[first+j*dt-part['shotStart'] for j,part in zip(starts,parts)],
            pairCode=sha(Path(contact.__file__)),openings=openings,counts=counts,beamTransitions=edge,
            ambiguousOccupancy=ambiguous,contacts=touch,sampledPoseErrorBeforeFirstContact=poseError,
            distinctWithin120=len(counts),released=sorted(released),
            scope='Original recorded launch states and release clocks. All admitted NOTES retained in the field; symmetric pair forces and field surface contact. Robot driving and other partners are outside this common speaker model.')
    try:
        for j in range(steps+1):
            t=first+j*dt
            if len(states)<len(starts) and j==starts[len(states)]:
                states.append(next(generators[len(states)]))
            force=[None]*len(states)
            for a,b in itertools.combinations(range(len(states)),2):
                if np.dot(states[a].center-states[b].center,states[a].center-states[b].center)>.355601**2:continue
                f,depth=contact.pair_wrenches(states[a],states[b],params)
                if depth>0:
                    touched.update((a,b))
                    key=parts[a]['id']+'/'+parts[b]['id']
                    v=touch.setdefault(key,dict(firstS=t,peakMm=0.,peakN=0.))
                    v.update(lastS=t,peakMm=max(v['peakMm'],1000*depth),peakN=max(v['peakN'],float(np.linalg.norm(f[0,:3]))))
                    for i,k in ((a,0),(b,1)):force[i]=f[k] if force[i] is None else force[i]+f[k]
            for i,a in enumerate(states):
                k=parts[i]['id'];src=sources[i]
                if i not in touched and a.t<6.998 and (j-starts[i])%round(.012/dt)==0:
                    idx=(j-starts[i])//round(.012/dt)
                    err=float(np.max(abs(src['trace'][idx]['q']-a.q)))
                    poseError[i]=max(err,poseError[i]);assert err<1.e-8,(k,err,idx)
                if a.release is not None and k not in released:
                    assert src['release']==a.release
                    released.add(k)
                if k not in openings and a.t<2.05:
                    tracks[i].append(dict(q=a.q.copy(),t=a.t))
                    if a.t>=src['opening']['fullyThroughByS']+.005:
                        actual=opening_passage(tracks[i]);openings[k]=actual
                        tracks[i].clear()
            # Mark an occupied transition once; a fresh event requires an intervening proof of two clear paths.
            if armed:
                covered=[i for i,a in enumerate(states) if all(shadow_margin(a.q,h)>0 for h in beams)]
                if covered:
                    ids=[parts[i]['id'] for i in covered];edge.append(dict(t=t,covered=ids));armed=False
                    if len(ids)==1 and ids[0] not in counts and openings.get(ids[0]):
                        counts[ids[0]]=t
                    else:ambiguous.append(dict(t=t,ids=ids))
            elif all(clear_ray(states,line) for line in rays):
                armed=True;edge.append(dict(t=t,bothClear=True))
            if j%round(.05/dt)==0:
                trace.append([round(t,6),[a.q.tolist() for a in states]])
            if j%round(1/dt)==0 or j==steps:
                publish(result.with_suffix('.progress.json'),report(t,False))
            if j<steps:
                states=[generators[i].send(f) for i,f in enumerate(force)]
                if j%2000==1999:
                    for gen in generators[:len(states)]:
                        # Source row history is only consumed by the original postprocessor, removed in the adapter.
                        history=gen.gi_frame.f_locals.get('rows')
                        if history is not None:history.clear()
        resultData=report(t,True)
        trajectory=result.with_suffix('.trajectory.json')
        publish(trajectory,trace);resultData['trajectory']=sha(trajectory);publish(result,resultData)
    finally:
        for gen in generators:gen.close()

if __name__=='__main__':
    run(sys.argv[1],int(sys.argv[2]))

