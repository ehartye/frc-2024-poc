"""Paired speed-command screen on identical incoming cassette, ring, field and motor states."""
import sys,subprocess
import numpy as np
from concurrent.futures import ThreadPoolExecutor
from make_generator import make,read,sha,p,old
from six_field import publish
from speaker_score_observer import opening_passage,beam_samples,shadow_margin

def trial(tag,v):
    n=96;dt=.00025;duration=9
    g,e,s=make(tag,n,duration+.015)
    assert e['setpoint']==s['targetSpeed']==6.88
    e['setpoint']=v
    target=e['target'];corners={i['key']:np.array(i['caseEndCornersM']) for i in target['sensorCaseEnds']}
    beams=[beam_samples(corners[a],corners[b]) for a,b in target['beamPairs']]
    a=next(g);rows=[];opening=both=None;minBus=a.bus;bounds=[];initial=a.q.copy()
    assert np.max(abs(initial-s['initialState']))<1e-9
    for j in range(round(duration/dt)+1):
        if a.t<=1.6:rows.append(dict(t=a.t,q=a.q.copy()))
        if j==round(1.6/dt):opening=opening_passage(rows)
        if a.t>1.6 and opening and both is None and all(shadow_margin(a.q,b)>0 for b in beams):both=a.t
        minBus=min(minBus,a.bus)
        if j%200==0:bounds.append([a.t,a.q[:3].tolist()])
        final=a;a=g.send(None)
    g.close()
    assert final.release is not None
    assert final.release['q']!=s['release']['q'] or v==6.88
    assert both is None or both>opening['fullyThroughByS']
    out=dict(tag=tag,speed=v,source=sha(old/f'split-tail_.25_0-{tag}-{n}-6.88.json'),originalSpeed=s['targetSpeed'],originalOpening=s['opening'],originalBeamTime=s['both'],incomingQ=initial.tolist(),release=final.release,opening=opening,both=both,minimumBusV=minBus,finalPose=final.q.tolist(),track=bounds,
        scope='Matched incoming field and robot state. Changing shooter command before feed recomputes roller acceleration, release and field response. Single-shot sensitivity only.')
    publish(p/f'speed-{tag}-{v:.2f}.json',out)
    print(tag,v,both,minBus,opening,flush=True)
if __name__=='__main__':
    if len(sys.argv)>1:trial(sys.argv[1],float(sys.argv[2]))
    else:
        # One original miss, the same piece in a passing run, and the tight-margin sixth shot with a late counter.
        tasks=[(id,v) for id in ('R02-inner','R01-inner','R03-center-low') for v in (6.80,6.96)]
        def launch(pair):subprocess.run([sys.executable,__file__,pair[0],str(pair[1])],check=True)
        with ThreadPoolExecutor(max_workers=2) as pool:list(pool.map(launch,tasks))

