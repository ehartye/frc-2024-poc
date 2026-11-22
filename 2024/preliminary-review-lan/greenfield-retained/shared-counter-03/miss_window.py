"""Retest the unchanged two frozen misses through the remaining mission time or a bounded 15 s field window."""
from make_generator import *
from concurrent.futures import ThreadPoolExecutor
import subprocess
def one(tag):
    n=96;run=read(old/f'run-{tag.split("-")[0]}-96.json');part=run['parts'][-1];sf=old/f'split-tail_.25_0-{tag}-96-6.88.json';initial=read(sf)
    dt=.00025;duration=min(15.,120-part['shotStart'])
    g,env,s=make(tag,n,duration+.01);a=next(g);target=env['target'];corners={r['key']:np.array(r['caseEndCornersM']) for r in target['sensorCaseEnds']};beams=[env['beam_samples'](corners[x],corners[y]) for x,y in target['beamPairs']]
    match=0.;first=None;last=None
    for j in range(round(duration/dt)+1):
        if j%48==0 and a.t<6.999:match=max(match,np.max(abs(a.q-initial['trace'][j//48]['q'])))
        if a.t>initial['opening']['fullyThroughByS'] and all(env['shadow_margin'](a.q,h)>=0 for h in beams) and first is None:first=a.t
        last=a;a=g.send(None)
    assert match<1.e-8 and last.release==s['release'];g.close()
    out=dict(tag=tag,originalShotSha=sha(sf),minimumOpeningMm=1000*s['opening']['minimumMarginM'],identicalSampledPoseError=match,elapsedAtStart=part['shotStart'],observedTo=duration,counterS=first,counterMissionClock=None if first is None else first+part['shotStart'],finalQ=last.q.tolist(),scope='Original release with extended field observation only. Any late credit requires the remaining robot route, deadline and prior counter occupancy to be reconciled.')
    (p/f'extended-{tag}.json').write_text(json.dumps(out,indent=2));print(out)
if len(sys.argv)>1:one(sys.argv[1])
else:
    def go(tag):
        with (p/f'extended-{tag}.log').open('w') as fo:subprocess.run([sys.executable,__file__,tag],check=True,stdout=fo,stderr=fo)
    with ThreadPoolExecutor(max_workers=2) as ex:list(ex.map(go,['R02-inner','R03-center-low']))

