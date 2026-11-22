"""Continue the three incomplete counter clears to the next NOTE under identical contacts."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import hashlib,json,math,sys,subprocess
p=Path(__file__).resolve().parent
read=lambda f:json.loads(f.read_bytes())
if len(sys.argv)==1:
    rows=read(p/'counter-clear.json')
    jobs=[(str(r['n']),r['id']) for r in rows if r['sustainedBothBeamsClearFromSampleS'] is None]
    def run(j):
        n,k=j
        with (p/f'collection-{k}-{n}.log').open('w') as f:subprocess.run([sys.executable,__file__,n,k],stdout=f,stderr=f,check=True)
    with ThreadPoolExecutor(max_workers=3) as ex:list(ex.map(run,jobs))
else:
    n,k=sys.argv[1:];mission=read(p/f'run-dev-{n}.json')
    i=next(i for i,part in enumerate(mission['parts']) if part['id']==k)
    earlier=mission['parts'][i];later=mission['parts'][i+1]
    through=math.ceil(later['counterClock']-earlier['shotStart']+1)
    base=(p/'counter_tail.py').read_text()
    a="suffix='tail_'+mu+'_'+height";assert base.count(a)==1
    base=base.replace(a,"suffix='collection_'+str(through)")
    a='needle="build=script.split("';assert base.count(a)==1
    base=base.replace(a,"patch+=\"sub('(7 if drive else .55)', '(%d if drive else .55)')\"%through+chr(10)\n"+a)
    sys.argv=[__file__,'dev-'+k,n,'.25','0']
    exec(compile(base,'unchanged shot through next arrival','exec'),dict(__file__=str(p/'counter_tail.py'),through=through))
    sf=p/f'split-collection_{through}-dev-{k}-{n}-6.88.json'
    s=read(sf);old=read(p/f'split-tail_.25_0-dev-{k}-{n}-6.88.json')
    assert old['release']==s['release'] and old['both']==s['both'] and s['trace'][:len(old['trace'])-1]==old['trace'][:-1]
    import numpy as np
    sys.path.insert(0,'C:/Users/ehart/repos/frc-2024-poc/sim/pressure')
    from speaker_score_observer import beam_samples,shadow_margin
    f=Path('C:/Users/ehart/Documents/FRC Native/2024/six-speaker-20261118/field-source/blue-speaker/opening-and-counter.json')
    g=read(f);corners={v['key']:np.array(v['caseEndCornersM']) for v in g['sensorCaseEnds']}
    beams=[beam_samples(corners[a],corners[b]) for a,b in g['beamPairs']]
    tr=[r for r in s['trace'] if r['t']>old['both']];x=np.array([r['q'] for r in tr]);t=np.array([r['t'] for r in tr])
    obs=np.array([[shadow_margin(q,beam)>=0 for beam in beams] for q in x]);j=np.flatnonzero(obs.any(axis=1))
    nextShot=read(p/f'split-tail_.25_0-dev-{later["id"]}-{n}-6.88.json')
    shift=later['shotStart']-earlier['shotStart'];near=abs(t-shift-nextShot['both'])<.04
    y=np.array([r['q'][:3] for r in nextShot['trace']]);t2=np.array([r['t']+shift for r in nextShot['trace']])
    span=(t2<t[-1])&(t2>=t[0]);interp=np.stack([np.interp(t2[span],t,x[:,k]) for k in range(3)],axis=1)
    gap=np.linalg.norm(interp-y[span],axis=1)-.3556
    out=dict(n=n,piece=k,throughS=through,nextPiece=later['id'],nextScoreTimeRelative=shift+nextShot['both'],eitherBeamStillCoveredNearNext=bool(obs[near].any()),bothStillCoveredNearNext=bool(obs[near].all(axis=1).any()),sustainedClearS=float(t[j[-1]+1]) if len(j) and j[-1]+1<len(t) else None,lastPositionM=x[-1,:3].tolist(),smallestCircumsphereSeparationM=float(min(gap)),scope='Continuous single-piece simulation through the next recorded score. Negative sphere separation requires the two torus geometries and coupled dynamics if they touch. Field speed and collector handling are unmeasured.',sha=hashlib.sha256(sf.read_bytes()).hexdigest())
    (p/f'collection-review-{k}-{n}.json').write_text(json.dumps(out,indent=2));print(out)

