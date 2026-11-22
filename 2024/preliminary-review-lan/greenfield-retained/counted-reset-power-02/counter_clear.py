"""Check the two face-defined counter beams at and after the scoring event for every held trajectory."""
from pathlib import Path
import hashlib, json, sys
import numpy as np
here=Path(__file__).resolve().parent
field=Path('C:/Users/ehart/Documents/FRC Native/2024/six-speaker-20261118/field-source/blue-speaker/opening-and-counter.json')
sys.path.insert(0,'C:/Users/ehart/repos/frc-2024-poc/sim/pressure')
from speaker_score_observer import beam_samples,shadow_margin
read=lambda p:json.loads(p.read_bytes())
geom=read(field);ends={r['key']:np.array(r['caseEndCornersM']) for r in geom['sensorCaseEnds']}
beams=[beam_samples(ends[a],ends[b]) for a,b in geom['beamPairs']]
rows=[]
for n in [48,96]:
    run=read(here/f'run-dev-{n}.json')
    for part in run['parts']:
        sf=here/f'split-tail_.25_0-dev-{part["id"]}-{n}-6.88.json';s=read(sf)
        assert hashlib.sha256(sf.read_bytes()).hexdigest()==part['shot']
        later=[r for r in s['trace'] if r['t']>s['both']]
        obsc=np.array([[shadow_margin(np.array(r['q']),beam)>=0 for beam in beams] for r in later])
        ix=np.flatnonzero(obsc.any(axis=1));last=int(ix[-1]) if len(ix) else -1
        clear=float(later[last+1]['t']) if last+1<len(later) else None
        rows.append(dict(n=n,id=part['id'],simultaneous=s['both'],sustainedBothBeamsClearFromSampleS=clear,throughS=later[-1]['t'],obscuredAtLastSample=obsc[-1].tolist()))
(here/'counter-clear.json').write_text(json.dumps(rows,indent=2));print([(r['n'],r['id'],r['sustainedBothBeamsClearFromSampleS']) for r in rows])

