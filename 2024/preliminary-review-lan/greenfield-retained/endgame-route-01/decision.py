"""Summarize the bounded review using recorded results and explicit preliminary acceptance tests."""
from pathlib import Path
import hashlib,json,math
p=Path(__file__).resolve().parent;root=p.parent
def src(f):
    raw=f.read_bytes()
    return json.loads(raw),hashlib.sha256(raw).hexdigest()
cases={}
for label in ('pull-front','pull-rear','long-front','long-rear','long-no-catch','equipped-front','equipped-rear'):
    r,digest=src(p/(label+'.json'));held=[q for q in r['trace'] if q['t']>=5.-1e-7]
    cases[label]=dict(input=digest,interval=[held[0]['t'],held[-1]['t']],sampleStep=.025,minClearMm=min(q['z'] for q in held)*1000,peakFloorN=max(q['ground'] for q in held),minHookContactN=[min(q['sides'][k] for q in held) for k in range(2)],engaged=r['latched'],maxPullN=r['maxPullN'],peakNonHookN=r['neighborN'])
    c=cases[label];c['passedHoldScreen']=c['minClearMm']>=10 and c['peakFloorN']<.01 and min(c['minHookContactN'])>0 and c['engaged'] and not any(q>.1 for q in c['peakNonHookN'].values())
camps,h=src(root/'split-field-02/result.json');tries=camps['attempts'];out=dict(preliminaryStatus='held',profile='A',campaign=dict(digest=h,passRuns=[r['attempt'] for r in tries if r['passAttempt']],failedRuns={r['attempt']:r['score'] for r in tries if not r['passAttempt']},untested=camps['untested'],maxPossible=camps['possible'],target=19),climbHoldSamples=cases,parkTime=src(p/'branch.json')[0]['park']['time'])
(p/'decision.json').write_text(json.dumps(out,indent=2));print({k:(round(v['minClearMm'],2),round(v['peakFloorN'],2),v['peakNonHookN']) for k,v in cases.items()})

