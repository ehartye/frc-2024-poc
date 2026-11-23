"""Reconcile the truncated post-score trajectories without changing a scored launch."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import json,hashlib,subprocess,sys
import numpy as np
p=Path(__file__).resolve().parent
def read(f):return json.loads(f.read_bytes())
def h(f):return hashlib.sha256(f.read_bytes()).hexdigest()
plan=read(p/'plan.json');runs=[read(i) for i in sorted(p.glob('run-dev-*.json'))]
def audit(task):
    tag,n,mu,z=task;old=p/f'split-outerlane-{tag}-{n}-6.88.json';ref=read(old)
    path=p/f'split-tail_{mu}_{z}-{tag}-{n}-6.88.json'
    if not path.exists():
        with path.with_suffix('.log').open('w') as log:subprocess.run([sys.executable,'counter_tail.py',tag,str(n),mu,z],cwd=p,check=True,stdout=log,stderr=log)
    s=read(path);matched={k:s[k]==ref[k] for k in ['sourceCarry','initialState','initialVelocity','rotorsAtFeed','release','exit','opening','beams','both','minBusV']}
    t0=s['feedTime']+1.;end=t0+4.8;trace=s['trace'];sel=[v for j,v in enumerate(trace[:-1]) if trace[j+1]['t']>=t0 and v['t']<=end]
    guard=12*max(np.diff([v['t'] for v in trace]));gap=1.39+plan['envelope'][0][0]-.1778-guard-max(v['q'][0] for v in sel)
    amps=max((11.8-v['busV'])/.027 for v in sel)
    ok=all(matched.values()) and trace[-1]['t']>=end and gap>0 and amps<=30 and s['opening']['fullyThroughByS']<=t0
    result=dict(tag=tag,n=n,mu=mu,floorHeightM=z,original=h(old),extended=h(path),same=matched,endS=trace[-1]['t'],minGapM=gap,afterShotA=amps,resetStart=t0,resetEnd=end,passScreen=bool(ok))
    (p/f'tail-receipt_{mu}_{z}-{tag}-{n}.json').write_text(json.dumps(result,indent=2))
    print(tag,n,mu,z,ok,round(gap,3),flush=True)
if __name__=='__main__':
    tasks=[]
    for d in runs:
        for part in d['parts'][:5]:
            if part.get('scored') and not part['resetParallelEligible']:
                for mu,z in [('.25','0'),('.75','.05')]:tasks.append((d['name']+'-'+part['id'],d['n'],mu,z))
    with ThreadPoolExecutor(max_workers=3) as pool:list(pool.map(audit,tasks))

