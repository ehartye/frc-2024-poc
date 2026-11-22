"""Take the scored robot to the verified approach and continue with the solved suspended robot."""
from pathlib import Path
import copy, json, math
import numpy as np
p=Path(__file__).resolve().parent;root=p.parent;df=root.parent/'www/data.json'
read=lambda f:json.loads(f.read_bytes())
d=read(df);g=d['concepts']['guided'];r=read(p/'endgame-transfer.json');assert r['remainingIn24S']>0 and r['slew']['pass5mm'] and not any(r['segmentHits'])
def rotate(a):
    a=math.radians(a);co,si=math.cos(a),math.sin(a)
    el=np.eye(4);el[:3,:3]=[[co,0,-si],[0,1,0],[si,0,co]];el[:3,3]=[185,0,501];return el
for side in ['front','rear']:
    parent=next(c for c in g['cases'] if c['key']=='guided-'+side)
    c=copy.deepcopy(parent)
    c.update(id='Sixth shot to climb · '+('forward' if side=='front' else 'rearward')+' CG',key='score-to-climb-'+side,measure=[],positions=[],time=[],chain=[],series={k:[] for k in parent['series']},basis='Travel starts at the lateral firing station on the sixth score. At the stand-off the empty cassette moves from 122° to the tested approach pose. Suspended motion continues from the two-hook physics run, including five seconds on mechanical stops after power cut. The reserved endgame window is 24 seconds. Traffic uses archived peer envelopes.')
    def push(t,xy,m,poses,chain):
        if c['time'] and t<=c['time'][-1]+1e-8:return
        c['time'].append(float(t));c['positions'].append(list(xy));c['chain'].append(chain);c['measure'].append(m)
        for k,v in poses.items():c['series'][k].append(v)
    attach={k for k,pieces in [('arm',['carriage','clutch','drum','keeper','arm','flap','stopLeft','stopRight','cord'])] for k in pieces}
    def poses(q):
        delta=rotate(q)@np.linalg.inv(rotate(125))
        return {k:(delta@np.array([*v[0],0,0,0,1]).reshape(4,4))[:3,:].round(8).flatten().tolist() if k in attach else v[0] for k,v in parent['series'].items()}
    t0=0.
    for start,end,T in zip(r['path'],r['path'][1:],r['legsS']):
        a,b=np.array(start),np.array(end);L=np.linalg.norm(b-a)/1000;peak=min(1.2,math.sqrt(1.4*L));ta=peak/1.4
        for s in np.arange(0,T,.06):
            dist=.7*s*s if s<=ta else L-.7*(T-s)**2 if s>=T-ta else .7*ta*ta+peak*(s-ta)
            push(t0+s,a+(b-a)*dist/L,'Travel from speaker around stage and traffic',poses(122),parent['chain'][0])
        t0+=T
    for s in np.arange(0,1,.05):push(t0+s,r['waiting'],'Set unloaded cassette for chain approach',poses(122+3*s*s*(3-2*s)),parent['chain'][0])
    old=r['earlierRouteArrivalAtWaitS'];cut=r['powerCutFromSixthS']
    for i,t in enumerate(parent['time']):
        if t<old:continue
        clock=t-old+t0+1
        txt=parent['measure'][i].split(' · ')[0] if 'to match cutoff' in parent['measure'][i] else parent['measure'][i]
        push(clock,parent['positions'][i],txt+' · '+f'{clock:.2f} s from sixth score',{k:v[i] for k,v in parent['series'].items()},parent['chain'][i])
    assert abs(c['time'][-1]-r['holdToEndS'])<.001
    c['results'].insert(0,dict(pair=['Sixth score','Stage approach'],status='sampled',witness=dict(reason=f'{len(r["path"])-1} clear legs, {r["driveS"]:.2f} s to the waiting point. Modeled power cut at {cut:.2f} s, {24-cut:.2f} s inside the endgame allocation.')))
    g['cases']=[x for x in g['cases'] if x['key']!=c['key']]+[c]
tmp=df.with_suffix('.transfer');tmp.write_text(json.dumps(d,separators=(',',':')),encoding='utf8');tmp.replace(df)
print(r['powerCutFromSixthS'],r['remainingIn24S'])

