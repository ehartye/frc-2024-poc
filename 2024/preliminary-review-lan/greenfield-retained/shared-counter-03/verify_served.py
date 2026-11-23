"""Check every displayed carry center against its source through the served LAN payload."""
import json,bisect,urllib.request,hashlib
from pathlib import Path
import numpy as np
p=Path(__file__).resolve().parent;cr=p.parent/'counted-reset-power-02';read=lambda f:json.loads(f.read_bytes())
url='http://192.168.1.129:8764/data.json'
with urllib.request.build_opener(urllib.request.ProxyHandler({})).open(url,timeout=30) as response:raw=response.read()
d=json.loads(raw);g=d['concepts']['guided'];f=d['concepts']['field'];mesh={**d['bodies'],**f['add'],**g['add']}['Chambered NOTE'];v=np.array(mesh['v'])
co,si=np.cos(np.radians(125)),np.sin(np.radians(125));center=np.array([185+215*co-50.4*si,0,501+215*si+50.4*co])
assert np.max(abs(v.mean(axis=0)-center))<1.e-5
checks=[]
for n in [48,96]:
    run=read(cr/f'run-dev-{n}.json');c=next(m for m in g['cases'] if m['key']==f'continuous-cycle-{n}')
    t=np.array(c['time']);m=np.array(c['series']['piece']).reshape(-1,3,4);chassis=np.array(c['series']['frame']).reshape(-1,3,4)
    rendered=np.einsum('tij,j->ti',m[:,:,:3],center)+m[:,:,3]
    for leg in run['parts']:
        k=leg['id'];carry=read(cr/f'dev-{k}-{n}.json');shot=read(cr/f'split-tail_.25_0-dev-{k}-{n}-6.88.json')
        start=leg['shotStart']-carry['finish'];samples=carry['trajectory'][::4]+carry['trajectory'][-1:];err=[];indices=[]
        for step in samples:
            ix=int(np.argmin(abs(t-start-step[0])))
            assert abs(t[ix]-start-step[0])<2.e-5
            world=chassis[ix,:,:3]@np.array(step[1:4])*1000
            err.append(float(np.linalg.norm(rendered[ix]-world)));indices.append(ix)
        maxErr=max(err)
        # Quantized field frame matrices, archived previous full-ring mesh and source row rounding have finite precision.
        assert maxErr<.01,(n,k,maxErr)
        join=np.linalg.norm(rendered[indices[-1]]-(np.array(shot['initialState'][:3])-[1.39,5.871,0])*1000)
        assert join<.01
        checks.append(dict(resolution=n,id=k,checked=len(set(indices)),largestDisplayErrorMm=maxErr,displayToShotMm=join,oldFullStrokeErrorMm=1000*carry['approachM']))
(p/'served-trajectory-check.json').write_text(json.dumps(dict(url=url,sha=hashlib.sha256(raw).hexdigest(),checks=checks),indent=2));print(len(checks),sum(r['checked'] for r in checks),max(r['largestDisplayErrorMm'] for r in checks),max(r['displayToShotMm'] for r in checks))

