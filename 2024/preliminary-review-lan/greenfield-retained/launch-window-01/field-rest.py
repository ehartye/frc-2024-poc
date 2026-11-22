"""Identify the field surfaces that the two missed pieces actually end against."""
from pathlib import Path
import sys,json,numpy as np
p=Path(__file__).resolve().parent;root=p.parent
src=(root/'mission-physics/shot.py').read_text();exec(compile(src[:src.index('def shot(')],'scoring field imports','exec'))
s=SpeakerSurfaces(field);objects={d['key']:d['name'] for d,*_ in s.parts}
out={}
for f in [p/'split-ready-dev-center-high-96-6.74.json',p.parent/'split-field-02/split-ready-R01-center-low-96-6.88.json']:
    r=json.loads(f.read_bytes());phi=np.arange(96)*2*np.pi/96;nodes=.1524*np.stack([np.cos(phi),np.sin(phi),phi*0],axis=1);trace=[q for q in r['trace'] if q['t']>=6.5];touch={};center=[]
    for q in trace:
        v=np.asarray(q['q']);xyz=nodes@rotation(v[3:]).T+v[:3];center.append(v[:3])
        for i,subset in s.candidates(v[:3],xyz,.1524,.0254):
            part,dist,_=s.nearest(i,xyz[subset]);depth=.0254-min(dist)
            if depth>0:
                k=part['key']+' / '+part['name'];entry=touch.setdefault(k,[0,0]);entry[0]+=1;entry[1]=max(entry[1],1000*depth)
    out[f.name]=dict(lastHalfSecondPositionRangeMm=(1000*np.ptp(center,axis=0)).tolist(),lastCenterM=center[-1].tolist(),samples=len(trace),terminalContacts=touch,scope='Source-registered field surfaces within the finite tube radius at recorded NOTE poses. Normal force is not inferred from discrete contacts.')
(p/'field-rest.json').write_text(json.dumps(out,indent=2));print(out)

