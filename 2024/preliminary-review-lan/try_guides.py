import json,sys,math
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parent;sys.path.insert(0,str(Path(sys.argv[1])/'scripts'))
from showcase_contacts import load_solids,box_gap
from mechanical_motion import bounds,common,transform,distance
b,_=load_solids(root/'supported-layout.step');boxes={n:bounds(s) for n,s in b.items()}
def is_guide(n):
    return n.startswith('Upper ') and any(s in n for s in ('rail 1 cut','rail 2 cut','spring seat','receiver slide','Carriage','guide backing',' nose','spring occupied'))
fixed={n:s for n,s in b.items() if not is_guide(n) and n!='upper web -1'}
case=json.loads((root/'supported-paths.json').read_bytes());inputs=json.loads((root/'build-cache'/'data.json').read_bytes())
result=[];fn=root/'guide-options.json'
for offset in [45,55,65,75,90,110,35]:
    coll=[];clear=[];gaps=[]
    for sign,side in [(1,'left'),(-1,'right')]:
        for n,s in b.items():
            if not is_guide(n) or not n.startswith('Upper '+side):continue
            # Positive Y on left and negative on right: keep the scoring aperture fixed.
            v=transform(s,[dict(kind='translation',travelMm=[0,sign*offset,0])],1);bb=bounds(v)
            for a,t in fixed.items():
                if box_gap(bb,boxes[a])>0.55:continue
                overlap=common(v,t)
                if overlap>.01:coll.append([n,a,round(overlap,2)])
            if box_gap(bb,boxes['Preload NOTE'])<=15:gaps.append(distance(v,fixed['Preload NOTE']))
    result.append(dict(offset=offset,contacts=coll,positive=len(coll),minNearNoteMm=min(gaps,default=None)))
    fn.write_text(json.dumps(result));print(offset,len(coll),[(r[0],r[1]) for r in coll[:3]],flush=True)

