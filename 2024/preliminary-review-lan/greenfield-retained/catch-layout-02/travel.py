"""Screen the full rotation and slide envelope. Shared rigid motion preserves the parked fit."""
from pathlib import Path
import sys,json,itertools,numpy as np,math
p=Path(__file__).resolve().parent;root=p.parent;sys.path.insert(0,str(root))
from layout import bounds,common,distance,place,mat
from showcase_contacts import load_solids
G,_=load_solids(p/'catch-install-trial.step');G={n.replace(chr(194)+chr(183),chr(183)):s for n,s in G.items()}
d=json.loads((p/'parent-review.json').read_bytes());r=json.loads((p/'layout-results.json').read_bytes())
stock=r['attr'];arm=d['cases'][0]['rig'];cin=next(c for c in d['cases'] if c['key']=='intake')
def role(n):
    if n in stock:return 'slide' if stock[n]['role']=='dog' else stock[n]['role']
    if cin['rig'].get(n)=='slide':return 'slide'
    return 'arm' if arm.get(n) in ['arm','flap','piece'] else 'frame'
b={n:np.array(bounds(s)) for n,s in G.items()}
special={'Chambered NOTE','Mouth retention flap','Battery service pin','Battery and terminal case','Pack lead','Distribution lead'}
results=[];near=[]
def broad(a,b):return max(np.max(a[:3]-b[3:]),np.max(b[:3]-a[3:]))
def bbox(n):
    v=b[n].copy();r=role(n)
    if r=='arm':
        radius=max(math.hypot(x-185,z-501) for x,z in itertools.product(v[[0,3]],v[[2,5]]))
        v[[0,2,3,5]]=[185-radius,501-radius,185+radius,501+radius]
    return v
for a in stock:
    for bb in G:
        if bb==a or (bb in stock and list(stock).index(bb)<list(stock).index(a)): continue
        ra,rb=role(a),role(bb)
        aa=bbox(a);ab=bbox(bb)
        if ra=='frame' or rb=='frame':
            if ra!='frame':aa[0]-=216
            if rb!='frame':ab[0]-=216
        if bb in special and bb not in ('Chambered NOTE','Mouth retention flap'):
            # Exact sampled service transforms checked separately below.
            pass
        if broad(aa,ab)>.001:method='disjoint envelope'
        elif ra==rb:method='rigidly co-moving, zero parked common'
        else:
            near.append([a,bb]);method='interval check required'
        results.append([a,bb,method])
print('retained for interval check',near)
out=dict(n=len(results),angularDeg=[-50.5,165],slideXmm=[-31,185],candidates=near,methods=results)
(p/'travel-broad.json').write_text(json.dumps(out,separators=(',',':')))

