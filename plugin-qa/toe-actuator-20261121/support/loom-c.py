from pathlib import Path
import json,numpy as np,math,runpy
q=Path(__file__).parent;wt=runpy.run_path(str(q/'support.py'));R=np.eye(3)
# A service loop on the arm side belongs to the arm model. End this scope in the outboard wrist transfer area.
# Each switch bundle has space for three independent contacts, cable diameter reservation 6 mm inclusive.
routes={'left':(3,[[-1,20,75],[-20,23.5,75],[-43,23.5,75],[-43,114,75],[-56,129,75],[-70,129,90],[-70,129,171]]),
'right':(3,[[-1,-23.5,75],[-20,-23.5,75],[-43,-23.5,75],[-43,-114,75],[-56,-129,75],[-70,-129,90],[-70,-129,171]]),
'rear':(3,[[12,-10.93,62],[12,-24.5,68],[12,-24.5,80],[-16,-24.5,80],[-54,-24.5,80],[-54,-114,80],[-66,-126,80],[-81,-138,95],[-81,-138,171]])}
# Rectangular passage reservations allow route evolution; compare three individual actual rigid connector withdrawal gauges separately.
bits={}
for k,(radius,points)in routes.items():
 for i,(a,b)in enumerate(zip(points,points[1:])):
  lo=np.minimum(a,b)-radius;hi=np.maximum(a,b)+radius;bits[f'{k}/{i}']=(None,lo,hi)
path=(q/'head-path.py').read_text();path=path[path.index('res={}'):].replace("'head-path.json'","'loom-ring-c.json'")
cg=runpy.run_path(str(q.parent/'ring-fit.py'));ns={**cg,'q':q,'parent':q.parent,'bits':bits};exec(compile(path,'<loom-ring>','exec'),ns)
fixed=json.loads((q/'roster-v4.json').read_bytes());violations={}
for name,(_,lo,hi)in bits.items():
 obs=[]
 for r in fixed:
  b=np.array(r['bbox']).reshape(2,3);d=np.zeros(3)
  if r['j']=='rear':b[0,0]-=34;b[1,0]+=53
  if r['j']=='left':b[0,1]-=23;b[1,1]+=34
  if r['j']=='right':b[0,1]-=34;b[1,1]+=23
  depth=np.minimum(b[1],hi)-np.maximum(b[0],lo)
  if np.min(depth)>.001:obs.append([r['name'],depth.tolist()])
 violations[name]=obs
(q/'loom-reservations-c.json').write_text(json.dumps(dict(routes=routes,broadPhase=violations,scope='Preliminary envelopes; overlapping common runs must use a larger multicore loom. No motor lead routing, bend dynamics, clips or 2R swing qualification.'),indent=1))


