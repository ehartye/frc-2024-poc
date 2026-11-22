from pathlib import Path
import json,numpy as np,runpy
q=Path(__file__).parent;w=runpy.run_path(str(q/'support.py'));vols=json.loads((q/'roster-v4.json').read_bytes());sh=w['load_solids'](q/'support-only-v4.step')[0];routes=json.loads((q/'loom-all-reservations.json').read_bytes())['routes']
stock=[];unmatched=[];taken=set()
for n,s in sh.items():
 bb=np.array(w['bounds'](s));matches=[r for r in vols if ' leaf ' not in r['name'] and max(abs(np.array(r['bbox'])-bb))<.005 and r['name'] not in taken]
 if len(matches)!=1:unmatched.append([n,[r['name']for r in matches],bb.tolist()]);continue
 row=matches[0];taken.add(row['name']);stock.append((row['name'],s,row['j']))
for j,base in [('left',78),('right',-78),('rear',0)]:
 if j=='rear':p=w['box']([-79,-4,6],[-71,4,92]);toe=w['box']([-118,-10,2],[-72,10,6])
 else:p=w['box']([6,base-4,6],[14,base+4,92]);y=98 if j=='left' else -98;toe=w['box']([0,y-23,2],[20,y+23,6])
 stock.append((j+' finger',w['union']([p,toe]),j))
stock.append(('wrist shaft',w['cyl']([-70,-218,195],[0,1,0],6.35,436),None))
rows=[]
for name,(radius,P) in routes.items():
 for seg,(a,b) in enumerate(zip(P,P[1:])):
  for tag,s,j in stock:
   l=np.array(np.minimum(a,b),float)-radius;h=np.array(np.maximum(a,b),float)+radius
   # Inverse translate stationary box over entire commanded stroke. AABB includes path segment and corner reservation.
   if j=='rear':l[0]-=53;h[0]+=34
   if j=='left':l[1]-=34;h[1]+=23
   if j=='right':l[1]-=23;h[1]+=34
   b0,b1=np.array(w['bounds'](s)).reshape(2,3)
   if np.min(np.minimum(h,b1)-np.maximum(l,b0))<=0:continue
   g=w['box'](l.tolist(),h.tolist());rows.append([name,seg,tag,w['common'](g,s),w['distance'](g,s)])
(q/'loom-neighbors.json').write_text(json.dumps(dict(fixedAndMovingSupportMatched=len(taken),unmatched=unmatched,checks=rows),indent=2));print('mapped',len(taken),'missing',len(unmatched),'rows',len(rows),'violations',[r for r in rows if r[3]>.001])
