from pathlib import Path
import runpy,json,numpy as np,math
w=runpy.run_path(str(Path(__file__).with_name('support.py')))
q=w['q'];rows,old=w['headRows']();xf=w['xf'];shaft=w['cyl']([-70,-218,195],[0,1,0],6.35,436)
rows.append(dict(name='reserved wrist shaft',j=None,shape=shaft))
for lab in ['left','right','rear']:
 if lab=='rear': p=w['box']([-79,-4,6],[-71,4,92]); toe=w['box']([-118,-10,2],[-72,10,6])
 else:
  y=78 if lab=='left' else -78;p=w['box']([6,y-4,6],[14,y+4,92]);Y=98 if y>0 else -98;toe=w['box']([0,Y-23,2],[20,Y+23,6])
 rows.append(dict(name=lab+' original finger',shape=w['union']([p,toe]),j=lab))
BB=np.array([w['bounds'](p['shape']) for p in rows]); records={}
for label,sl,sr in [('neutral',0,0),('capture',34,34),('home',-23,-53)]:
 D=np.array([{'left':[0,sl,0],'right':[0,-sl,0],'rear':[-sr,0,0],None:[0,0,0]}[p['j']] for p in rows])
 S=[xf(p['shape'],np.eye(3),d) if d.any() else p['shape'] for p,d in zip(rows,D)]
 B=BB+np.tile(D,2); collisions=[]
 for i,p in enumerate(rows):
  for j in np.where(np.all(np.minimum(B[i,3:],B[:i,3:])-np.maximum(B[i,:3],B[:i,:3])>.0001,axis=1))[0]:
   a=p['name'];b=rows[j]['name']
   if (' leaf ' in a and ' leaf ' in b) or ('bearing' in a and 'bearing' in b):continue
   mm=w['common'](S[i],S[j])
   if mm>.01:collisions.append([a,b,mm])
 records[label]=collisions
 (q/'screen.json').write_text(json.dumps(records,indent=1));print(label,len(collisions), sorted(collisions,key=lambda r:-r[2])[:12],flush=True)
