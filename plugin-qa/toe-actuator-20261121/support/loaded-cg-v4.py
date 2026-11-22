from pathlib import Path
import numpy as np,json,runpy,hashlib
q=Path(__file__).parent;v=runpy.run_path(str(q/'loads-v4.py'));r,m,c=v['r'],v['m'],v['c'];result={}
for n in (96,192):
 d=q.parent/f'ring-{n}'; ring=d/'ring.npy';raw=np.load(ring);res=[]; src=d/'floor/cycle.json';pieces=json.loads(src.read_bytes())
 for p in pieces['samples']:
  contacts=set(p['ringContacts'])
  if not contacts.intersection({'Ltoe','Rtoe','backtoe','backpost'}):continue
  # Calculate only at actual contact states (25 ms); interpolate instrumented toe displacement rather than implicit commanded motion.
  t=p['t'];j=[np.interp(t,raw[:,0],raw[:,i])*1000 for i in (6,7,8)]
  D=np.array([{'left':[0,j[0],0],'right':[0,-j[1],0],'rear':[-j[2],0,0],None:[0,0,0]}[x['j']]for x in r])
  cg=np.average(c+D,weights=m,axis=0);R=v['np'];a=p['toolAngleRad'];cs=np.cos(a);sn=np.sin(a); rot=np.array([[cs,0,sn],[0,1,0],[-sn,0,cs]])
  ringHead=rot.T@(np.array(p['centerMm'])-np.array([p['robotXY'][0]*1000+p['toolX']*1000,p['robotXY'][1]*1000,p['toolMm']]))
  loaded=(m.sum()*cg+.2413*ringHead)/(m.sum()+.2413);worldLever=rot@(loaded-[-70,0,195]); tau=9.80665*(m.sum()+.2413)*worldLever[0]/1000
  res.append(dict(t=t,contacts=sorted(contacts),onlyHead=not bool(contacts-{'Ltoe','Rtoe','backtoe','backpost'}),headCGmm=cg.tolist(),ringCenterHeadMm=ringHead.tolist(),combinedCGmm=loaded.tolist(),fullGravityNmAboutPitch=float(tau)))
 sel=[x for x in res if x['onlyHead']];result[str(n)]=dict(sourceSHA=hashlib.sha256(src.read_bytes()).hexdigest(),toeSHA=hashlib.sha256(ring.read_bytes()).hexdigest(),rows=res,scope='all mass borne by wrist only while sole observed contacts are on pickup. Shared floor/bed rows retain combined CG but torque is hypothetical.',pureHeadCount=len(sel),absoluteGoverningGravity=max(sel,key=lambda a:abs(a['fullGravityNmAboutPitch'])))
(q/'loaded-cg-v4.json').write_text(json.dumps(result,indent=1))
for n,data in result.items():print(n,data['pureHeadCount'],data['absoluteGoverningGravity'])

