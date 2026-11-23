from pathlib import Path
import runpy,numpy as np,math,json
q=Path(__file__).parent;w=runpy.run_path(str(q/'support.py'));g=list(w['load_solids'](q.parent/'REV-41-1615.STEP')[0].items());I=np.eye(3);common=w['common'];cyl=w['cyl'];cut=w['cut'];dist=w['distance'];ret=[]
for x in [-16*math.cos(math.pi/6),16*math.cos(math.pi/6)]:
 for y in [-8,8]:
  # Source Z is toward the pinion. The four centers have an actual 1 mm jump in surrounding cast face height.
  z=11.25 if y<0 else 12.25;pad=cut(cyl([x,y,z],[0,0,1],4,12.5-z),cyl([x,y,z-.1],[0,0,1],2.85,12.7-z))
  regionA=cyl([x,y,7],[0,0,1],1.5001,14.5);regionB=cyl([x,y,1.5],[0,0,1],1.5001,5.5);head=cyl([x,y,21.5],[0,0,1],2.7501,3)
  def obs(s):return [[n,common(s,h)]for n,h in g if min(np.minimum(np.array(w['bounds'](s))[3:],np.array(w['bounds'](h))[3:])-np.maximum(np.array(w['bounds'](s))[:3],np.array(w['bounds'](h))[:3]))>=-.001]
  ret.append(dict(x=x,y=y,padHeightMm=12.5-z,padNominal=obs(pad),padProbe=obs(w['xf'](pad,I,[0,0,-.02])),shaftOutsidePilot=obs(regionA),shaftAnnularPilot=obs(regionB),head=obs(head),negativeShiftedShaftOutsidePilot=obs(w['xf'](regionA,I,[2,0,0]))))
(q/'gearbox-stack-envelope.json').write_text(json.dumps(ret,indent=1));print([(r['padHeightMm'],r['padNominal'],r['shaftOutsidePilot'],r['shaftAnnularPilot'])for r in ret])
