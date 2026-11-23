import json,runpy,numpy as np,math
from pathlib import Path
q=Path(__file__).parent;w=runpy.run_path(str(q/'support.py'))
g=list(w['load_solids'](q.parent/'REV-41-1615.STEP')[0].items());bearing=list(w['load_solids'](q/'bearing.step')[0].values());bolt=next(iter(w['load_solids'](q/'M3X20.STEP')[0].values()));common=w['common'];distance=w['distance'];cyl=w['cyl'];X=w['xf'];cut=w['cut'];I=np.eye(3)
out={}
for u in [-1,1]:
 for v in [-1,1]:
  xy=np.array([u*16*math.cos(math.pi/6),v*8.])
  def tube(z1,z2,ri,ro):
   return cut(cyl([*xy,z1],[0,0,1],ro,z2-z1),cyl([*xy,z1-.3],[0,0,1],ri,z2-z1+.6))
  pads=[]
  for base in [11.25]:
   pad=tube(base,12.5,2.85,4)
   pads.append([base,common(pad,g[0][1]),distance(pad,g[0][1]),common(X(pad,I,[0,0,-.02]),g[0][1])])
  screw=X(bolt,I,[*xy,24.5]);thread=[]
  for z in range(2):
   n,h=g[z]; measured=common(screw,h)
   interior=cyl([*xy,-.001],[0,0,1],1.51,7.002);bad=common(cut(screw,interior),h)
   thread.append([n,measured,bad])
  out[str(xy.tolist())]=dict(provisionalBossZUnderCommonGapProbe=pads,threadedSupplierScrew=thread,underheadNative=w['bounds'](screw),screwSHA=w['w']['H'](q/'M3X20.STEP'))
print(out);(q/'gearbox-mount-corrected.json').write_text(json.dumps(out,indent=1))

