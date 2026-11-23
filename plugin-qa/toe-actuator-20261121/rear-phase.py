from pathlib import Path
import numpy as np,runpy,json,math
q=Path(__file__).parent;w=runpy.run_path(str(q/'gear-phase.py'));get=w['env'];P=get['make'](rz=185);g=next(p for p in P if p.name=='rear pinion');rs=[p for p in P if p.name.startswith('rear seg')]
def vol(qs,ph):
 gear=w['tr'](w['gear'],get['rot'](math.radians(ph))@g.A,g.t)
 return sum(w['common'](gear,w['tr'](w['rack'],p.A,p.t+np.array([-qs,0,0]))) for p in rs)
V={}
for ph in range(12):
 V[ph]=vol(0,ph)
best=min(V,key=V.get);d={x:vol(x,best-math.degrees(x/11.25))for x in [0.,34.,-53.]};print(V,best,d);(q/'rear-gear-phase.json').write_text(json.dumps(dict(home=V,selectedHomePhaseDeg=best,endpoints=d),indent=2))
