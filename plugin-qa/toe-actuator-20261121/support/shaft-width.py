import runpy,numpy as np,math,json
from pathlib import Path
q=Path(__file__).parent;w=runpy.run_path(str(q/'support.py'));from OCP.BRepBuilderAPI import BRepBuilderAPI_MakePolygon,BRepBuilderAPI_MakeFace
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.gp import gp_Vec,gp_Pnt
def shaft(a,width):
 pg=BRepBuilderAPI_MakePolygon()
 for j in range(6):th=math.radians(a+j*60);pg.Add(gp_Pnt(width/np.sqrt(3)*np.cos(th),width/np.sqrt(3)*np.sin(th),-15))
 pg.Close();return BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(pg.Wire()).Face(),gp_Vec(0,0,32)).Shape()
gear=next(iter(w['load_solids'](q.parent/'REV-41-1332.step')[0].values()));ins=w['load_solids'](q/'bearing.step')[0]['REV-41-1527']
# Native side gear maps [U,V,B]=[+Y,+Z,+X]. Native shaft local maps [+Z,-Y,+X].
out=[]
for a in [0,20,21,21.2,21.4,21.6,21.8,22]:
 for width in [4.88,4.9,4.94,4.98]:
  sh=shaft(a,width);relative=np.array([[0,-1,0],[1,0,0],[0,0,1.]])
  out.append([a,width,w['common'](w['xf'](sh,relative,np.zeros(3)),gear),w['common'](sh,ins)])
print(out);(q/'shaft-width.json').write_text(json.dumps(out))
