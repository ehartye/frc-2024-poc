import runpy,numpy as np,math,json
from pathlib import Path
q=Path(__file__).parent;w=runpy.run_path(str(q/'support.py'));from OCP.BRepBuilderAPI import BRepBuilderAPI_MakePolygon,BRepBuilderAPI_MakeFace
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.gp import gp_Vec,gp_Pnt
def shaft(a):
 pg=BRepBuilderAPI_MakePolygon()
 for j in range(6):th=math.radians(a+j*60);pg.Add(gp_Pnt(5/np.sqrt(3)*np.cos(th),5/np.sqrt(3)*np.sin(th),-15))
 pg.Close();return BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(pg.Wire()).Face(),gp_Vec(0,0,32)).Shape()
gear=next(iter(w['load_solids'](q.parent/'REV-41-1332.step')[0].values()));ins=w['load_solids'](q/'bearing.step')[0]['REV-41-1527']
# Native side gear maps [U,V,B]=[+Y,+Z,+X]. Native shaft local maps [+Z,-Y,+X].
out=[]
for a in range(0,61,2):
 sh=shaft(a);relative=np.array([[0,-1,0],[1,0,0],[0,0,1.]])
 out.append([a,w['common'](w['xf'](sh,relative,np.zeros(3)),gear),w['common'](sh,ins)])
print(out);(q/'clocking.json').write_text(json.dumps(out))
