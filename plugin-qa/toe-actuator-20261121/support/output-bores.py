from pathlib import Path
import numpy as np,sys
q=Path(__file__).parent;sys.path.insert(0,str(Path.home()/'repos/agent-frc/scripts'));from showcase_contacts import load_solids;from mechanical_motion import bounds
from OCP.TopAbs import TopAbs_FACE
from OCP.TopoDS import TopoDS
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
from stock_sections import entities
for p in [q.parent/'REV-41-1615.STEP']:
 a,ss=load_solids(p)
 for name,solid in a.items():
  print(name)
  rows=set()
  for e in entities(solid,TopAbs_FACE):
   f=TopoDS.Face(e);s=BRepAdaptor_Surface(f)
   if s.GetType()!=GeomAbs_Cylinder:continue
   c=s.Cylinder();o=np.array(c.Location().Coord());axis=np.array(c.Axis().Direction().Coord());i=np.argmax(abs(axis))
   if abs(axis[i])<.999:continue
   br=np.array(bounds(f)).reshape(2,3);row=(round(c.Radius(),3),i,tuple(np.delete(o,i).round(2)),tuple(br[:,i].round(2)))
   if c.Radius()>.65 and i==2: rows.add(str(row).replace('np.float64(', '').replace('np.int64(', ''))
  print(str(rows)[:2100])

