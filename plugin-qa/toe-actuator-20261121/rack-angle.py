from pathlib import Path
import sys,math,collections
q=Path(__file__).parent
sys.path.insert(0,sys.argv[1]);from showcase_contacts import load_solids
from stock_sections import entities
from OCP.TopAbs import TopAbs_FACE
from OCP.TopoDS import TopoDS
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Plane
from mechanical_motion import bounds
parts,_=load_solids(q/'2311-0001-0118.step');body=next(iter(parts.values()));rows=collections.Counter()
for e in entities(body,TopAbs_FACE):
 f=TopoDS.Face(e);s=BRepAdaptor_Surface(f)
 if s.GetType()!=GeomAbs_Plane: continue
 n=s.Plane().Axis().Direction()
 if abs(n.Y())>.001 or abs(n.X())<.1 or abs(n.Z())<.1:continue
 if bounds(f)[5]<7:continue
 rows[round(math.degrees(math.atan2(abs(n.Z()),abs(n.X()))),4)]+=1
print(rows)
