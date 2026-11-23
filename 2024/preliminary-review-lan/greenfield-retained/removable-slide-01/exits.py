from pathlib import Path
import sys,numpy as np
here=Path(__file__).resolve().parent
sys.path.insert(0,str(here.parent/'loaded-actuation-01'))
from motor_source import bodies,bounds
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.TopAbs import TopAbs_FACE
from OCP.GeomAbs import GeomAbs_Cylinder,GeomAbs_Plane
it=TopExp_Explorer(bodies[0],TopAbs_FACE)
while it.More():
    f=TopoDS.Face(it.Current());q=BRepAdaptor_Surface(f);B=np.array(bounds(f))
    if max(abs(B[[1,2,4,5]]))>20 and q.GetType() in (GeomAbs_Cylinder,GeomAbs_Plane):
        k=q.Cylinder() if q.GetType()==GeomAbs_Cylinder else q.Plane()
        print(round(k.Radius(),3) if q.GetType()==GeomAbs_Cylinder else 'plane',np.round(k.Axis().Direction().Coord(),3),np.round(B,3))
    it.Next()

