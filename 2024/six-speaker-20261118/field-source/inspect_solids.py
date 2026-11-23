from pathlib import Path
import json
import numpy as np
from OCP.BRepTools import BRepTools
from OCP.BRep import BRep_Builder, BRep_Tool
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.TopoDS import TopoDS_Shape, TopoDS
from OCP.TopAbs import TopAbs_FACE, TopAbs_VERTEX
from OCP.TopExp import TopExp_Explorer
from OCP.GeomAbs import GeomAbs_Plane, GeomAbs_Cylinder, GeomAbs_Sphere
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopLoc import TopLoc_Location

base=Path(__file__).parent/'cad-extraction'
def read_shape(filename):
    shape=TopoDS_Shape()
    assert BRepTools.Read_s(shape,str(base/filename), BRep_Builder())
    return shape
def faces(shape):
    e=TopExp_Explorer(shape,TopAbs_FACE)
    while e.More():
        yield TopoDS.Face(e.Current())
        e.Next()
def detail(face):
    surf=BRepAdaptor_Surface(face)
    geom=surf.GetType()
    prop=GProp_GProps()
    BRepGProp.SurfaceProperties_s(face,prop)
    v=TopExp_Explorer(face,TopAbs_VERTEX)
    vertices=set()
    while v.More():
        vertices.add(tuple(BRep_Tool.Pnt_s(TopoDS.Vertex(v.Current())).Coord()))
        v.Next()
    row=dict(geom=geom.name,area=prop.Mass(),center=prop.CentreOfMass().Coord(),vertices=sorted(vertices))
    if geom in (GeomAbs_Plane,GeomAbs_Cylinder,GeomAbs_Sphere):
        primitive={GeomAbs_Plane:surf.Plane,GeomAbs_Cylinder:surf.Cylinder,GeomAbs_Sphere:surf.Sphere}[geom]()
        row['axis']=primitive.Axis().Direction().Coord() if geom!=GeomAbs_Sphere else None
        row['origin']=primitive.Location().Coord()
        if geom!=GeomAbs_Plane:row['radius']=primitive.Radius()
    return row
if __name__=='__main__':
    for i,f in enumerate(faces(read_shape('42SMR.brep'))):
        r=detail(f)
        print(i,r['geom'],round(r['area'],1),np.round(r['center'],2),'axis',np.round(r['axis'],2) if r.get('axis') else None,'rad',round(r.get('radius',0),2))
