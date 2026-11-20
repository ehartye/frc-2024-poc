"""Original OCP primitive helpers for A2 spatial CAD; millimetres. No team CAD copied."""
import argparse,json,math,pathlib
from OCP.gp import gp_Pnt,gp_Dir,gp_Ax1,gp_Ax2,gp_Trsf,gp_Vec
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox,BRepPrimAPI_MakeCylinder,BRepPrimAPI_MakeTorus
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.STEPControl import STEPControl_Writer,STEPControl_Reader,STEPControl_AsIs
from OCP.IFSelect import IFSelect_RetDone
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE,TopAbs_SOLID
from OCP.TopoDS import TopoDS
from OCP.TopLoc import TopLoc_Location
from OCP.BRep import BRep_Tool
ROOT=pathlib.Path(__file__).resolve().parent
COLORS={'frame':'#879baa','bumper':'#235cb5','drive':'#2c3440','mechanism':'#e49b31','support':'#627f96','battery':'#283444','piece':'#ed6533'}
def box(x,y,z,a,b,c):return BRepPrimAPI_MakeBox(gp_Pnt(x,y,z),a,b,c).Shape()
def cyl(p,axis,r,l):return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(*p),gp_Dir(*axis)),r,l).Shape()
def move(s,dx=0,dy=0,dz=0):
 t=gp_Trsf();t.SetTranslation(gp_Vec(dx,dy,dz));return BRepBuilderAPI_Transform(s,t,True).Shape()
def pitch(s,a,p):
 t=gp_Trsf();t.SetRotation(gp_Ax1(gp_Pnt(0,0,0),gp_Dir(0,-1,0)),a);return move(BRepBuilderAPI_Transform(s,t,True).Shape(),*p)
def tube(x,y,z,l,w,h,t,axis):
 outer=box(x,y,z,l,w,h)
 inner=box(x-1,y+t,z+t,l+2,w-2*t,h-2*t) if axis=='x' else box(x+t,y-1,z+t,l-2*t,w+2,h-2*t)
 return BRepAlgoAPI_Cut(outer,inner).Shape()
