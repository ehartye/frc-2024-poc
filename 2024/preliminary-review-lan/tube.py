"""Generate a closed round routing allowance along tangent lines and circular bends."""
import math
import numpy as np
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge,BRepBuilderAPI_MakeWire,BRepBuilderAPI_MakeFace
from OCP.BRepOffsetAPI import BRepOffsetAPI_MakePipe
from OCP.GC import GC_MakeArcOfCircle
from OCP.gp import gp_Pnt,gp_Dir,gp_Ax2,gp_Circ
def lead(waypoints,radius=4.875,bend=50):
    points=np.array(waypoints,dtype=float);wire=BRepBuilderAPI_MakeWire();start=points[0]
    def p(x):return gp_Pnt(*x)
    def line(a,b):
        if np.linalg.norm(a-b)>1e-6:wire.Add(BRepBuilderAPI_MakeEdge(p(a),p(b)).Edge())
    for a,b,c in zip(points,points[1:],points[2:]):
        u=(b-a)/np.linalg.norm(b-a);v=(c-b)/np.linalg.norm(c-b)
        if abs(u.dot(v))>1e-7:raise ValueError('Expected a measured quarter-turn')
        lo=b-bend*u;hi=b+bend*v;mid=b-bend*u+bend*v+bend*(u-v)/math.sqrt(2)
        line(start,lo);arc=GC_MakeArcOfCircle(p(lo),p(mid),p(hi)).Value();wire.Add(BRepBuilderAPI_MakeEdge(arc).Edge());start=hi
    line(start,points[-1]);cross=gp_Circ(gp_Ax2(p(points[0]),gp_Dir(*(points[1]-points[0]))),radius)
    face=BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(cross).Edge()).Wire()).Face()
    pipe=BRepOffsetAPI_MakePipe(wire.Wire(),face);assert pipe.IsDone();return pipe.Shape()

