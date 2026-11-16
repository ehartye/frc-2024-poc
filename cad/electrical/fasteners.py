"""Selected #10-32 fastening stacks; no helical threads or preload simulation."""
import math
from a2_primitives import cyl
from OCP.gp import gp_Pnt,gp_Vec
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakePolygon,BRepBuilderAPI_MakeFace
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut

def hex_prism(x,y,z,across_flats,height):
 p=BRepBuilderAPI_MakePolygon();r=across_flats/math.sqrt(3)
 for i in range(6):p.Add(gp_Pnt(x+r*math.cos(math.pi*i/3+math.pi/6),y+r*math.sin(math.pi*i/3+math.pi/6),z))
 p.Close();return BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(p.Wire()).Face(),gp_Vec(0,0,height)).Shape()

def stack(name,x,y,seat_z,nut_top,length,button=False):
 # am-1042 exterior uses supplier0.249in thickness and3/8in wrench size.
 # Screw head uses standard #10SHCS max dimensions as an explicit dimensioned proxy.
 r=4.826/2;nt=6.3246
 if seat_z-length>nut_top-nt-1.5875:raise ValueError('less than two nominal pitches beyond nut')
 shaft=cyl((x,y,seat_z-length),(0,0,1),r,length)
 height=2.5654 if button else 4.826
 head=cyl((x,y,seat_z),(0,0,1),(9.1694 if button else 7.9248)/2,height)
 head=BRepAlgoAPI_Cut(head,hex_prism(x,y,seat_z+(height-1.778 if button else 2.5),3.175 if button else 3.96875,3.5)).Shape()
 nut=hex_prism(x,y,nut_top-nt,9.525,nt)
 nut=BRepAlgoAPI_Cut(nut,cyl((x,y,nut_top-nt-1),(0,0,1),r,nt+2)).Shape()
 return [(name+'-shaft',shaft,'fastener-selected'),(name+'-head',head,'fastener-selected'),(name+'-nut',nut,'fastener-selected')]
