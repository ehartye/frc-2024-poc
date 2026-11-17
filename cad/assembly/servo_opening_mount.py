"""Original source-datum servo mounting candidate. Powered holding only, no latch."""
import math,json,pathlib
import spring_cassette_candidate as s
import cassette_opening_cam as cam
h=s.h
SOURCE_SHA='2ae8de8a60ea26b95021ca04b3a71c2f0a68ec0f3b7cafd27d5b0353f536afb2'
# Native -Z output axis becomes cradle +X; shaft plane sourceZ=-43.05.
OFFSET=(-3.05,206.30259329813,140.)
def register(shape):
 t=h.gp_Trsf();t.SetRotation(h.gp_Ax1(h.gp_Pnt(0,0,0),h.gp_Dir(0,1,0)),-math.pi/2)
 return h.move(h.BRepBuilderAPI_Transform(shape,t,True).Shape(),*OFFSET)
def local_parts(theta):
 u=cam.evaluate(theta)['contactTravelMm'];parts=[]
 plate=s.cut(h.box(-14,-29,-32.25,28,58,3),h.box(-10.2,-20.2,-33.25,20.4,40.4,5))
 for x in [-5,5]:
  for y in [-24.75,24.75]:
   plate=s.cut(plate,h.cyl((x,y,-33.25),(0,0,1),2.25,5))
   parts.append((f'opening servo M4 clearance shank {x} {y}',register(h.cyl((x,y,-35.25),(0,0,1),2,12)),'fastener-proxy'))
 parts.append(('opening servo original drilled mounting frame',register(plate),'original-stock'))
 # Source-size envelope only, ears intentionally included; do not use for exact bracket interference.
 parts.append(('opening servo complete source envelope',register(h.box(-10.1,-27.01,-43.56,20.2,54.02,43.93)),'source-envelope'))
 cparts=cam.build(theta)
 parts.append(('opening eccentric cam',h.move(cparts[0][1],0,0,40),'original-cam'))
 # Continuous original follower weldment from the moving cassette top bridge to cam face.
 follower=h.move(cparts[1][1],0,0,40)
 follower=s.fuse(follower,h.box(39.5,203-u,77,16,3,57))
 follower=s.fuse(follower,h.box(39.5,195-u,71,16,11,6))
 parts.append(('opening follower proposed weldment',follower,'original-stock'))
 return parts
