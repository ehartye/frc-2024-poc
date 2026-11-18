"""Additive positive-side relocation; original rejected placement remains immutable."""
import math
import servo_opening_mount as old
s=old.s;h=old.h
# Source output plane at x=130, y=196.25, z=185 after registration.
SHIFT=(90,0,45)
def register(shape):return h.move(old.register(shape),*SHIFT)
def local_parts(theta,travel_mm=None):
 limit=old.cam.evaluate(theta)['contactTravelMm'];u=limit if travel_mm is None else travel_mm
 if isinstance(u,bool) or not isinstance(u,(int,float)) or not math.isfinite(u) or not 0<=u<=limit:raise ValueError('outside unilateral cam boundary')
 parts=[]
 for n,p,k in old.local_parts(theta):
  if 'shank' in n or 'mounting frame' in n or k=='source-envelope':parts.append((n,h.move(p,*SHIFT),k))
 # A continuous original support contacts the frame at its upper edge and
 # the existing outer anchor at z=116. Weld sizing is not yet qualified.
 support=s.fuse(h.box(70,218,116,6,6,86),h.box(70,218,199,49,6,3))
 parts.append(('relocated servo original support',support,'original-stock'))
 # Follower routed beyond x=78 backbone end. Base sits on existing beam top.
 follower=h.box(142,195-u,77,6,11,3)
 follower=s.fuse(follower,h.box(142,203-u,80,6,3,111))
 parts.append(('relocated opening follower',follower,'original-stock'))
 # Original two-bolt horn adapter with axial screw-access bore. The native
 # horn mounting holes are radius1.25 at source (+/-4,+/-6.928203) mm.
 center=(132,196.25,185)
 flange=h.cyl(center,(1,0,0),10,3)
 for dy,dz in [(-4,-6.928203230275509),(4,6.928203230275509)]:
  flange=s.cut(flange,h.cyl((131,196.25+dy,185+dz),(1,0,0),1.25,5))
 neck=h.cyl((135,196.25,185),(1,0,0),3,7)
 camshape=h.cyl((142,196.25+.75,185),(1,0,0),6,6)
 adapter=s.fuse(s.fuse(flange,neck),camshape)
 adapter=s.cut(adapter,h.cyl((131,196.25,185),(1,0,0),1.6,18))
 parts.append(('original horn flange neck eccentric cam',spin(adapter,theta),'original-cam'))
 for dy,dz in [(-4,-6.928203230275509),(4,6.928203230275509)]:
  bolt=h.cyl((128,196.25+dy,185+dz),(1,0,0),1,9)
  parts.append(('horn adapter M2 shank '+str(dy),spin(bolt,theta),'fastener-proxy'))
 # Original conservative envelope only, not redistributed supplier geometry.
 horn=s.fuse(h.cyl((129.5,196.25,185),(1,0,0),19,2.5),h.cyl((126,196.25,185),(1,0,0),5,3.5))
 horn=s.cut(horn,h.cyl((125,196.25,185),(1,0,0),1.6,8))
 parts.append(('horn supplier-size envelope',horn,'source-envelope'))
 # One millimetre axial relief clears native case cap, without claiming spline fit.
 parts=[(n,h.move(p,1,0,0) if ('follower' in n or 'horn' in n) else p,k) for n,p,k in parts]
 return parts

def spin(shape,theta):
 t=h.gp_Trsf();t.SetRotation(h.gp_Ax1(h.gp_Pnt(0,196.25,185),h.gp_Dir(1,0,0)),theta)
 return h.BRepBuilderAPI_Transform(shape,t,True).Shape()

def register_horn(shape,theta=0):
 t=h.gp_Trsf();t.SetRotation(h.gp_Ax1(h.gp_Pnt(0,0,0),h.gp_Dir(0,0,1)),math.pi/2)
 p=h.move(h.BRepBuilderAPI_Transform(shape,t,True).Shape(),133,196.25,185)
 return spin(p,theta)
