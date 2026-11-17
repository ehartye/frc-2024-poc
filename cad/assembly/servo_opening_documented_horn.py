"""Exact historical1097+1828+1359 variant; private supplier shapes only for checks."""
import math
import servo_opening_wide_support as wide
s=wide.s;h=wide.h;old=wide.old
SOURCE_SHA={'REV-41-1828.STEP':'63dce40b36e4b0a0861457d4cfe6fdfe29c89a34d4b5d2edbc04ecb47e1d3254','REV-41-1359.STEP':'f0438b538fe18e5afc1e18c95d890106a8549ef90cee73e26bd288ea5bdd5615'}
OFFSETS=[(-6.928203230275509,-4),(6.928203230275509,4)]
def register_horn(shape,theta=0):
 t=h.gp_Trsf();t.SetRotation(h.gp_Ax1(h.gp_Pnt(0,0,0),h.gp_Dir(0,0,1)),-math.pi/2)
 return wide.fast.base.spin(h.move(h.BRepBuilderAPI_Transform(shape,t,True).Shape(),130.5,196.25,185),theta)
def register_screw(shape,seat=131.75,dy=0,dz=0,theta=0):
 t=h.gp_Trsf();t.SetRotation(h.gp_Ax1(h.gp_Pnt(0,0,0),h.gp_Dir(0,0,1)),math.pi/2)
 return wide.fast.base.spin(h.move(h.BRepBuilderAPI_Transform(shape,t,True).Shape(),seat+2,196.25+dy,185+dz),theta)
def local_parts(theta,travel_mm=None):
 parts=[(n,p,k) for n,p,k in wide.local_parts(theta,travel_mm) if n not in ['original M3 horn adapter and bored cam','horn supplier-size envelope'] and not n.startswith(('M3x8 horn screw','candidate M3x8 center'))]
 flange=h.cyl((134,196.25,185),(1,0,0),12,3)
 for dy,dz in OFFSETS:flange=s.cut(flange,h.cyl((133,196.25+dy,185+dz),(1,0,0),1.65,5))
 body=s.fuse(s.fuse(flange,h.cyl((137,196.25,185),(1,0,0),4.5,6)),h.cyl((143,197,185),(1,0,0),6,6))
 body=s.cut(body,h.cyl((133,196.25,185),(1,0,0),3.3,17))
 parts.append(('original1828 flange neck cam',wide.fast.base.spin(body,theta),'original-cam'))
 horn=h.box(126.999,176.249,164.999,7.002,40.002,40.002)
 # AABB enclosure includes source blend fillets; .001mm numerical padding is NOT manufacturing tolerance.
 parts.append(('1828 source-sized conservative envelope',horn,'source-envelope'))
 for label,seat,dy,dz in [('center',131.75,0,0)]+[(str(dy),137,dy,dz) for dy,dz in OFFSETS]:
  p=s.fuse(h.cyl((seat-8,196.25+dy,185+dz),(1,0,0),1.5,8),h.cyl((seat,196.25+dy,185+dz),(1,0,0),3.176,2.001))
  parts.append(('1359 conservative envelope '+label,wide.fast.base.spin(p,theta),'threaded-fastener-envelope'))
 return parts
